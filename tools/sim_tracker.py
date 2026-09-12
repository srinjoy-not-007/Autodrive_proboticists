"""Offline closed-loop simulator for the path trackers.

Not a replacement for driving it — every number here still has to be confirmed
in the simulator — but the one question this answers honestly is *whether a
tracker cuts a corner*, because a cut is geometry and geometry is the part a
kinematic model gets right.  It exists so that ``mpc2`` could be compared with a
faithful port of ``mpc``'s control law on the same line, at the same speeds,
through the same actuator, without touching a running container.

What is modelled, and where each number comes from (all measured 2026-09-11
unless noted):

* physics at 1000 Hz, as the simulator runs it;
* sensors every 90 ms (11.1 Hz) — pose, gyro, encoder speed, steering angle —
  which is also the rate at which the bridge picks a command up;
* command path: the latched command is sampled at a frame boundary and then
  takes a further 0.090 s to reach the actuator, so command-to-movement is
  0.090–0.180 s with a 0.135 s median, which is exactly what latency_probe.py
  measured;
* actuator slews at 3.2 rad/s to the commanded angle (``vehicle.STEER_RATE``);
* throttle is a speed servo: target = 24.6 * throttle, about 6 m/s2 of
  authority in both directions, so zero throttle decelerates at ~6 m/s2;
* front tyre delivers ~0.92 of the curvature the steering angle asks for up to
  9.9 m/s2 of lateral acceleration and then collapses abruptly (skidpad in the
  ``-explore`` world);
* collisions are the real car footprint against the map's distance field.

Usage (from the repo root):

    .venv/bin/python tools/sim_tracker.py                       # both trackers
    .venv/bin/python tools/sim_tracker.py --line data/racelines/track_v2_safe_ay6.0.csv \
        --laps 6 --tracker mpc2 --plot /tmp/mpc2.png

Reproducing the live rate failure (mpc3 on porto_mesh_mc0.63_ay6.75, 2026-09-12
logs).  `--plant-realistic` is the identified speed loop plus both encoder
channels; `--speed-diag` prints the lag decomposition in the same metric the
logs are read with; `--pose-lon-bias` injects the measured along-axis
localization offset, which is what actually produces the hairpin collisions:

    .venv/bin/python tools/sim_tracker.py --tracker mpc3 \
        --line data/racelines/porto_mesh_mc0.63_ay6.75.csv --map data/maps/porto_mesh \
        --laps 20 --pose-noise 0.03 --plant-realistic --speed-diag \
        --frame 0.104 --pose-lon-bias 0.00      # 0 collisions / 100 laps, 5 seeds
    ...  --frame 0.088 --pose-lon-bias 0.15      # 64 / 100, all at s 11.1-11.8

The structural alternative to scaling the per-period constants is to stop
solving in the pose callback at all (`-r control_mode=timer`, `-r control_hz=20`):
the same solve on a fixed wall clock, the pose callback only storing the
estimate.  Rate invariance is then a property of the schedule rather than of a
tune, and is checked by running the SAME command at three sensor periods:

    for f in 0.088 0.097 0.104; do .venv/bin/python tools/sim_tracker.py \
        --tracker mpc3 --line data/racelines/porto_mesh_mc0.63_ay6.75.csv \
        --map data/maps/porto_mesh --laps 12 --plant-realistic --enc-alpha 0.35 \
        --pose-noise 0.03 --frame $f -r control_mode=timer; done
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from pit_crew import vehicle                                    # noqa: E402
from pit_crew.mpc2_core import (Actuator, CurvatureBias, MPC2,   # noqa: E402
                                PoseFilter, RefPath, wrap)
from pit_crew.mpc3_core import (ILC, MPC3, RateControl,            # noqa: E402
                                RATE_NOMINAL, RATE_TERMS,
                                corridor_clearance)
from pit_crew.throttle2 import ServoThrottle                     # noqa: E402

DT = 0.001                  # physics step, s
FRAME = 0.090               # sensor / command-pickup period, s
EXTRA_DEAD = 0.090          # bridge -> actuator, on top of the frame quantum
# Scan time -> /pit_crew/pose on the wire: the particle filter's own runtime.
# The pose carries the scan stamp, so the delay is not lost, but it does mean
# the freshest steering sample is newer than the pose being optimised from.
POSE_LATENCY = 0.045
SERVO_SPEED_PER_THROTTLE = 24.6
SERVO_A = 6.0               # m/s2 of authority, both directions
SERVO_K = 8.0               # 1/s, how hard the servo chases its target
# The speed loop as CALIBRATED against the live logs, rather than as guessed.
# The values above make the plant follow the speed plan almost perfectly --
# measured, 0.30-0.50 m of spatial lag and 0.10 m/s of residual against a plan
# the real car lags by 1.45 m with 0.265 m/s residual, and a peak of 6.54 m/s
# where the real car reaches 6.01 against the same 6.91 m/s plan.  That gap is
# not cosmetic: the collision mechanism at 11 Hz is arriving at the hairpin
# too fast because the speed loop's transport lag (which was acting as a free
# corner-entry brake) shrinks, and a plant with no lag cannot show it at all.
# `--servo-cal` selects these; the defaults above are left untouched so every
# earlier number in docs/STATUS.md stays reproducible.
# Best fit found so far by grid: at 0.104 s it gives 0.60 m of lag, 0.249 m/s
# of de-lagged residual and a 6.13 m/s peak, against the log's 1.45 m, 0.265
# and 6.01.  The residual and the peak match; the LAG does not, and that is
# the part the collision mechanism needs, so this is a partial calibration and
# is NOT yet a gate the rate work can be judged against.  What is still
# missing is most likely inside `ServoThrottle._predict`, whose servo model
# (K 18) is much stiffer than the plant this fit wants (K 6): the resulting
# over-prediction is a systematic under-command, i.e. lag.
SERVO_CAL = dict(servo_k=6.0, a_accel=5.0, a_brake=4.5, drag=0.273)
# `--plant-realistic`: the speed loop IDENTIFIED against the clean 100-lap log
# rather than grid-searched against a spatial-lag statistic.  The log's
# `throttle` column is the command the SIMULATOR reports in force, so fitting
# `dv/dt = clip(K*(24.6*u - v), -dmax, amax) - drag*v` against the ground-truth
# speed (d/dt of the IPS path length) identifies the plant with no controller
# model in the loop at all: `tools/sim_servo.py fit
# data/logs/race_20260912_015615.csv` gives K 18, tau 0 (w.r.t. that column),
# a_max 5.50, a_min -4.00, drag 0.273 at an RMSE of 0.087 m/s -- i.e. the servo
# really is first-order-with-K-18, and SERVO_CAL's K 6 is an artefact of having
# fitted the wrong observable (see `--speed-diag`).
PLANT_REAL = dict(servo_k=18.0, a_accel=5.5, a_brake=4.0, drag=0.273)
# The race logger's `speed` column is NOT the plant speed and NOT
# `vehicle.SpeedEstimator` either: `src/tools/race_logger.py` runs its own
# per-encoder-message low pass, `v += 0.3 * clip(v_enc - v, -2, 2)`.  Its lag is
# most of the "1.45 m" live speed lag, so the harness models the channel
# explicitly and reports both, or the two can never be compared.
LOG_ENC_ALPHA = 0.30
LOG_ENC_CLAMP = 2.0
# The wheel encoder over-reads the ground speed under drive: mean encoder speed
# is 4.096 m/s against 3.943 m/s of ground truth on the 100-lap log (+3.9%),
# which is wheel slip and not filter lag.
ENC_SLIP = 1.039
GRIP_RATIO = 0.95           # of asked curvature, well inside the envelope
# Sustained lateral acceleration the car actually delivers ON TRACK.  The
# skidpad's 9.9 m/s2 is a TRANSIENT peak -- the Unity sideways tyre peaks at
# mu 1.0 at 0.01 slip and falls to an asymptote of 0.5 by 0.1 slip, and a
# corner is held long enough to sit near the asymptote.  The first version of
# this simulator used the skidpad's 9.9 and therefore never saturated on an
# ay-6.0 profile, which is why it missed the failure that the first live mpc2
# run then produced (slid wide at both hairpins, achieved/asked curvature 0.74).
#
# CALIBRATED against the ported mpc.py, not derived.  On the corrected mesh map
# (no pose bias) the live mpc runs ay 6.5 clean and takes 5 collisions in 32
# laps at ay 7.0; the port matches that at AY_MAX 6.5.  On the OLD map, with a
# 0.15 m inside pose bias injected, the matching value was 5.5 -- use
# `--ay-max 5.5 --pose-bias-inside 0.15` to reproduce that configuration.  The
# tyre asymptote argument alone gives 4.9, at which the port crashes nine times
# in eight laps and the live car does not.
AY_MAX = 6.5
AY_KNEE = 0.85              # fraction of AY_MAX where saturation starts


# ---------------------------------------------------------------------------
# plant
# ---------------------------------------------------------------------------

class Plant:

    # --- opt-in steering-actuator laws (`--plant-actuator`) ----------------
    # 'slew' is what this harness has always simulated: a 3.2 rad/s rate limit
    # in WALL CLOCK, plus `EXTRA_DEAD` of transport delay.  It is rate
    # invariant, so it can never show a plant that behaves differently at a
    # different telemetry period -- which is exactly the hypothesis the
    # display runs raised.  Two alternatives are available, both default-off:
    #
    #   'pole'  first order with `tau`, no rate limit.  This is what the
    #           staircase log actually identifies (dead 0.12-0.16 s, tau
    #           0.06-0.08 s, any slew ceiling above ~6 rad/s fits equally
    #           well; a 3.2 rad/s ceiling fits WORSE, rmse 0.0414 against
    #           0.0406).  Still wall-clock, so still rate invariant.
    #   'tick'  the rate-DEPENDENT hypothesis: the actuator advances at most
    #           `tick_frac` of the remaining error, or `steer_rate * frame`
    #           rad, once per telemetry tick.  Its effective slew in rad/s is
    #           then 1/frame, i.e. 1.8x faster at 0.058 s than at 0.104 s.
    #
    # Nothing here runs unless a flag asks for it; `slew` with the module
    # defaults reproduces every earlier number byte for byte.
    def __init__(self, x, y, yaw, v=0.0, ay_max=AY_MAX, servo_k=SERVO_K,
                 a_accel=SERVO_A, a_brake=SERVO_A, drag=0.0,
                 actuator='slew', steer_rate=None, tau=0.0, dead=None,
                 frame=FRAME, tick_frac=1.0):
        self.x, self.y, self.yaw, self.v = x, y, yaw, v
        self.ay_max = float(ay_max)
        self.servo_k = float(servo_k)
        self.a_accel = float(a_accel)
        self.a_brake = float(a_brake)
        self.drag = float(drag)
        self.delta = 0.0                 # actual steering angle, rad
        self.cmd_queue = []              # (t_effective, steer_cmd)
        self.steer_target = 0.0
        self.throttle = 0.0
        self.kappa = 0.0
        self.actuator = str(actuator)
        self.steer_rate = (vehicle.STEER_RATE if steer_rate is None
                           else float(steer_rate))
        self.tau = float(tau)
        self.dead = EXTRA_DEAD if dead is None else float(dead)
        self.frame = float(frame)
        self.tick_frac = float(tick_frac)
        self._tick_budget = 0.0          # rad left in this telemetry tick

    def accept_frame(self, t, steer_cmd, throttle):
        """The bridge picks the latched command up; it bites `dead` later."""
        self.cmd_queue.append((t + self.dead, float(steer_cmd), float(throttle)))
        if self.actuator == 'tick':
            # One telemetry tick, one allowance.  The command arrives on the
            # same wire as the frame, so the allowance is renewed on the frame.
            self._tick_budget = self.steer_rate * self.frame

    def step(self, t):
        while self.cmd_queue and self.cmd_queue[0][0] <= t:
            _, s, th = self.cmd_queue.pop(0)
            self.steer_target, self.throttle = s, th
        err = self.steer_target - self.delta
        if self.actuator == 'pole':
            move = err * (1.0 - math.exp(-DT / max(self.tau, 1e-6)))
            lim = self.steer_rate * DT
            move = max(-lim, min(lim, move))
        elif self.actuator == 'tick':
            want = err * self.tick_frac
            lim = min(self._tick_budget, self.steer_rate * DT * 1e9)
            move = max(-lim, min(lim, want))
            self._tick_budget = max(0.0, self._tick_budget - abs(move))
        else:
            lim = self.steer_rate * DT
            move = max(-lim, min(lim, err))
        self.delta += move
        self.delta = max(-vehicle.MAX_STEER, min(vehicle.MAX_STEER, self.delta))

        k_ask = math.tan(self.delta) / vehicle.WHEELBASE
        ay = self.v * self.v * abs(k_ask) * GRIP_RATIO
        knee = AY_KNEE * self.ay_max
        if ay > knee:
            span = max(self.ay_max - knee, 1e-6)
            ay = knee + span * (1.0 - math.exp(-(ay - knee) / span))
        self.kappa = (math.copysign(ay, k_ask) / max(self.v * self.v, 1e-3)
                      if abs(k_ask) > 1e-9 else 0.0)

        v_tgt = SERVO_SPEED_PER_THROTTLE * self.throttle
        a = max(-self.a_brake, min(self.a_accel,
                                   self.servo_k * (v_tgt - self.v)))
        a -= self.drag * self.v
        self.v = max(-3.0, self.v + a * DT)
        yr = self.v * self.kappa
        ym = self.yaw + 0.5 * yr * DT
        self.x += self.v * math.cos(ym) * DT
        self.y += self.v * math.sin(ym) * DT
        self.yaw = wrap(self.yaw + yr * DT)
        return yr


# ---------------------------------------------------------------------------
# tracker A: mpc2, driving the real pit_crew.mpc2_core classes
# ---------------------------------------------------------------------------

class MPC2Sim:
    """The control law of src/pit_crew/mpc2.py with the ROS plumbing removed.

    Anything that changes here must change there and vice versa; the solver,
    the actuator model and the reference path are the shipped objects, only the
    callbacks are re-implemented.
    """

    name = 'mpc2'
    rate = None                      # solves once per sensor frame

    def __init__(self, path, **kw):
        self.path = path
        self.P = dict(cmd_dead_time=0.135, extra_latency_s=0.045,
                      ay_limit=6.0, cap_use_measured=1, cap_ff_headroom=1.15,
                      cap_preview_min_m=1.0,
                      brake_horizon_s=0.35, throttle_gain=0.040,
                      brake_gain=0.05, use_steer_feedback=True,
                      use_measured_speed=False, apply_ahead_s=0.09,
                      pose_blend=0.7, blend_gate_m=0.5,
                      kappa_bias_gain=0.8, kappa_bias_max=0.10,
                      steer_fb_age=0.0)
        solver_keys = ('horizon', 'step_dt', 'min_step_m', 'iters', 'w_lat',
                       'w_yaw', 'w_steer', 'w_rate', 'w_jerk', 'w_cut',
                       'w_lat_T', 'w_yaw_T', 'w_barrier', 'barrier_margin',
                       'cut_offset_m', 'cut_kappa_ref',
                       'steer_rate', 'apply_ahead_s')
        skw = {k: kw.pop(k) for k in list(kw) if k in solver_keys}
        self.P.update(kw)
        skw.setdefault('apply_ahead_s', self.P['apply_ahead_s'])
        self.mpc = MPC2(path, **skw)
        self.act = Actuator(dead_time=self.P['cmd_dead_time'])
        self.filt = PoseFilter(self.act, blend=self.P['pose_blend'],
                               gate_m=self.P['blend_gate_m'])
        self.bias = CurvatureBias(gain=self.P['kappa_bias_gain'],
                                  limit=self.P['kappa_bias_max'])
        self.thr = vehicle.ThrottleController(ff_gain=self.P['throttle_gain'],
                                              brake_gain=self.P['brake_gain'])
        self.speed = 0.0
        self.idx = None
        self.e_lat = 0.0
        self.solve_ms = []

    def plan_ay(self, i0, v):
        P = self.path
        span = max(self.P['cap_preview_min_m'], v * self.mpc.N * self.mpc.dt)
        n = max(2, int(span / max(P.spacing, 1e-3)))
        idx = (i0 + np.arange(n)) % P.n
        return float((P.v_prof[idx] ** 2 * np.abs(P.kappa[idx])).max())

    def on_frame(self, t, gyro, speed, steer_fb):
        """Everything the bridge publishes at the frame, immediately."""
        self.speed_meas = speed
        self.bias.update(gyro, speed, steer_fb)
        if self.P['use_steer_feedback']:
            self.act.snap(t - self.P['steer_fb_age'], steer_fb)

    def on_pose(self, t_now, t_stamp, x, y, yaw):
        """The localized pose: stamped at the scan, delivered later."""
        if self.speed <= 0.0:
            self.speed = float(self.path.v_prof[
                self.path.nearest(x, y, None, yaw=yaw)])
        fx, fy, fyaw, _ = self.filt.update(t_stamp, x, y, yaw,
                                           max(self.speed, 0.4))
        self.meas = (t_stamp, fx, fy, fyaw)

    def update(self, t):
        import time
        t0 = time.perf_counter()
        ts, x, y, yaw = self.meas
        v = max(self.speed, 0.4)
        t_bite = t + self.P['cmd_dead_time'] + self.P['extra_latency_s']
        kb = self.bias.value
        x, y, yaw, delta0 = self.act.predict(
            x, y, yaw, ts, t_bite, v,
            kappa_of=lambda d: math.tan(d) / vehicle.WHEELBASE + kb)
        i0 = self.path.nearest(x, y, self.idx, yaw=yaw)
        self.idx = i0
        pyaw = float(self.path.yaw[i0])
        dx, dy = x - self.path.xy[i0, 0], y - self.path.xy[i0, 1]
        e_lat = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
        e_yaw = wrap(yaw - pyaw)
        self.e_lat = e_lat
        v_cap = v
        if self.P['cap_use_measured'] and self.speed_meas > 0.1:
            v_cap = max(v, self.speed_meas)
        cap = vehicle.MAX_STEER
        if self.P['ay_limit'] > 0:
            cap = min(cap, math.atan(vehicle.WHEELBASE * self.P['ay_limit']
                                     / max(v_cap * v_cap, 0.25)))
        ay_cap = max(self.P['ay_limit'],
                     self.P['cap_ff_headroom'] * self.plan_ay(i0, v))
        cap = min(vehicle.MAX_STEER,
                  math.atan(vehicle.WHEELBASE * ay_cap
                            / max(v_cap * v_cap, 0.25)))
        cmd, _ = self.mpc.solve(e_lat, e_yaw, delta0, i0, v, cap=cap,
                                bias=self.bias.value)
        n_ahead = max(2, int(max(0.2, self.speed * self.P['brake_horizon_s'])
                             / max(self.path.spacing, 1e-3)))
        idx = (i0 + np.arange(n_ahead)) % self.path.n
        self.speed = float(self.path.v_prof[idx].min())
        v_fb = self.speed_meas if self.P['brake_gain'] > 0 else None
        th = float(self.thr.command(self.speed, v_fb, 0.0))
        self.act.record(t, cmd)
        self.solve_ms.append((time.perf_counter() - t0) * 1e3)
        return cmd, th


# ---------------------------------------------------------------------------
# tracker C: mpc3, driving the real pit_crew.mpc3_core / throttle2 objects
# ---------------------------------------------------------------------------

class MPC3Sim:
    """The control law of src/pit_crew/mpc3.py with the ROS plumbing removed.

    Same rule as MPC2Sim: the solver, the actuator model, the reference path and
    the throttle law are the shipped objects; only the callbacks are
    re-implemented.  Anything that changes here must change in mpc3.py too.
    """

    name = 'mpc3'
    rate = None                      # solves once per sensor frame ('per_pose')
    # `control_mode` 'timer' sets `self.rate` to `control_hz` instead, and then
    # `run()` drives `update()` off the wall clock exactly as it already does
    # for `LegacyMPCSim` -- the plant has always stepped at 1 kHz, so the only
    # thing that changes is the solve schedule.  The command is still PICKED UP
    # by the bridge at a sensor frame (that is the simulator, not the tracker),
    # so what timer mode buys is a command that is at most one tick old at the
    # pickup instead of one sensor period old.

    def __init__(self, path, **kw):
        self.path = path
        self.P = dict(cmd_dead_time=0.135, use_steer_feedback=True,
                      steer_fb_age=0.0, pose_blend=1.0, blend_gate_m=0.5,
                      ay_limit=9.0, cap_ff_headroom=1.15, cap_preview_min_m=1.0,
                      cap_use_measured=True, max_predict_s=0.35,
                      v_meas_min=0.2, v_start=0.5, servo_k=18.0, servo_drag=0.273,
                      servo_a_auth=4.5, servo_a_accel=5.0, servo_k_p=0.0,
                      servo_head_max=0.5, servo_u_max=0.60, servo_v_floor=1.0,
                      servo_gain=24.6,
                      # --- rate adaptivity (opt-in, identity when off) -----
                      rate_adaptive=0, rate_nominal=RATE_NOMINAL,
                      rate_window=20, rate_min_change=0.002,
                      rate_terms=','.join(RATE_TERMS),
                      rate_lo=0.5, rate_hi=1.5,
                      stale_pose_s=0.35, enc_alpha=0.35,
                      # --- ILC (opt-in) ------------------------------------
                      ilc_enable=0, ilc_bin_m=0.25, ilc_k_gain=0.30,
                      ilc_q_span_m=0.50, ilc_k_max=0.15, ilc_shift_max_m=0.08,
                      ilc_shift_len_m=1.50, ilc_clear_target=0.30,
                      ilc_clear_band=0.10, ilc_clear_loss=0.05,
                      # Matches the node's default: the speed table costs more
                      # lap time than the steering table saves (12 laps on the
                      # safe line, 7.86 -> 8.09 s with it, 7.86 -> 7.76 s
                      # without).  Pass `ilc_v_enable=1` to re-measure it.
                      ilc_v_enable=0, ilc_v_down=1.0, ilc_v_up=0.005,
                      ilc_v_lo=0.85, ilc_v_hi=1.05, ilc_store='',
                      # --- solve schedule (opt-in; see mpc3.py) ------------
                      control_mode='per_pose', control_hz=20.0,
                      timer_apply_ahead_s=0.0, timer_sync_dead=1,
                      # --- teleport recovery (opt-in; see mpc3.py) ---------
                      v_pose_gate=0.0, v_pose_min_dt=0.03, reindex_after=0)
        solver_keys = ('horizon', 'step_dt', 'min_step_m', 'iters', 'outer',
                       'w_lat', 'w_yaw', 'w_steer', 'w_rate', 'w_jerk',
                       'w_lat_T', 'w_yaw_T', 'w_barrier', 'barrier_margin',
                       'yaw_gain', 'ay_deliver', 'sat_knee', 'ay_budget',
                       'a_brake', 'a_accel', 'e_free', 'k_e', 'v_mod_min',
                       'thr_lead_s', 'thr_lead_mode', 'thr_enc_tau', 'delay_s',
                       'apply_ahead_s', 'apply_floor_steps', 'steer_rate',
                       'w_dcmd', 'cmd_tau', 'cmd_notch', 'e_dead',
                       'steer_tau', 'model_cmd_tau', 'lead_mode', 'lead_m0',
                       'lead_k', 'lead_min_m')
        skw = {k: kw.pop(k) for k in list(kw) if k in solver_keys}
        for k in ('horizon', 'iters', 'outer'):
            if k in skw:
                skw[k] = int(round(skw[k]))
        self.P.update(kw)
        self.mpc = MPC3(path, **skw)
        self.act = Actuator(dead_time=self.P['cmd_dead_time'],
                            steer_tau=self.mpc.cfg['steer_tau'])
        self.filt = PoseFilter(self.act, blend=self.P['pose_blend'],
                               gate_m=self.P['blend_gate_m'])
        self.thr = ServoThrottle(
            K=self.P['servo_k'], drag=self.P['servo_drag'],
            tau=self.P['cmd_dead_time'], a_auth=self.P['servo_a_auth'],
            a_accel=self.P['servo_a_accel'], k_p=self.P['servo_k_p'],
            head_max=self.P['servo_head_max'], lead_max=0.0,
            ay_limit=self.mpc.cfg['ay_budget'], v_floor=self.P['servo_v_floor'],
            u_max=self.P['servo_u_max'], servo_gain=self.P['servo_gain'])
        self.v_plan0 = 0.0
        self._enc_seen = False
        self._pose_prev = None           # (stamp, x, y) of the last pose
        self._v_pose = None
        self._gate_run = 0
        self.meas = None                 # (t_stamp, x, y, yaw); None = no pose
        self._last = (0.0, 0.0)          # last (steer, throttle) published
        self.speed_meas = 0.0
        self.idx = None
        self.e_lat = 0.0
        self.solve_ms = []
        self.v_cmd = 0.0
        self.rate_ctl = RateControl(
            self.mpc, self.act, enabled=bool(self.P['rate_adaptive']),
            filt=self.filt, speed_est=None,
            nominal=float(self.P['rate_nominal']),
            window=int(self.P['rate_window']),
            terms=tuple(str(self.P['rate_terms']).split(',')),
            min_change=float(self.P['rate_min_change']),
            rate_lo=float(self.P['rate_lo']),
            rate_hi=float(self.P['rate_hi']),
            cmd_dead_time=self.P['cmd_dead_time'],
            delay_s=self.mpc.cfg['delay_s'],
            apply_ahead_s=self.mpc.cfg['apply_ahead_s'],
            thr_lead_s=self.mpc.cfg['thr_lead_s'],
            max_predict_s=self.P['max_predict_s'],
            stale_pose_s=self.P['stale_pose_s'],
            enc_alpha=self.P['enc_alpha'], pose_blend=self.P['pose_blend'],
            enc_tau=self.mpc.cfg['thr_enc_tau'])
        self.ilc = None
        self.ilc_log = []
        self.idx_prev = None
        if self.P['ilc_enable']:
            self.ilc = ILC(
                float(path.s[-1] + path.spacing),
                bin_m=self.P['ilc_bin_m'], k_gain=self.P['ilc_k_gain'],
                q_span_m=self.P['ilc_q_span_m'], k_max=self.P['ilc_k_max'],
                shift_max_m=self.P['ilc_shift_max_m'],
                shift_len_m=self.P['ilc_shift_len_m'],
                clear_target=self.P['ilc_clear_target'],
                clear_band=self.P['ilc_clear_band'],
                v_down=self.P['ilc_v_down'], v_up=self.P['ilc_v_up'],
                v_lo=self.P['ilc_v_lo'], v_hi=self.P['ilc_v_hi'],
                clear_loss=self.P['ilc_clear_loss'],
                v_enable=bool(self.P['ilc_v_enable']),
                store=str(self.P['ilc_store']))
            self.ilc.set_nominal_clearance(path)
            self.mpc.ilc = self.ilc
        # Mirrors mpc3.MPC3Tracker.__init__ exactly: only `apply_ahead_s` moves
        # (one tick, because the command is held for one tick), `nd` and the
        # horizon are untouched, and `rate_adaptive` is switched off because in
        # timer mode nothing the controller does is timed by the sensor.
        self.control_mode = str(self.P['control_mode']).strip().lower()
        if self.control_mode not in ('per_pose', 'timer'):
            raise SystemExit("control_mode must be 'per_pose' or 'timer', not "
                             f"{self.P['control_mode']!r}")
        self._bridge = (float(self.P['cmd_dead_time'])
                        - 0.5 * float(self.P['rate_nominal']))
        if self.control_mode == 'timer':
            self.rate = max(float(self.P['control_hz']), 1.0)
            self.rate_ctl.enabled = False
            ahead = float(self.P['timer_apply_ahead_s']) or 1.0 / self.rate
            self.mpc.set_rate(apply_ahead_s=ahead)
            if self.P['timer_sync_dead']:
                self.act.dead_time = self._bridge

    # -- the two "callbacks" ------------------------------------------------
    def on_frame(self, t, gyro, speed, steer_fb):
        self.speed_meas = speed
        if self.P['use_steer_feedback']:
            self.act.snap(t - self.P['steer_fb_age'], steer_fb)

    def on_pose(self, t_now, t_stamp, x, y, yaw):
        self.rate_ctl.update(t_stamp)
        if self.P['v_pose_gate'] > 0.0:
            pv = self._pose_prev
            dt = (t_stamp - pv[0]) if pv is not None else 0.0
            self._v_pose = (math.hypot(x - pv[1], y - pv[2]) / dt
                            if dt > self.P['v_pose_min_dt'] else None)
            self._pose_prev = (t_stamp, x, y)
        if self.v_plan0 <= 0.0:
            self.v_plan0 = float(self.path.v_prof[
                self.path.nearest(x, y, None, yaw=yaw)])
        v = self.measured_speed() or self.start_speed()
        fx, fy, fyaw, _ = self.filt.update(t_stamp, x, y, yaw, max(v, 0.4))
        self.meas = (t_stamp, fx, fy, fyaw)

    def enc_alpha(self, nominal):
        """The encoder filter's alpha the LIVE node would be running.

        The harness models `vehicle.SpeedEstimator` outside the tracker (it is
        the bridge that reports a speed), so the rate-adapted alpha has to be
        handed back to it or the `enc` term cannot be isolated at all.
        """
        if not self.P['rate_adaptive'] or nominal <= 0.0:
            return nominal
        r = self.rate_ctl.applied / self.rate_ctl.adapter.nominal
        return 1.0 - (1.0 - nominal) ** r

    def pickup_time(self, t, ts):
        """When a command published at ``t`` is actually LATCHED.

        The bridge consumes one command per telemetry tick, and the pose stamp
        IS a telemetry tick, so the pickup grid is observable without knowing
        anything about the tracker's own schedule.  Commands issued between two
        ticks share a pickup time and the last one wins -- which is what the
        simulator does, and what `Actuator._target_at` does with equal stamps.
        """
        p = float(self.rate_ctl.est.period)
        if not (p > 1e-3):
            return t
        k = max(1, int(math.ceil((t - ts) / p - 1e-9)))
        return ts + k * p

    def measured_speed(self):
        v = self.speed_meas
        gate = self.P['v_pose_gate']
        if gate > 0.0 and self._v_pose is not None and v > self._v_pose + gate:
            self._gate_run += 1
            v = self._v_pose
        else:
            self._gate_run = 0
        if v > self.P['v_meas_min']:
            self._enc_seen = True
            return v
        return None

    def start_speed(self):
        return self.v_plan0 if self._enc_seen else self.P['v_start']

    def plan_ay(self, i0, v):
        P = self.path
        span = max(self.P['cap_preview_min_m'], v * self.mpc.N * self.mpc.dt)
        n = max(2, int(span / max(P.spacing, 1e-3)))
        idx = (i0 + np.arange(n)) % P.n
        return float((P.v_prof[idx] ** 2 * np.abs(P.kappa[idx])).max())

    # -- one control cycle ---------------------------------------------------
    def update(self, t):
        import time
        t0 = time.perf_counter()
        if self.meas is None:
            # Timer mode only: ticks can precede the first pose.  Hold, which is
            # what the latched command does in the simulator anyway.
            return self._last
        ts, x, y, yaw = self.meas
        if self.control_mode == 'timer' and t - ts > self.P['stale_pose_s']:
            # The node's `on_control_tick` skips the tick and lets the watchdog
            # hold the steering and coast; here that is the same publication.
            self._last = (self._last[0], 0.0)
            return self._last
        v_meas = self.measured_speed()
        v0 = max(v_meas if v_meas is not None else self.start_speed(), 0.4)
        g = self.mpc.cfg['yaw_gain']
        t_now = min(t, ts + self.rate_ctl.values["max_predict_s"])
        x, y, yaw, delta0 = self.act.predict(
            x, y, yaw, ts, t_now, v0,
            kappa_of=lambda d: g * math.tan(d) / vehicle.WHEELBASE)
        ra = int(self.P['reindex_after'])
        last = None if (ra > 0 and self._gate_run >= ra) else self.idx
        i0 = self.path.nearest(x, y, last, yaw=yaw)
        self.idx = i0
        pyaw = float(self.path.yaw[i0])
        dx, dy = x - self.path.xy[i0, 0], y - self.path.xy[i0, 1]
        e_lat = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
        e_yaw = wrap(yaw - pyaw)
        self.e_lat = e_lat
        dt = self.mpc.dt
        committed = [self.act.delta_at(t_now + (k + 1) * dt)
                     for k in range(self.mpc.nd)]
        v_cap = (max(v0, v_meas) if (self.P['cap_use_measured'] and v_meas)
                 else v0)
        ay_cap = max(self.P['ay_limit'],
                     self.P['cap_ff_headroom'] * self.plan_ay(i0, v0))
        cap = min(vehicle.MAX_STEER,
                  math.atan(vehicle.WHEELBASE * ay_cap
                            / max(v_cap * v_cap, 0.25)))
        cmd, info = self.mpc.solve(e_lat, e_yaw, delta0, i0, v0, cap=cap,
                                   committed=committed)
        if self.ilc is not None:
            if (self.idx_prev is not None
                    and self.idx_prev > 0.8 * self.path.n
                    and i0 < 0.2 * self.path.n):
                st = self.ilc.finish_lap()
                if st:
                    self.ilc_log.append(st)
            self.idx_prev = i0
            self.ilc.observe(float(self.path.s[i0]), e_lat,
                             corridor_clearance(self.path, i0, e_lat))
        self.v_plan0 = float(self.path.v_prof[i0])
        self.v_cmd = float(info['v_cmd'])
        th = float(self.thr.command(self.v_cmd, v_meas, 0.0, now=t,
                                    kappa=float(self.path.kappa[i0])))
        self.act.record(self.pickup_time(t, ts)
                        if (self.control_mode == 'timer'
                            and self.P['timer_sync_dead']) else t, cmd)
        self.solve_ms.append((time.perf_counter() - t0) * 1e3)
        self._last = (cmd, th)
        return cmd, th


# ---------------------------------------------------------------------------
# tracker B: a faithful port of the shipping mpc.py
# ---------------------------------------------------------------------------

class LegacyMPCSim:
    """src/pit_crew/mpc.py's control law, unchanged, at its own 50 Hz.

    Ported rather than imported because mpc.py is a ROS node and is live in a
    container mid-experiment.  Kept line-for-line equivalent: gyro dead
    reckoning to now, bicycle prediction through ``cmd_delay_s`` using the
    published command history, global nearest index, the 9-candidate grid over
    +-4 rate-limit steps, the 12x0.06 s rollout with its proportional
    relaxation tail, and the same rate limiter on the applied command.
    """

    name = 'mpc'
    rate = 50.0

    def __init__(self, path, **kw):
        self.path = path
        self.P = dict(horizon=12, step_dt=0.06, min_step_m=0.10, w_lat=12.0,
                      w_yaw=3.0, w_steer=0.5, w_rate=4.0, n_cand=9,
                      reach_steps=4.0, control_hz=50.0, steer_rate=3.2,
                      pose_blend=0.55, cmd_delay_s=0.18, ay_limit=7.8,
                      max_predict_s=0.35, brake_horizon_s=0.35,
                      throttle_gain=0.040, brake_gain=0.05)
        self.P.update(kw)
        self.control_dt = 1.0 / self.P['control_hz']
        self.thr = vehicle.ThrottleController(ff_gain=self.P['throttle_gain'],
                                              brake_gain=self.P['brake_gain'])
        self.pose = None
        self.gyro = 0.0
        self.speed = 0.0
        self.steer = 0.0
        self.hist = [(-1e9, 0.0)]
        self.e_lat = 0.0
        self.t = 0.0
        self.speed_meas = 0.0

    def on_frame(self, t, gyro, speed, steer_fb):
        self.gyro = gyro
        self.speed_meas = speed

    def on_pose(self, t_now, t_stamp, x, y, yaw):
        if self.pose is None:
            self.pose = (x, y, yaw, t_stamp)
            return
        px, py, pyaw = self._predicted(t_stamp)
        a = self.P['pose_blend']
        self.pose = (px + a * (x - px), py + a * (y - py),
                     pyaw + a * wrap(yaw - pyaw), t_stamp)

    def _predicted(self, now):
        x, y, yaw, t = self.pose
        dt = min(max(now - t, 0.0), self.P['max_predict_s'])
        ym = yaw + 0.5 * self.gyro * dt
        v = self.speed
        return (x + v * math.cos(ym) * dt, y + v * math.sin(ym) * dt,
                yaw + self.gyro * dt)

    def _through_delay(self, x, y, yaw, now):
        v = max(self.speed, 0.4)
        d = self.P['cmd_delay_s']
        n = max(1, int(round(d / 0.01)))
        dt = d / n
        j = 0
        for k in range(n):
            te = now + k * dt - d
            while j + 1 < len(self.hist) and self.hist[j + 1][0] <= te:
                j += 1
            delta = self.hist[j][1]
            yr = v * math.tan(delta) / vehicle.WHEELBASE
            ym = yaw + 0.5 * yr * dt
            x += v * math.cos(ym) * dt
            y += v * math.sin(ym) * dt
            yaw = wrap(yaw + yr * dt)
        return x, y, yaw

    def _rollout_cost(self, d0, e_lat, e_yaw, i0, v):
        P = self.path
        cost = 0.0
        delta, prev, idx = d0, self.steer, i0
        s_tr = 0.0
        ds = max(v * self.P['step_dt'], self.P['min_step_m'])
        for _ in range(int(self.P['horizon'])):
            kap = float(P.kappa[idx % P.n])
            den = max(1.0 - kap * e_lat, 0.2)
            e_lat = e_lat + math.sin(e_yaw) * ds
            kc = math.tan(delta) / vehicle.WHEELBASE
            e_yaw = wrap(e_yaw + (kc - kap * math.cos(e_yaw) / den) * ds)
            cost += (self.P['w_lat'] * e_lat ** 2 + self.P['w_yaw'] * e_yaw ** 2
                     + self.P['w_steer'] * delta ** 2
                     + self.P['w_rate'] * (delta - prev) ** 2)
            prev = delta
            ff = math.atan(vehicle.WHEELBASE * kap)
            delta = delta + 0.5 * (ff - delta)
            s_tr += ds
            idx = i0 + int(s_tr / max(P.spacing, 1e-3))
        return cost

    def update(self, t):
        if self.pose is None:
            return 0.0, 0.0
        P = self.path
        x, y, yaw = self._predicted(t)
        x, y, yaw = self._through_delay(x, y, yaw, t)
        i0 = int(np.argmin(np.hypot(P.xy[:, 0] - x, P.xy[:, 1] - y)))
        pyaw = float(P.yaw[i0])
        dx, dy = x - P.xy[i0, 0], y - P.xy[i0, 1]
        e_lat = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
        e_yaw = wrap(yaw - pyaw)
        self.e_lat = e_lat
        v = max(self.speed, 0.4)
        cap = vehicle.MAX_STEER
        if self.P['ay_limit'] > 0:
            cap = min(cap, math.atan(vehicle.WHEELBASE * self.P['ay_limit']
                                     / max(v * v, 0.25)))
        reach = self.P['steer_rate'] * self.control_dt * self.P['reach_steps']
        lo, hi = max(-cap, self.steer - reach), min(cap, self.steer + reach)
        if lo > hi:
            lo = hi = float(np.clip(self.steer, -cap, cap))
        best, bc = self.steer, float('inf')
        for cand in np.linspace(lo, hi, int(self.P['n_cand'])):
            c = self._rollout_cost(float(cand), e_lat, e_yaw, i0, v)
            if c < bc:
                bc, best = c, float(cand)
        ms = self.P['steer_rate'] * self.control_dt
        self.steer += float(np.clip(best - self.steer, -ms, ms))
        self.steer = float(np.clip(self.steer, -vehicle.MAX_STEER,
                                   vehicle.MAX_STEER))
        n_ahead = max(2, int(max(0.2, self.speed * self.P['brake_horizon_s'])
                             / max(P.spacing, 1e-3)))
        idx = (i0 + np.arange(n_ahead)) % P.n
        self.speed = float(P.v_prof[idx].min())
        v_fb = self.speed_meas if self.P['brake_gain'] > 0 else None
        th = float(self.thr.command(self.speed, v_fb, 0.0))
        self.hist.append((t, self.steer))
        while len(self.hist) > 2 and self.hist[1][0] < t - 1.0:
            self.hist.pop(0)
        return self.steer, th


# ---------------------------------------------------------------------------
# the loop
# ---------------------------------------------------------------------------

def load_checkpoints(map_prefix):
    """Ordered numbered checkpoints of a scene, as the simulator resets to them.

    `data/maps/<name>_checkpoints.csv` is written by `tools/map_from_mesh.py`
    straight out of the Unity scene, so these ARE the poses a collision reset
    uses.  Measured against seven Porto resets in data/logs (Sept 12): the car
    reappears 0.14-0.16 m short of the last numbered checkpoint it passed, at
    that checkpoint's yaw and at rest.
    """
    import csv as _csv
    rows = []
    with open(map_prefix + '_checkpoints.csv') as f:
        for r in _csv.DictReader(f):
            if r['kind'] == 'checkpoint' and r['name'].lstrip('-').isdigit():
                rows.append((int(r['name']), float(r['x']), float(r['y']),
                             float(r['yaw'])))
    rows.sort()
    return np.array([[x, y, yaw] for _n, x, y, yaw in rows], float)


def footprint_clear(dist, meta, x, y, yaw, pts=None):
    """Smallest wall distance under the car's four corners plus its sides."""
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    H, W = dist.shape
    if pts is None:
        hl, hw = 0.5 * vehicle.CAR_LENGTH, 0.5 * vehicle.CAR_WIDTH
        pts = np.array([[a, b] for a in (-hl, -hl / 3, hl / 3, hl)
                        for b in (-hw, 0.0, hw)])
    c, s = math.cos(yaw), math.sin(yaw)
    gx = x + pts[:, 0] * c - pts[:, 1] * s
    gy = y + pts[:, 0] * s + pts[:, 1] * c
    cx = np.clip(((gx - ox) / res).astype(int), 0, W - 1)
    cy = np.clip(((gy - oy) / res).astype(int), 0, H - 1)
    return float(dist[cy, cx].min())


def run(tracker, path, laps=6, dist=None, meta=None, seed=0, pose_noise=0.0,
        yaw_noise=0.0, noise_tau=0.5, pose_bias_inside=0.0, ay_max=AY_MAX,
        pose_latency=POSE_LATENCY, verbose=True, v_start=None, frame=FRAME,
        enc_alpha=0.0, servo=None, enc_slip=1.0, log_enc_alpha=0.0,
        pose_lon_bias=0.0, teleport=None,
        pose_speed_cap=0.0, pose_lag_tau=0.5, pose_lag_kappa=0.2, actuator=None):
    """Drive ``tracker`` round ``path`` until ``laps`` are complete.

    Localization error is modelled as an Ornstein-Uhlenbeck process in the
    body-lateral direction rather than white noise, because that is what a
    particle filter against a map produces: a slowly wandering bias, not
    independent jitter.  White noise would flatter whichever tracker filters
    hardest and tell us nothing about the corner geometry.
    """
    rng = np.random.default_rng(seed)
    pending = []                       # poses in flight through localization
    err_lat = err_yaw = 0.0
    frame = float(frame)
    beta = math.exp(-frame / max(noise_tau, 1e-3))
    # The wheel-encoder speed the bridge reports is `vehicle.SpeedEstimator`'s
    # output, a per-SAMPLE low pass: its lag in seconds therefore depends on
    # the sensor period, which is one of the things rate adaptivity has to
    # answer for.  0.0 = off, i.e. the perfect speed the harness used before.
    enc_v = None
    # Opt-in (default 0.0 = off): the localization motion model's own speed
    # cap (`localization.max_step_m` / `rate_nominal`, 5.769 m/s by default)
    # truncates dead reckoning above that speed.  Where the scan cannot
    # observe along-track motion -- a long straight -- the estimate then falls
    # behind at (v_true - cap) and only recovers where the corridor curves.
    pose_lag = 0.0
    # A second, purely DIAGNOSTIC encoder channel: what `src/tools/race_logger.py`
    # would have written in its `speed` column.  Every "live speed lag" number in
    # docs/STATUS.md came from that column, so without modelling it here the
    # harness and the logs are not measuring the same signal.
    log_enc = None
    i0 = 0
    # v_start=0 is a STANDING START, which is how a race actually begins and
    # is not the same test as a flying lap: the encoder has said nothing yet.
    plant = Plant(path.xy[i0, 0], path.xy[i0, 1], float(path.yaw[i0]),
                  v=(float(path.v_prof[i0]) if v_start is None else float(v_start)),
                  ay_max=ay_max, frame=frame, **(servo or {}),
                  **(actuator or {}))
    steer_cmd, throttle = 0.0, 0.0
    t = 0.0
    next_frame = 0.0
    next_ctl = 0.0
    total_s = float(path.s[-1] + path.spacing)
    last_i = i0
    lap = 0
    lap_t0 = 0.0
    lap_times = []
    collisions = 0
    coll_s = []
    cool = -1.0
    rec = []
    path_m = 0.0
    last_xy = (plant.x, plant.y)
    t_end = laps * 30.0 + 20.0
    # --- opt-in realistic checkpoint teleport (`--teleport`) ---------------
    # Default None keeps the idealized on-line reset this harness has always
    # used, so every existing number is unchanged.
    tp = teleport or {}
    tp_cp = tp.get('cp')
    tp_tau = float(tp.get('reloc_tau', 0.0))
    tp_every = float(tp.get('every', 0.0))
    tp_ex = tp_ey = 0.0            # pose error injected by the last teleport
    tp_t = -1e9                    # when it happened
    tp_next = tp_every if tp_every > 0 else 1e9
    tp_events = []                 # (t, s, x, y) of each injected teleport
    tp_rec = []                    # (t, e_lat, v, v_prof) after each teleport
    tp_coll = []                   # seconds from the last teleport to a collision

    def do_teleport(now, j):
        """Put the car at the last numbered checkpoint it passed, at rest."""
        nonlocal tp_ex, tp_ey, tp_t
        d = np.hypot(tp_cp[:, 0] - plant.x, tp_cp[:, 1] - plant.y)
        k = int(np.argmin(d))
        # "Last checkpoint passed" = the nearest one that is BEHIND the car in
        # its own heading frame; the nearest one otherwise.
        ahead = ((tp_cp[:, 0] - plant.x) * math.cos(plant.yaw)
                 + (tp_cp[:, 1] - plant.y) * math.sin(plant.yaw))
        back = np.where(ahead <= 0.0)[0]
        if len(back):
            k = int(back[np.argmin(d[back])])
        ox, oy = plant.x, plant.y
        plant.x, plant.y, plant.yaw = (float(tp_cp[k, 0]), float(tp_cp[k, 1]),
                                       float(tp_cp[k, 2]))
        plant.v = 0.0
        # The commands LATCH through a reset and the steering rack does not
        # move, so `cmd_queue`, `steer_target` and `delta` are deliberately
        # left alone -- unlike the idealized reset above, which clears them.
        # The particle filter does not jump: measured on 100 live teleports its
        # frame-to-frame step is SMALLER than in normal driving (0.23 m against
        # 0.38 m).  It walks to the checkpoint over ~1.2 s, which is modelled
        # here as the teleport displacement decaying with `reloc_tau`.
        tp_ex, tp_ey = ox - plant.x, oy - plant.y
        tp_t = now
        tp_events.append((now, float(path.s[j]), plant.x, plant.y))

    while t < t_end and lap < laps:
        if t >= next_frame - 1e-9:
            gyro = plant.v * plant.kappa
            sd = math.sqrt(max(1.0 - beta * beta, 1e-9))
            err_lat = beta * err_lat + rng.normal(0, pose_noise * sd)
            err_yaw = beta * err_yaw + rng.normal(0, yaw_noise * sd)
            bias = 0.0
            if pose_bias_inside:
                # The particle filter reads 0.12-0.17 m toward the INSIDE in
                # both hairpins and is unbiased on the straights, because the
                # LiDAR's 0.1933 m forward offset is ignored in the map and in
                # the filter alike.  Inside is the sign of the curvature.
                kr = float(path.kappa[last_i])
                bias = (math.copysign(pose_bias_inside, kr)
                        * min(1.0, abs(kr) / 0.4))
            # A LONGITUDINAL pose error is not the same failure as a lateral
            # one and the harness modelled only the lateral kind.  Measured
            # est-truth along the car's own axis at the s 11.3 hairpin: -0.21/
            # -0.13 m at a 0.104 s period and -0.03/-0.07 m at 0.088 s, i.e.
            # the estimate sits ~0.15 m FURTHER FORWARD at the faster rate.
            # An arc-length error turns the car in early where kappa ramps,
            # which is a corner cut and not a tracking error at all.
            lon_b = pose_lon_bias
            if pose_speed_cap > 0.0:
                pose_lag += max(0.0, plant.v - pose_speed_cap) * frame
                kr = abs(float(path.kappa[last_i]))
                if pose_lag_kappa > 0.0 and kr > 0.0:
                    obs = min(1.0, kr / pose_lag_kappa)
                    pose_lag *= math.exp(-frame * obs / max(pose_lag_tau, 1e-3))
                lon_b -= pose_lag
            nx = (plant.x - math.sin(plant.yaw) * (err_lat + bias)
                  + math.cos(plant.yaw) * lon_b)
            ny = (plant.y + math.cos(plant.yaw) * (err_lat + bias)
                  + math.sin(plant.yaw) * lon_b)
            if tp_tau > 0.0 and t - tp_t < 8.0 * tp_tau:
                # Localization has not caught up with the reset yet.
                g = math.exp(-(t - tp_t) / tp_tau)
                nx += tp_ex * g
                ny += tp_ey * g
            # The bridge publishes IMU, encoders and the steering angle at the
            # frame.  The POSE goes through the particle filter first, so it is
            # stamped with the scan time and delivered `pose_latency` later --
            # which is why the newest steering sample is always newer than the
            # pose the controller is about to optimise from.
            # The wheel encoder measures WHEEL speed, which over-reads the
            # ground speed under drive (measured +3.9% in the mean over the
            # 100-lap log).  `enc_slip` 1.0 keeps the old perfect sensor.
            v_wheel = enc_slip * plant.v
            if enc_alpha > 0.0:
                a = getattr(tracker, 'enc_alpha', lambda n: n)(enc_alpha)
                enc_v = (v_wheel if enc_v is None
                         else enc_v + a * (v_wheel - enc_v))
                v_rep = enc_v
            else:
                v_rep = plant.v
            if log_enc_alpha > 0.0:
                log_enc = (v_wheel if log_enc is None else log_enc
                           + log_enc_alpha * max(-LOG_ENC_CLAMP,
                                                 min(LOG_ENC_CLAMP,
                                                     v_wheel - log_enc)))
            tracker.on_frame(t, gyro, v_rep, plant.delta)
            pending.append((t + pose_latency, t, nx, ny,
                            wrap(plant.yaw + err_yaw)))
            plant.accept_frame(t, steer_cmd, throttle)
            next_frame += frame
        while pending and pending[0][0] <= t:
            _, ts, nx, ny, nyaw = pending.pop(0)
            tracker.on_pose(t, ts, nx, ny, nyaw)
            if tracker.rate is None:
                steer_cmd, throttle = tracker.update(t)
        if tracker.rate is not None and t >= next_ctl - 1e-9:
            steer_cmd, throttle = tracker.update(t)
            next_ctl += 1.0 / tracker.rate
        plant.step(t)
        t += DT

        # bookkeeping at the sensor rate is enough for the metrics
        if abs(t / DT - round(t / DT)) < 1e-6 and round(t * 1000) % 10 == 0:
            j = path.nearest(plant.x, plant.y, last_i, yaw=plant.yaw)
            if j < last_i - path.n // 2:
                j += 0                      # wrapped; handled below
            if last_i > path.n * 0.8 and j < path.n * 0.2:
                lap += 1
                lap_times.append(t - lap_t0)
                lap_t0 = t
            last_i = j
            pyaw = float(path.yaw[j])
            e = (-math.sin(pyaw) * (plant.x - path.xy[j, 0])
                 + math.cos(pyaw) * (plant.y - path.xy[j, 1]))
            clear = (footprint_clear(dist, meta, plant.x, plant.y, plant.yaw)
                     if dist is not None else 1.0)
            if tp_cp is not None and tp_every > 0.0 and t >= tp_next:
                tp_next = t + tp_every
                do_teleport(t, j)
                cool = t + 1.0
            if clear <= 0.0 and t > cool:
                collisions += 1
                coll_s.append(float(path.s[j]))
                if tp_events:
                    tp_coll.append(t - tp_t)     # BEFORE this collision's own
                cool = t + 1.0
                # The simulator returns the car to its last checkpoint after a
                # collision.  Without that, one contact leaves the car grinding
                # along a wall for the rest of the run and every number after
                # it is meaningless -- an early version reported "1 collision"
                # for a run that completed no laps at all.
                if tp_cp is not None:
                    do_teleport(t, j)
                else:
                    plant.x, plant.y = path.xy[j, 0], path.xy[j, 1]
                    plant.yaw = float(path.yaw[j])
                    plant.v = 0.5 * float(path.v_prof[j])
                    plant.delta = 0.0
                    plant.cmd_queue.clear()
                    plant.steer_target = 0.0
            if tp_events and t - tp_t < 6.0:
                # Global nearest: after a backwards teleport the windowed
                # search `j` above can still be pointing ahead of the car,
                # which is one of the things being measured -- it must not
                # also corrupt the measurement.
                jg = path.nearest(plant.x, plant.y, None, yaw=plant.yaw)
                gy = float(path.yaw[jg])
                eg = (-math.sin(gy) * (plant.x - path.xy[jg, 0])
                      + math.cos(gy) * (plant.y - path.xy[jg, 1]))
                tp_rec.append((t - tp_t, eg, plant.v, float(path.v_prof[jg]),
                               float(clear), len(tp_events)))
            path_m += math.hypot(plant.x - last_xy[0], plant.y - last_xy[1])
            last_xy = (plant.x, plant.y)
            rec.append((t, plant.x, plant.y, plant.yaw, plant.v, plant.delta,
                        float(path.s[j]), e, clear, lap,
                        plant.throttle, throttle,
                        enc_v if enc_v is not None else plant.v,
                        log_enc if log_enc is not None else plant.v))
    return dict(rec=np.array(rec), laps=lap, lap_times=lap_times,
                collisions=collisions, coll_s=coll_s, path_m=path_m,
                line_m=total_s, tp_events=tp_events, tp_coll=tp_coll,
                tp_rec=np.array(tp_rec) if tp_rec else np.zeros((0, 6)))


def teleport_report(out, frame):
    """Recovery from each injected teleport, in the metrics the logs are read
    with: time to |e_lat| < 0.15 m, time to 90% of the profile speed, and how
    often a collision follows within 3 s."""
    ev, r = out['tp_events'], out['tp_rec']
    if not len(ev):
        print('  teleports: none injected')
        return
    t_e, t_v, casc, gaps = [], [], 0, []
    coll = np.array(out['coll_s'])
    for k in range(1, len(ev) + 1):
        m = r[r[:, 5] == k]
        if not len(m):
            continue
        ok = np.abs(m[:, 1]) < 0.15
        t_e.append(float(m[ok, 0].min()) if ok.any() else float('nan'))
        ok = m[:, 2] >= 0.9 * m[:, 3]
        t_v.append(float(m[ok, 0].min()) if ok.any() else float('nan'))
        # a collision inside the 3 s window shows up as clearance <= 0

    gaps = [g for g in out['tp_coll'] if g >= 0.0]
    casc = sum(1 for g in gaps if g <= 3.0)
    te = np.array(t_e, float); tv = np.array(t_v, float)
    fin = np.isfinite(te)
    print(f'  teleports {len(ev)}  '
          f't(|e_lat|<0.15 m) p50 {np.nanmedian(te):.2f} s '
          f'p90 {np.nanpercentile(te, 90):.2f} s '
          f'(never: {int((~fin).sum())})')
    print(f'    t(v >= 0.9 v_prof) p50 {np.nanmedian(tv):.2f} s '
          f'p90 {np.nanpercentile(tv, 90):.2f} s;  '
          f'cascade (collision within 3 s) {casc}/{len(ev)} '
          f'({100.0 * casc / len(ev):.0f}%)')
    if gaps:
        g = np.array(gaps)
        print(f'    collisions after a teleport: {len(g)}; gap to the '
              f'previous teleport p50 {np.median(g):.1f} s '
              f'min {g.min():.1f} s; <=3 s {int((g <= 3).sum())}, '
              f'<=6 s {int((g <= 6).sum())}')


def report(name, out, path, lo=10.0, hi=14.0, warm=1):
    r = out['rec']
    if not len(r):
        print(f'{name}: no data')
        return
    if (r[:, 9] >= warm).sum() > 50:
        r = r[r[:, 9] >= warm]                  # drop the out-lap
    s, e = r[:, 6], r[:, 7]
    print(f'\n=== {name} ===')
    lt = out['lap_times'][warm:] if len(out['lap_times']) > warm else out['lap_times']
    if lt:
        print(f'  laps {out["laps"]}  lap time median {np.median(lt):.3f} s '
              f'best {min(lt):.3f} s')
    print(f'  collisions {out["collisions"]}'
          + (f'  at s {[round(x,1) for x in out["coll_s"]]}' if out['coll_s'] else ''))
    print(f'  lap-wide |e_lat|  p50 {np.percentile(np.abs(e),50):.3f} m  '
          f'p95 {np.percentile(np.abs(e),95):.3f} m  max {np.abs(e).max():.3f} m')
    print(f'  min footprint clearance {r[:,8].min():.3f} m')
    if out['laps']:
        kap = path.kappa[np.searchsorted(path.s, s).clip(0, path.n - 1)]
        c = np.abs(kap) > 0.3
        ins = (e[c] * np.sign(kap[c])).mean() if c.any() else float('nan')
        print(f'  path {out["path_m"] / max(out["laps"], 1):.3f} m/lap against a '
              f'{out["line_m"]:.3f} m line; in corners the car is '
              f'{ins:+.3f} m INSIDE the line')
    print(f'  hairpin signed offset (+ = left of travel, - = inside the RH bend):')
    for a in np.arange(lo, hi, 1.0):
        m = (s >= a) & (s < a + 1)
        if m.sum():
            print(f'    s {a:4.1f}-{a+1:4.1f}  mean {e[m].mean():+.3f}  '
                  f'p10 {np.percentile(e[m],10):+.3f}  p90 {np.percentile(e[m],90):+.3f}')
    m = (s >= lo) & (s < hi)
    if m.sum():
        print(f'    s {lo:.0f}-{hi:.0f} overall mean {e[m].mean():+.3f}  '
              f'most-inside {e[m].min():+.3f}')


def speed_diag(out, path, grid=0.10, warm=1, servo_gain=SERVO_SPEED_PER_THROTTLE):
    """The speed loop's spatial lag, decomposed, in the SAME metric as the logs.

    Every number is a median-per-0.10 m-bin curve round the lap, and a "lag" is
    the shift ``d`` (metres of arc) that minimises the RMS between two such
    curves.  This is exactly what the log analysis does, which is the point: the
    harness and `data/logs/*.csv` have to be measured with one ruler.

    Four channels, and conflating them is what produced the "missing 0.9 m":

    * ``v_prof``   the offline profile;
    * ``setpoint`` 24.6 * the throttle in force, i.e. the controller's ``v_cmd``
      exactly -- the boundary between a PLAN lag and a SERVO lag;
    * ``v_true``   the plant speed, which is what the IPS-derived speed in a log
      measures;
    * ``log_enc``  what `race_logger.py` would have written (needs
      ``--log-enc-alpha``), which is where "1.45 m" came from.
    """
    r = out['rec']
    if not len(r):
        print('  speed-diag: no data')
        return None
    if r.shape[1] < 14:
        print('  speed-diag: run() did not record the throttle channels')
        return None
    r = r[r[:, 9] >= warm]
    if len(r) < 50:
        print('  speed-diag: not enough post-warmup data')
        return None
    L = float(path.s[-1] + path.spacing)
    g = np.arange(0.0, L, grid)

    def curve(val):
        idx = np.clip((r[:, 6] / grid).astype(int), 0, len(g) - 1)
        out_ = np.full(len(g), np.nan)
        for k in np.unique(idx):
            out_[k] = np.median(val[idx == k])
        ok = ~np.isnan(out_)
        return np.interp(g, g[ok], out_[ok])

    vp = np.interp(g, path.s, path.v_prof, period=L)
    ch = dict(setpoint=curve(servo_gain * r[:, 10]), v_true=curve(r[:, 4]),
              v_enc=curve(r[:, 12]), log_enc=curve(r[:, 13]))

    def lag(a, b, lo=-1.5, hi=4.0):
        ds = np.arange(lo, hi, 0.01)
        e = [np.sqrt(np.mean((a - np.interp((g - d) % L, g, b, period=L)) ** 2))
             for d in ds]
        k = int(np.argmin(e))
        return float(ds[k]), float(e[k]), float(e[int(np.argmin(np.abs(ds)))])

    print('  speed-loop lag (positive = the first signal lags the second):')
    res = {}
    for nm, a, b in (('v_true   vs v_prof', ch['v_true'], vp),
                     ('setpoint vs v_prof', ch['setpoint'], vp),
                     ('v_true   vs setpoint', ch['v_true'], ch['setpoint']),
                     ('v_enc    vs v_prof', ch['v_enc'], vp),
                     ('log_enc  vs v_prof', ch['log_enc'], vp)):
        d, e1, e0 = lag(a, b)
        res[nm.split(' vs ')[0].strip() + '|' + nm.split(' vs ')[1].strip()] = d
        print('    %-21s %+6.2f m   rms %.3f -> %.3f m/s' % (nm, d, e0, e1))
    print('    peaks  v_prof %.2f  setpoint %.2f  v_true %.2f  v_enc %.2f  '
          'log_enc %.2f' % (vp.max(), ch['setpoint'].max(), ch['v_true'].max(),
                            ch['v_enc'].max(), ch['log_enc'].max()))
    print('    s-bin   v_prof  setpt  v_true   sp-prof  true-sp  true-prof')
    for lo in range(0, int(L), 2):
        k = (g >= lo) & (g < lo + 2)
        print('     %5.1f  %6.2f %6.2f %6.2f   %+6.2f  %+6.2f  %+6.2f'
              % (lo, vp[k].mean(), ch['setpoint'][k].mean(), ch['v_true'][k].mean(),
                 (ch['setpoint'][k] - vp[k]).mean(),
                 (ch['v_true'][k] - ch['setpoint'][k]).mean(),
                 (ch['v_true'][k] - vp[k]).mean()))
    res['peak_true'] = float(ch['v_true'].max())
    res['peak_log_enc'] = float(ch['log_enc'].max())
    return res


def per_lap(out):
    """Lap time, |e_lat| p95 and worst clearance, lap by lap.

    The ILC's whole claim is that lap N+1 is better than lap N, which a
    run-wide percentile cannot show.
    """
    r = out['rec']
    print('   lap   time     |e| p50   |e| p95    |e| max   min clear')
    for i, lt in enumerate(out['lap_times']):
        m = r[:, 9] == i
        if not m.any():
            continue
        e = np.abs(r[m, 7])
        print(f'   {i:3d}  {lt:6.3f}    {np.percentile(e,50):7.3f}  '
              f'{np.percentile(e,95):7.3f}   {e.max():7.3f}   {r[m,8].min():7.3f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--line', default='data/racelines/track_v2_safe_ay6.0.csv')
    ap.add_argument('--map', default='data/maps/track_v2')
    ap.add_argument('--laps', type=int, default=6)
    ap.add_argument('--tracker', default='both',
                    choices=('both', 'all', 'mpc', 'mpc2', 'mpc3'))
    ap.add_argument('--speed-scale', type=float, default=1.0)
    ap.add_argument('--hairpin', default='10,14')
    ap.add_argument('--no-barrier', action='store_true')
    ap.add_argument('--kappa-span', type=float, default=0.5)
    ap.add_argument('--ay-max', type=float, default=AY_MAX,
                    help='sustained lateral grip of the plant, m/s2')
    ap.add_argument('--frame', type=float, default=FRAME,
                    help='sensor / command-pickup period, s (0.088-0.104 measured)')
    ap.add_argument('--enc-alpha', type=float, default=0.0,
                    help="model vehicle.SpeedEstimator's per-sample low pass")
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--servo-cal', action='store_true',
                    help='plant speed loop calibrated to the live logs')
    ap.add_argument('--plant-realistic', action='store_true',
                    help='plant speed loop IDENTIFIED from the clean 100-lap '
                         'log (K 18, +5.5/-4.0, drag 0.273); implies the '
                         'encoder slip and both encoder channels')
    ap.add_argument('--enc-slip', type=float, default=1.0,
                    help='wheel/ground speed ratio the encoder reports '
                         '(measured 1.039)')
    ap.add_argument('--log-enc-alpha', type=float, default=0.0,
                    help="model race_logger.py's own speed filter (0.30) so "
                         'harness numbers compare with a log column')
    ap.add_argument('--pose-lon-bias', type=float, default=0.0,
                    help='localization bias ALONG the car axis, m (+ = the '
                         'estimate reads further forward than the car is)')
    ap.add_argument('--speed-diag', action='store_true',
                    help='decompose the speed-loop spatial lag')
    ap.add_argument('--per-lap', action='store_true')
    ap.add_argument('--pose-latency', type=float, default=POSE_LATENCY,
                    help='scan stamp to pose delivery, s')
    ap.add_argument('--pose-bias-inside', type=float, default=0.0,
                    help='localization bias toward the inside of a bend, m')
    ap.add_argument('--v-start', type=float, default=None,
                    help='initial speed; 0 = standing start')
    ap.add_argument('--pose-speed-cap', type=float, default=0.0,
                    help='OPT-IN, 0=off: localization motion-model speed cap, '
                         'm/s. The injected estimate falls behind by '
                         'int max(0, v-cap) dt and recovers only where the '
                         'corridor curves (see --pose-lag-tau/--pose-lag-kappa).')
    ap.add_argument('--pose-lag-tau', type=float, default=0.5,
                    help='recovery time constant of that lag in a corner, s')
    ap.add_argument('--pose-lag-kappa', type=float, default=0.2,
                    help='curvature 1/m at which the lag recovers at full rate')
    ap.add_argument('--pose-noise', type=float, default=0.0)
    ap.add_argument('--yaw-noise', type=float, default=0.0)
    ap.add_argument('--noise-tau', type=float, default=0.5,
                    help='correlation time of the localization error, s')
    ap.add_argument('-p', '--param', action='append', default=[],
                    help='mpc2 override, key=value')
    ap.add_argument('-q', '--legacy-param', action='append', default=[],
                    help='mpc (legacy) override, key=value')
    ap.add_argument('-r', '--mpc3-param', action='append', default=[],
                    help='mpc3 override, key=value')
    ap.add_argument('--teleport', action='store_true',
                    help='opt-in: after a collision put the car at the last '
                         'numbered CHECKPOINT of --map at rest, with the '
                         'localization estimate lagging behind it, instead of '
                         "the harness's idealized on-line reset")
    ap.add_argument('--teleport-every', type=float, default=0.0,
                    help='also inject a teleport every N s of driving, so '
                         'recovery can be measured without waiting for a '
                         'collision (0 = only on collisions).  Lap COUNTS and '
                         'lap TIMES are meaningless under this: a teleport '
                         'across the start line re-crosses it.  Read the '
                         'collision count and the teleport report instead')
    ap.add_argument('--teleport-reloc-tau', type=float, default=0.84,
                    help='time constant of the localization estimate walking '
                         'to the checkpoint. 0.84 s is fitted to 100 live '
                         'teleports: |est-truth| 0.63 m at the reset, under '
                         '0.15 m after 1.20 s (median)')
    ap.add_argument('--plant-actuator', default='slew',
                    choices=('slew', 'pole', 'tick'),
                    help='steering-actuator law in the PLANT (default slew, '
                         'the 3.2 rad/s wall-clock limiter this harness has '
                         'always used).  "pole" is the first-order actuator '
                         'the staircase log identifies; "tick" is the '
                         'rate-DEPENDENT hypothesis, one allowance per '
                         'telemetry tick.')
    ap.add_argument('--plant-steer-rate', type=float, default=None,
                    help='plant slew, rad/s (default vehicle.STEER_RATE 3.2); '
                         'with --plant-actuator tick it sets the per-tick '
                         'allowance, steer_rate * frame rad')
    ap.add_argument('--plant-tau', type=float, default=0.08,
                    help='first-order time constant, s, for '
                         '--plant-actuator pole (identified 0.06-0.08)')
    ap.add_argument('--plant-dead', type=float, default=None,
                    help='bridge -> actuator dead time, s (default '
                         'EXTRA_DEAD 0.090; the staircase identifies '
                         '0.12-0.16 end to end)')
    ap.add_argument('--plant-tick-frac', type=float, default=1.0,
                    help='fraction of the remaining error the actuator takes '
                         'per telemetry tick with --plant-actuator tick')
    ap.add_argument('--plot', default='')
    args = ap.parse_args()
    lo, hi = (float(x) for x in args.hairpin.split(','))

    dist = meta = None
    if args.map:
        from pit_crew import pf_core
        dist, _img, meta = pf_core.load_map(args.map)

    def parse(items):
        out = {}
        for it in items:
            k, v = it.split('=')
            out[k] = (float(v) if v.replace('.', '').replace('-', '').isdigit()
                      else v)
        return out
    kw, lkw = parse(args.param), parse(args.legacy_param)
    kw3 = parse(args.mpc3_param)

    # `--plant-realistic` is one switch for the whole measured sensor+plant
    # chain, because the parts are not independent: an identified plant with a
    # perfect speed sensor is not the live car either.  Explicit flags still win.
    enc_alpha = args.enc_alpha
    enc_slip = args.enc_slip
    log_enc_alpha = args.log_enc_alpha
    if args.plant_realistic:
        if enc_alpha <= 0.0:
            enc_alpha = 0.35            # vehicle.SpeedEstimator, what mpc3 sees
        if enc_slip == 1.0:
            enc_slip = ENC_SLIP
        if log_enc_alpha <= 0.0:
            log_enc_alpha = LOG_ENC_ALPHA

    tp_cfg = None
    if args.teleport or args.teleport_every > 0.0:
        if not args.map:
            raise SystemExit('--teleport needs --map (for the checkpoints)')
        tp_cfg = dict(cp=load_checkpoints(args.map),
                      reloc_tau=args.teleport_reloc_tau,
                      every=args.teleport_every)

    # Opt-in actuator law.  Default 'slew' with no overrides reproduces the
    # historical plant exactly, so every number already in docs/STATUS.md
    # stays reproducible without the flag.
    act_cfg = None
    if (args.plant_actuator != 'slew' or args.plant_steer_rate is not None
            or args.plant_dead is not None):
        act_cfg = dict(actuator=args.plant_actuator,
                       steer_rate=args.plant_steer_rate,
                       tau=args.plant_tau, dead=args.plant_dead,
                       tick_frac=args.plant_tick_frac)

    results = {}
    order = {'both': ('mpc', 'mpc2'), 'all': ('mpc', 'mpc2', 'mpc3')}.get(
        args.tracker, (args.tracker,))
    for which in order:
        path = RefPath(args.line, speed_scale=args.speed_scale,
                       kappa_span_m=args.kappa_span)
        if dist is not None and not args.no_barrier:
            path.load_corridor(dist, meta)
        tr = (MPC2Sim(path, **kw) if which == 'mpc2' else
              MPC3Sim(path, **kw3) if which == 'mpc3' else
              LegacyMPCSim(path, **lkw))
        out = run(tr, path, laps=args.laps, dist=dist, meta=meta,
                  pose_noise=args.pose_noise, yaw_noise=args.yaw_noise,
                  noise_tau=args.noise_tau, ay_max=args.ay_max,
                  pose_bias_inside=args.pose_bias_inside,
                  pose_latency=args.pose_latency, v_start=args.v_start,
                  frame=args.frame, enc_alpha=enc_alpha, seed=args.seed,
                  enc_slip=enc_slip, log_enc_alpha=log_enc_alpha,
                  pose_lon_bias=args.pose_lon_bias,
                  pose_speed_cap=args.pose_speed_cap,
                  pose_lag_tau=args.pose_lag_tau,
                  pose_lag_kappa=args.pose_lag_kappa, teleport=tp_cfg,
                  servo=(PLANT_REAL if args.plant_realistic else
                         SERVO_CAL if args.servo_cal else None),
                  actuator=act_cfg)
        report(which, out, path, lo, hi)
        if tp_cfg:
            teleport_report(out, args.frame)
        if args.per_lap:
            per_lap(out)
        if args.speed_diag:
            speed_diag(out, path)
        if which in ('mpc2', 'mpc3') and getattr(tr, 'solve_ms', None):
            sm = np.array(tr.solve_ms)
            print(f'  solve time  median {np.median(sm):.2f} ms  '
                  f'p95 {np.percentile(sm,95):.2f} ms  max {sm.max():.2f} ms')
        results[which] = (out, path)

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 9))
        p0 = next(iter(results.values()))[1]
        ax.plot(p0.xy[:, 0], p0.xy[:, 1], 'k--', lw=1, label='reference')
        for nm, (out, _p) in results.items():
            r = out['rec'][out['rec'][:, 9] >= 1]
            ax.plot(r[:, 1], r[:, 2], lw=1, label=nm)
        ax.axis('equal')
        ax.legend()
        fig.savefig(args.plot, dpi=110)
        print(f'wrote {args.plot}')


if __name__ == '__main__':
    main()
