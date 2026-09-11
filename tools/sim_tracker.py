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

    def __init__(self, x, y, yaw, v=0.0, ay_max=AY_MAX):
        self.x, self.y, self.yaw, self.v = x, y, yaw, v
        self.ay_max = float(ay_max)
        self.delta = 0.0                 # actual steering angle, rad
        self.cmd_queue = []              # (t_effective, steer_cmd)
        self.steer_target = 0.0
        self.throttle = 0.0
        self.kappa = 0.0

    def accept_frame(self, t, steer_cmd, throttle):
        """The bridge picks the latched command up; it bites EXTRA_DEAD later."""
        self.cmd_queue.append((t + EXTRA_DEAD, float(steer_cmd), float(throttle)))

    def step(self, t):
        while self.cmd_queue and self.cmd_queue[0][0] <= t:
            _, s, th = self.cmd_queue.pop(0)
            self.steer_target, self.throttle = s, th
        step = vehicle.STEER_RATE * DT
        self.delta += max(-step, min(step, self.steer_target - self.delta))
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
        a = max(-SERVO_A, min(SERVO_A, SERVO_K * (v_tgt - self.v)))
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
        pose_latency=POSE_LATENCY, verbose=True):
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
    beta = math.exp(-FRAME / max(noise_tau, 1e-3))
    i0 = 0
    plant = Plant(path.xy[i0, 0], path.xy[i0, 1], float(path.yaw[i0]),
                  v=float(path.v_prof[i0]), ay_max=ay_max)
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
            nx = plant.x - math.sin(plant.yaw) * (err_lat + bias)
            ny = plant.y + math.cos(plant.yaw) * (err_lat + bias)
            # The bridge publishes IMU, encoders and the steering angle at the
            # frame.  The POSE goes through the particle filter first, so it is
            # stamped with the scan time and delivered `pose_latency` later --
            # which is why the newest steering sample is always newer than the
            # pose the controller is about to optimise from.
            tracker.on_frame(t, gyro, plant.v, plant.delta)
            pending.append((t + pose_latency, t, nx, ny,
                            wrap(plant.yaw + err_yaw)))
            plant.accept_frame(t, steer_cmd, throttle)
            next_frame += FRAME
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
            if clear <= 0.0 and t > cool:
                collisions += 1
                coll_s.append(float(path.s[j]))
                cool = t + 1.0
                # The simulator returns the car to its last checkpoint after a
                # collision.  Without that, one contact leaves the car grinding
                # along a wall for the rest of the run and every number after
                # it is meaningless -- an early version reported "1 collision"
                # for a run that completed no laps at all.
                plant.x, plant.y = path.xy[j, 0], path.xy[j, 1]
                plant.yaw = float(path.yaw[j])
                plant.v = 0.5 * float(path.v_prof[j])
                plant.delta = 0.0
                plant.cmd_queue.clear()
                plant.steer_target = 0.0
            path_m += math.hypot(plant.x - last_xy[0], plant.y - last_xy[1])
            last_xy = (plant.x, plant.y)
            rec.append((t, plant.x, plant.y, plant.yaw, plant.v, plant.delta,
                        float(path.s[j]), e, clear, lap))
    return dict(rec=np.array(rec), laps=lap, lap_times=lap_times,
                collisions=collisions, coll_s=coll_s, path_m=path_m,
                line_m=total_s)


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--line', default='data/racelines/track_v2_safe_ay6.0.csv')
    ap.add_argument('--map', default='data/maps/track_v2')
    ap.add_argument('--laps', type=int, default=6)
    ap.add_argument('--tracker', default='both', choices=('both', 'mpc', 'mpc2'))
    ap.add_argument('--speed-scale', type=float, default=1.0)
    ap.add_argument('--hairpin', default='10,14')
    ap.add_argument('--no-barrier', action='store_true')
    ap.add_argument('--kappa-span', type=float, default=0.5)
    ap.add_argument('--ay-max', type=float, default=AY_MAX,
                    help='sustained lateral grip of the plant, m/s2')
    ap.add_argument('--pose-latency', type=float, default=POSE_LATENCY,
                    help='scan stamp to pose delivery, s')
    ap.add_argument('--pose-bias-inside', type=float, default=0.0,
                    help='localization bias toward the inside of a bend, m')
    ap.add_argument('--pose-noise', type=float, default=0.0)
    ap.add_argument('--yaw-noise', type=float, default=0.0)
    ap.add_argument('--noise-tau', type=float, default=0.5,
                    help='correlation time of the localization error, s')
    ap.add_argument('-p', '--param', action='append', default=[],
                    help='mpc2 override, key=value')
    ap.add_argument('-q', '--legacy-param', action='append', default=[],
                    help='mpc (legacy) override, key=value')
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

    results = {}
    for which in (('mpc', 'mpc2') if args.tracker == 'both' else (args.tracker,)):
        path = RefPath(args.line, speed_scale=args.speed_scale,
                       kappa_span_m=args.kappa_span)
        if dist is not None and not args.no_barrier:
            path.load_corridor(dist, meta)
        tr = (MPC2Sim(path, **kw) if which == 'mpc2'
              else LegacyMPCSim(path, **lkw))
        out = run(tr, path, laps=args.laps, dist=dist, meta=meta,
                  pose_noise=args.pose_noise, yaw_noise=args.yaw_noise,
                  noise_tau=args.noise_tau, ay_max=args.ay_max,
                  pose_bias_inside=args.pose_bias_inside,
                  pose_latency=args.pose_latency)
        report(which, out, path, lo, hi)
        if which == 'mpc2' and tr.solve_ms:
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
