"""mpc3 — speed-aware horizon tracker.  The tracker is allowed to slow down.

Launch exactly like the shipping trackers:

    CONTROLLER=mpc3 MAP=porto_mesh LIDAR_OFFSET=0.1933 \
        ./scripts/run_exp.sh <name> <line> [-p key:=value ...]

Same parameters the runners already pass (``raceline_csv``, ``speed_scale``),
same published topics (``steering_command``, ``throttle_command``,
``/pit_crew/cte``), plus ``map_prefix`` for the corridor barrier, derived from
the raceline name when it is not given.

What is different from ``mpc`` and ``mpc2``, and why it should matter
--------------------------------------------------------------------
Every collision in ``docs/STATUS.md`` above ay 6 is at a hairpin EXIT, and in
every one of them the car was at or under its planned speed and still could not
hold the line.  Both existing trackers have exactly one actuator for that
situation -- more steering lock -- because the speed they ask for comes from an
offline profile indexed by position and nothing else.  When the lock they add is
not delivered (the tyre saturates, or the actuator is still slewing through its
dead time), there is no second move.

mpc3's horizon carries speed, and the throttle target is the horizon's, not the
profile's.  Running wide means the plan needs more curvature to rejoin, more
curvature at a given grip budget means less speed, and the servo is asked for
that lower speed immediately -- one frame, not one corner, later.  The steering
model is the measured one (yaw gain 0.956, delivered curvature saturating at
``ay_deliver / v^2``), the dead time is structure (the first steps of the
horizon are committed to the commands already in flight), and there is no
corner-cut term: the geometry is the raceline's job.

Throttle is ``throttle2.ServoThrottle``, which inverts the simulator's speed
servo and is never negative.  ``vehicle.ThrottleController``'s ``brake_gain``
route -- negative throttle, i.e. REVERSE DRIVE -- does not exist here at all.
"""

import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32

from pit_crew import vehicle
from pit_crew.mpc3_core import (ILC, MPC3, RATE_NOMINAL, RATE_TERMS, Actuator,
                                GapRider, PoseFilter, RateControl, RefPath,
                                corridor_clearance, gap_feedforward, wrap)
from pit_crew.residual_policy import observation as residual_observation
from pit_crew.throttle2 import ServoThrottle


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def derive_map_prefix(raceline_csv, maps_dir=None):
    """Guess the map that goes with a raceline path (as mpc2 does).

    Strip ``_token`` suffixes off the raceline stem until a map of that name
    exists: ``porto_mesh_safe_ay6.0`` -> ``porto_mesh``.  Returns '' when
    nothing matches, which disables the barrier rather than guessing wrong.
    """
    if maps_dir is None:
        maps_dir = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(raceline_csv))), 'maps')
    stem = os.path.splitext(os.path.basename(raceline_csv))[0]
    parts = stem.split('_')
    for n in range(len(parts), 0, -1):
        cand = os.path.join(maps_dir, '_'.join(parts[:n]))
        if os.path.exists(cand + '.yaml') and os.path.exists(cand + '.pgm'):
            return cand
    return ''


class MPC3Tracker(Node):

    def __init__(self):
        super().__init__('mpc3')
        p = self.declare_parameters('', [
            ('raceline_csv', '/home/data/racelines/track_v2_safe.csv'),
            ('speed_scale', 1.0),
            ('map_prefix', ''),           # '' = derive from the raceline name
            ('use_barrier', True),
            ('require_barrier', True),    # refuse to race without a corridor
            # --- solver -----------------------------------------------------
            ('horizon', 20),
            ('step_dt', 0.060),
            ('min_step_m', 0.10),
            ('iters', 6),
            ('outer', 2),
            ('w_lat', 24.0),
            ('w_yaw', 3.0),
            ('w_steer', 0.30),
            ('w_rate', 0.60),
            ('w_jerk', 0.05),
            ('w_lat_T', 40.0),
            ('w_yaw_T', 8.0),
            ('w_barrier', 400.0),
            ('barrier_margin', 0.24),
            # --- measured plant ---------------------------------------------
            ('yaw_gain', 0.956),
            ('ay_deliver', 9.5),
            ('sat_knee', 0.80),
            # --- speed plan --------------------------------------------------
            ('ay_budget', 7.5),
            ('a_brake', 4.0),
            # NOTE (Sep 19 audit): `mpc3_core.DEFAULTS['a_accel']` is **5.0**
            # -- "the measured servo authority", the same number as
            # `servo_a_accel` below -- and the two were written 4.0/5.0 in the
            # SAME commit (6c55a13) with no explanation. This declaration wins
            # for the node, so the live stack has always planned its forward
            # acceleration pass at 4.0; `tools/sim_tracker.py` never sets
            # `a_accel`, so the OFFLINE HARNESS has always run 5.0. Every
            # harness number in docs/FINALS_TUNING.md therefore models a
            # different speed plan than the car.
            # Measured (harness, finals_mesh_hyb675b_race.csv, shipping args,
            # 6 seeds x 12 laps at each frame): 5.0 -> 72/0 at both frames,
            # min clearance 0.125 / 0.025; 4.0 -> 72/0 at 0.098 but 72/**1** at
            # 0.058, clearance 0.103 / 0.000. So it is NOT inert and 4.0 is the
            # worse of the two. Aligning the node to 5.0 is a speed-plan
            # change and owes Gates C and D; until then this stays 4.0 so that
            # nothing moves without being driven.
            ('a_accel', 4.0),
            ('e_free', 0.18),
            ('k_e', 2.0),
            ('v_mod_min', 0.55),
            ('e_cut_accel_scale', 1.0),
            ('e_cut_accel_kappa', 0.0),
            ('thr_lead_s', 0.225),
            ('thr_lead_mode', 'point'),   # 'min' = brake-only lead, opt-in
            # OPT-IN, defaults reproduce the shipping tune exactly:
            # `thr_source` 'ceiling' (raced) | 'profile' | 'min' -- which speed
            # curve the throttle setpoint is read from, and `thr_brake_lead_s`
            # an extra lead taken ONLY where the profile is braking.
            ('thr_source', 'ceiling'),
            ('thr_brake_lead_s', 0.0),
            ('thr_enc_tau', 0.19),
            # --- actuator / delay --------------------------------------------
            ('steer_rate_limit', 3.2),
            ('cmd_dead_time', 0.135),     # measured, structural (devkit bridge)
            ('delay_s', 0.180),           # dead time + estimation lag
            ('apply_ahead_s', 0.090),
            # Floor, in horizon steps, under which `apply_ahead_s` is not
            # believed (see `mpc3_core.DEFAULTS`).  1.0 reproduces the old
            # `max(step_dt, apply_ahead_s)` exactly, so the shipping tune is
            # untouched; it only binds below a 0.069 s sensor period, where it
            # was silently discarding the whole `apply` rate term AND pinning
            # the publish index to an integer plan knot.  Set 0.0 together with
            # `rate_adaptive`.
            ('apply_floor_steps', 1.0),
            # OPT-IN, -1.0 = off: read the published angle at exactly
            # `nd + apply_lead_steps` horizon steps, decoupling the readout
            # lead from the committed-step count `nd`.  See
            # `mpc3_core.DEFAULTS` for why (hairpin-entry turn-in).
            ('apply_lead_steps', -1.0),
            # OPT-IN, and only meaningful together (see mpc3_core.DEFAULTS):
            # `rate_terms=...,delay_tick` puts `period + dead_tick_c` on the
            # actuator's dead time, and `nd_mode:=period` derives the committed
            # steps from the same number, so `Actuator.predict` and the
            # rollout's `nd` describe one delay instead of two.
            ('dead_tick_c', 0.045),
            ('nd_mode', 'fixed'),
            # OPT-IN, 0.0 = off: pin the delay `nd` is derived from, so the
            # actuator's dead-time law and the solver's committed-step count
            # can be set independently.  See mpc3_core.DEFAULTS.
            ('nd_delay_s', 0.0),
            # --- command smoothing (OPT-IN; 0.0 = the tune that raced) -------
            # The front wheels flutter on straights because the solver's FIRST
            # free step carries no jerk cost and only a small rate cost, so the
            # cheapest answer to a small lateral error is a one-step flick at
            # the slew limit -- which the car never executes, because the
            # command is held for a whole sensor period.  Measured in
            # `tools/sim_tracker.py` with the pose noise switched OFF, so it is
            # a dead-time relay limit cycle, not noise chasing.  See
            # `mpc3_core.DEFAULTS` for what each knob does; validated together
            # at `w_dcmd` 50 + `cmd_tau` 0.10.
            ('w_dcmd', 0.0),
            ('cmd_tau', 0.0),
            ('cmd_notch', 0.0),
            ('e_dead', 0.0),
            # OPT-IN, 0.0 = off: cap the PUBLISHED angle at the
            # line-matching angle plus `pub_gate_corr` rad while no corner
            # (|kappa| >= `pub_gate_kappa`) is within `pub_gate_d` m ahead.
            # Stops the publish lead turning in on a curvature step it is
            # still metres away from.  See `mpc3_core.DEFAULTS`.
            ('pub_gate_corr', 0.0),
            ('pub_gate_d', 1.5),
            ('pub_gate_kappa', 0.30),
            # Release: no gate while the car is further than this off line
            # (a collision reset needs full authority to rejoin).
            ('pub_gate_e', 0.25),
            # --- straight-only weave conditioning (OPT-IN; see mpc3_core) ----
            # The 18 m straight carries a 2.9 Hz dead-time limit cycle of the
            # lateral loop.  `weave_notch_f` puts a second-order notch on the
            # published angle (unity gain and zero phase at DC, so it costs
            # nothing where there IS something to track); `straight_hf_gain` scales
            # only the FAST part of the angle (split at `straight_hf_tau`)
            # through the same gate, leaving DC untouched;
            # `straight_lead_scale` shortens the publish lead through it.  The gate is a RAMP: 1 on an empty
            # straight, 0 once a corner is inside `straight_d` m or the car is
            # `straight_e` off line (the collision-reset release).
            ('weave_notch_f', 0.0),
            ('weave_notch_k', 0.0),
            ('weave_notch_q', 1.0),
            ('straight_w_lat_scale', 1.0),
            ('straight_w_yaw_scale', 1.0),
            ('straight_hf_gain', 1.0),
            ('straight_hf_tau', 0.50),
            ('straight_lead_scale', 1.0),
            ('straight_pp_k', 0.0),
            ('straight_pp_ld_min', 1.5),
            ('straight_pp_ld_max', 5.0),
            ('straight_pp_td', 0.20),
            ('straight_pp_tk', 0.0),
            ('straight_pp_period_max', 0.0),
            ('barrier_margin_slow', 0.0),
            ('barrier_slow_period', 0.100),
            ('straight_d', 1.5),
            ('straight_kappa', 0.30),
            ('straight_e', 0.25),
            # --- s-indexed parameter schedule (OPT-IN, '' = off) -------------
            # Path to a JSON schedule (see `mpc3_core.ParamSchedule`) giving
            # per-arc-length overrides for a whitelisted subset of the knobs
            # above.  Inside the container the finals schedule lives at
            # /home/data/racelines/<name>.json, so it has to be in the
            # Dockerfile COPY list to ship.
            ('param_schedule', ''),
            ('sched_blend_m', 1.0),
            ('sched_hold_n', 5.0),
            ('sched_jump_m', 1.0),
            ('v_scale', 1.0),
            # --- the rack inside the rollout (OPT-IN; see mpc3_core) ---------
            # `steer_tau` is handed to BOTH the solver's rollout and the
            # `Actuator` that does the delay compensation, so there is one rack
            # model in the node.  `model_cmd_tau` 1.0 additionally puts the
            # `cmd_tau` output filter inside the rollout.
            ('steer_tau', 0.0),
            # `steer_tau_k` schedules that rack lag on the MEASURED period
            # (`steer_tau + steer_tau_k * (T - steer_tau_ref)`); -0.5 is what
            # the measured publish->rack lag implies against the CONSTANT
            # `cmd_dead_time`, 0.0 is shipping. See mpc3_core's DEFAULTS.
            ('steer_tau_k', 0.0),
            ('steer_tau_ref', 0.058),
            ('model_cmd_tau', 0.0),
            # --- publish lead: seconds ('temporal') or metres ('spatial') ----
            # 'spatial' reads the published angle a DISTANCE ahead
            # (`max(lead_min_m, lead_m0 + lead_k * v)`), which is
            # rate-invariant; `apply_ahead_s` and `rate_adaptive`'s `apply`
            # term are then inert, every other rate term still applies.  The
            # defaults are `apply_ahead_s` 0.090 s expressed as a distance,
            # NOT ForzaETH's pure-pursuit L1 -- see `mpc3_core.DEFAULTS`.
            # 'time' reads it `lead_t0 + lead_tk * period` SECONDS ahead, i.e.
            # a fixed number of SENSOR PERIODS -- the law that makes the driven
            # corner offset period-invariant (see `mpc3_core.DEFAULTS`).  Inert
            # unless `lead_mode` is set to 'time'.
            ('lead_mode', 'temporal'),
            ('lead_m0', 0.0),
            ('lead_k', 0.09),
            ('lead_min_m', 0.30),
            ('lead_t0', 0.0),
            ('lead_tk', 0.0),
            ('use_steer_feedback', True),
            ('steer_fb_age', 0.0),
            # --- steering authority ------------------------------------------
            # Hard cap on the published angle, `atan(L * ay_cap / v^2)`; the
            # rollout's own `ay_deliver` saturation does most of this work now.
            #
            # AUDIT (Sep 19, finals track, harness only -- NOT driven). This is
            # a Porto-era belt-and-braces and on this track it is the single
            # biggest remaining knob. It is NOT limiting corner ay: measured on
            # the two clean finals logs (`race_20260918_191411` display,
            # `_185202` Xvfb) the driven |ay| p99 is 6.82 / 6.36 against a tyre
            # that is linear to 9.81, and |cmd| sits within 5 % of this cap on
            # 33-43 % of samples at s 16-20 and s 40-44 (where the cap IS
            # `MAX_STEER`, so that is steering lock, not ay) and on 8-9 % at
            # s 32-36 at 5 m/s, where the cap really is 0.113 rad. What it
            # limits there is CORRECTION authority on the approach to the
            # corner that runs wide at the organisers' period.
            # Harness, `finals_mesh_hyb675b_race.csv`, shipping args,
            # notch-free, 9 seeds x 12 laps = 108 laps at EACH frame:
            #   9.0 (this) 12.945 / 12.485 s, 0+0 coll, clearance 0.125 / 0.025
            #   12.0       12.650 / 12.270,   0+0,      0.125 / 0.050
            #   16.0       12.620 / 12.030,   0+0,      0.175 / 0.100
            #   20.0       12.655 / 11.975,   0+0,      0.175 / 0.125
            # i.e. -0.33 s/lap at the organisers' 0.098 s AND more clearance
            # AND a smaller s 19 entry cut, with driven ay p99 unmoved
            # (6.34-6.43 on every arm, abort limit 9.3). 14.0 took 1 collision
            # in 36 laps at 0.058, so the ladder is not monotone -- these all
            # sit near the edge and a single 36-lap cell is not evidence.
            # Teleport stress (`--teleport-every 4`, 36 laps): base 41/55,
            # 12.0 24/44, 16.0 30/32, 20.0 43/45 -- no arm is worse, so the
            # old "do not remove the cap to get more steering" note (CLAUDE.md,
            # written for the LEGACY `mpc`, which had no saturation model and
            # ran ay_limit 4.5) does not reproduce here. Straight weave is
            # near-neutral (per-2 m actuator std 0.0225 -> 0.0249 at 0.058,
            # 0.0279 -> 0.0276 at 0.098): `pub_gate_corr` 0.05 already binds
            # tighter than this cap on the straight.
            # It is a SPEED-GAINING change and is left at 9.0 until it has
            # Gate C (100 Xvfb laps) and Gate D (32 display laps).
            ('ay_limit', 9.0),
            # `cap_ff_headroom` / `cap_preview_min_m`: never cap below the
            # LINE's own feedforward over the preview. Measured inert on this
            # track -- `1.5` is byte-identical to `1.15` at both frames,
            # because `1.5 * plan_ay` never reaches 9.0 anywhere on the lap.
            ('cap_ff_headroom', 1.15),
            ('cap_preview_min_m', 1.0),
            ('cap_use_measured', True),
            # --- pose ---------------------------------------------------------
            ('pose_blend', 1.0),          # 1.0 = take every sample whole
            ('blend_gate_m', 0.5),
            ('kappa_span_m', 0.5),
            ('track_window', 90),
            ('track_back', 3),
            ('track_yaw_tol', 1.2),
            ('track_lost_m', 1.0),
            ('max_predict_s', 0.35),
            ('stale_pose_s', 0.35),
            # --- telemetry GAP handling (OPT-IN; 0.0 = the coast that ships) --
            # A dropped telemetry frame is one sensor period of silence and is
            # harmless; three in a row (0.40 s at 0.104 s, 0.174 s at 0.058 s)
            # are longer than `stale_pose_s` and hand the car to the watchdog,
            # which holds the steering and cuts the throttle -- mid-corner that
            # is a collision.  With `gap_predict_s` > 0 the tracker instead
            # keeps solving from the last estimate, dead reckoned forward by
            # the same `Actuator.predict` the delay compensation uses, at the
            # plan's own timing, with the commanded speed ramped down at
            # `gap_decel` (never below `gap_v_min`).  Past
            # `stale_pose_s + gap_predict_s` the old coast takes over.
            ('gap_predict_s', 0.0),
            # What to publish while riding: 'solve' keeps the MPC on a
            # dead-reckoned pose (feedback on an estimate nobody measured);
            # 'ff' publishes the LINE's own steering angle at the dead-reckoned
            # arc length and no cross-track term at all.
            ('gap_mode', 'solve'),
            ('gap_decel', 3.0),           # m/s2 off the plan speed per second
            ('gap_v_min', 1.0),           # m/s floor of that ramp
            # Re-acquisition: the first pose after a gap comes out of a filter
            # that dead reckoned through the gap itself, so it can be a snap.
            # 0.0 = take it whole, which is what ships.
            ('gap_reacq_blend', 0.0),
            ('gap_reacq_gate_m', 0.0),    # 0.0 = keep `blend_gate_m`
            ('gap_reacq_n', 2),           # poses blended after a gap
            ('gap_reacq_gap_s', 0.0),     # 0.0 = one `stale_pose_s`
            # --- throttle (ServoThrottle) -------------------------------------
            ('servo_gain', 24.6),
            ('servo_k', 18.0),
            ('servo_drag', 0.273),
            ('servo_a_auth', 4.5),
            ('servo_a_accel', 5.0),
            ('servo_k_p', 0.0),
            ('servo_head_max', 0.5),
            ('servo_head_kappa', 0.0),   # 1/m; 0 = off. Fades servo_head_max out while turning
            # m; 0 = off.  No servo over-command when the offline profile BRAKES
            # (a_prof < -0.5) anywhere in the next this-many metres: the servo is
            # for corner EXITS; pushed up to plan speed INTO a braking zone the
            # car arrives hot at the slow sensor period (finals s 18-20: 1.11 ->
            # 1.24 s on Xvfb with the servo on).
            ('servo_decel_look_m', 0.0),
            # OPT-IN (0 = always on): fade the servo out when the measured
            # sensor period is past this value (linear ramp over 4 ms above it).
            # The servo pays on the display (0.058 s) and nothing at 0.098 s
            # (FINALS_TUNING: 'why the servo gains nothing at 0.098'); at the
            # slow end of the lottery it pushes the car wide on corner exits.
            ('servo_period_max', 0.0),
            # --- OPT-IN bounded residual policy ('' = off, byte-identical) ---
            # Path to a numpy .npz holding a small MLP (see
            # pit_crew/residual_policy.py).  It runs AFTER the solve and BEFORE
            # the publish guard, and can only add +-0.03 rad of steering and
            # scale the speed setpoint by 0.95-1.05 -- and not above 1.0 in a
            # corner or a declared slow zone.  Trained offline against the
            # log-fitted surrogate (tools/rl_train.py); it is not in any
            # shipping config and needs the usual two live gates.
            ('residual_policy', ''),
            ('servo_u_max', 0.60),
            ('servo_v_floor', 1.0),
            # OPT-IN real brake; 'none' is the shipping law byte for byte.
            # The sim brakes all four wheels when the throttle is EXACTLY 0.0
            # (VehicleController.Drive: DriveTorque == 0 -> brakeTorque =
            # MotorTorque); negative throttle is reverse drive, so 0.0 is the
            # only brake we have, and we have never published it.
            # 'zero' | 'pulse' -- see throttle2.ServoThrottle.
            ('brake_mode', 'none'),
            ('brake_a_min', 1.5),
            ('brake_dv', 0.3),
            ('brake_a_full', 4.6),
            # --- OPT-IN closed-loop PI throttle (all 0 = off, byte-identical).
            # A proportional+integral correction on (setpoint - encoder speed),
            # added as extra SPEED SETPOINT on top of the feedforward, to CLOSE
            # the ~0.5 m/s under-plan deficit accelerating out of slow corners.
            # Structure follows MonacoF1's public stack (PI on speed error,
            # integrator clamped to a traction-slip bound, on a 0.0408/(m/s)
            # feedforward), translated into ServoThrottle's servo-inverting
            # frame.  The prefilter keeps a step out of the loop, the slip clamp
            # bounds the integral, and the positive correction is faded by
            # `servo_head_kappa` (shared with the over-command) so exits do not
            # run wide.  Output stays in [0, u_max] (never reverse) and the
            # exact-0.0 brake is applied downstream -- see throttle2.
            ('thr_pi_kp', 0.0),
            ('thr_pi_ki', 0.0),
            ('thr_pi_prefilter_tau', 0.0),
            ('thr_pi_slip_clamp', 0.0),
            ('v_meas_min', 0.2),          # below this the encoder says nothing
            ('v_start', 0.5),             # plan speed before the encoder ever
                                          # reports -- see start_speed()
            # --- collision teleport recovery (OPT-IN, 0.0 = off) --------------
            # A checkpoint reset stops the car dead, but the wheel encoder does
            # NOT reset with it: measured over 100 live teleports in
            # data/logs (Sept 12), the encoder-derived speed reads a median
            # 1.79 m/s (p90 3.50, max 6.01) ABOVE the IPS ground speed for a
            # median 0.21 s (p90 0.82 s).  `SpeedEstimator.jumped`, which is
            # what `measured_speed()` waits for, needs a single sample above
            # 18 m/s and fired on 0 of those 100 teleports, so none of the
            # teleport handling below it ever ran.
            #
            # The consequence is not cosmetic.  `v0` sets the steering cap,
            # `atan(L * ay_cap / v^2)`: at a phantom 4 m/s with the car really
            # doing 1 m/s the cap is ~0.18 rad instead of the 0.52 rad of full
            # lock, exactly while the tracker has ~0.5 m of lateral error to
            # remove (measured reset offset from the racing line: p50 0.513 m,
            # p90 0.599 m).  It also stretches the horizon's arc steps by the
            # same factor -- the standing-start failure `start_speed()` already
            # documents, arriving by a different door.
            #
            # The fix needs no teleport DETECTION, which is just as well: the
            # published estimate does not jump at a teleport (its frame-to-
            # frame step is SMALLER than in normal driving, 0.23 m against
            # 0.38 m), so there is nothing to trigger on.  Instead bound the
            # encoder by an independent measurement of the same quantity, the
            # distance the localization estimate itself covered.  `v_pose_gate`
            # is how much faster than that the encoder may claim to be before
            # it is disbelieved; on 80 500 normal live samples a gate of
            # 1.5 m/s binds on 1.2% of them and 2.0 m/s on 0.6%.
            ('v_pose_gate', 0.0),         # m/s; 0 = off (shipping default)
            ('v_pose_min_dt', 0.03),      # s; ignore pose pairs closer than this
            # When the gate binds for `reindex_after` consecutive poses, also
            # drop the windowed reference search back to a global one.  The
            # window only looks `track_back` points (0.3 m) behind, and a
            # checkpoint reset moves the car up to 4.5 m BACKWARDS along the
            # line (measured: 50% of resets go backwards, p05 -3.39 m): the
            # index then stays ahead of the car.  Measured on the same 100
            # teleports, the windowed search disagrees with a global one by
            # more than 0.20 m on 12 of them, by more than 1 m on one.
            ('reindex_after', 0),         # poses; 0 = off (shipping default)
            # --- rate adaptivity (OPT-IN, identity at `rate_nominal`) ---------
            # The sensor period is a property of the MACHINE, not of the
            # tracker: 0.104 s on the dev host, 0.088 s in the submission
            # image, 0.097 s measured by the organisers.  mpc3's defaults were
            # validated at 0.104 s and several of them are really "so many
            # sensor periods"; see `mpc3_core.RateAdapter` for the isolation.
            ('rate_adaptive', False),
            ('rate_nominal', RATE_NOMINAL),
            ('rate_window', 20),          # poses in the robust median
            ('rate_min_change', 0.002),   # s; below this, do not re-point
            ('rate_terms', ','.join(RATE_TERMS)),
            # Trust band on the MEASURED period, as a ratio of `rate_nominal`.
            # 0.5-1.5 x 0.104 s is [0.052, 0.156]; outside it `RateControl`
            # clamps the PERIOD, so what is applied is the tune for 0.052 s (or
            # 0.156 s), NOT the validated 0.104 s one -- and below 0.069 s
            # `apply_floor_steps` makes the `apply` term inert anyway, so every
            # period from 0.040 to 0.069 s publishes the same 0.060 s lead.  The
            # brief for the finals is that the organisers' machine may sit
            # anywhere in 0.05-0.11 s, and 0.050 s is BELOW that floor -- it
            # would silently race the 0.104 s tune at 20 Hz.  Exposed so the
            # floor can be moved from the launch line without a rebuild.
            ('rate_lo', 0.5),
            ('rate_hi', 1.5),
            # OPT-IN bound on how fast the APPLIED period may follow the
            # measured one, in seconds of period per second.  0.0 is off and
            # byte-identical.  `RateEstimator` is a median, so a mid-run drift
            # (an OBS recording starting, say) is not followed smoothly: it is
            # held for half a window and then arrives in two samples.  Measured
            # on a 0.092 -> 0.110 s step at window 20: 0.83 s of nothing, then
            # the whole 0.018 s in two samples -- a 0.0156 s step in
            # `apply_ahead_s` and a 0.039 s step in `thr_lead_s` inside one
            # frame.  0.01 spreads that same step over 1.8 s.
            ('rate_slew', 0.0),
            # --- iterative learning control (OPT-IN) --------------------------
            ('ilc_enable', False),
            ('ilc_bin_m', 0.25),
            ('ilc_k_gain', 0.30),
            ('ilc_q_span_m', 0.50),
            ('ilc_k_max', 0.15),
            ('ilc_shift_max_m', 0.08),
            ('ilc_shift_len_m', 1.50),
            ('ilc_clear_target', 0.30),
            ('ilc_clear_band', 0.10),
            ('ilc_clear_loss', 0.05),
            # The speed-ceiling table is a NET LOSS in the harness even with
            # `set_nominal_clearance` wired: 12 laps on the safe line gave
            # 7.86 -> 8.09 s with it and 7.86 -> 7.76 s without, and the
            # steering table alone still halved |e_lat| p95.  It is off by
            # default; `ilc_enable` on its own means the steering table only.
            ('ilc_v_enable', False),
            ('ilc_v_down', 1.0),
            ('ilc_v_up', 0.005),
            ('ilc_v_lo', 0.85),
            ('ilc_v_hi', 1.05),
            # Persistence is OPT-IN, and empty by default so the node and
            # `tools/sim_tracker.py` agree: with a store, run N+1 starts on
            # run N's table, so the harness (which never had one) would no
            # longer be measuring what races.  Set it deliberately, per line.
            ('ilc_store', ''),
            # --- control schedule (OPT-IN: 'timer') ---------------------------
            # 'per_pose' is the validated structure: solve inside the pose
            # callback, so the control rate IS the sensor rate and the command
            # is held for one sensor period.  That is what makes several
            # constants implicitly per-period (see `RateAdapter`), and the
            # sensor period is a per-start lottery of the simulator
            # (0.088-0.106 s measured on one host).  'timer' is the Autoware
            # structure instead: a fixed wall-clock solve, so the command is
            # re-issued every `1/control_hz` whatever the sensor does, the pose
            # callback only stores the measurement, and the only thing the
            # sensor period still sets is how FRESH the estimate is.
            ('control_mode', 'per_pose'),
            ('control_hz', 20.0),
            # Timer mode only.  The devkit bridge consumes ONE command per
            # telemetry tick, so a command published off that grid waits
            # 0..one period before it is latched and the commands in between
            # are dropped.  Solving in the pose callback hides that (one
            # publish per tick, at a fixed phase); solving on a wall clock does
            # not, and the `Actuator`'s constant dead time then models commands
            # that never acted.  With this on, a command is recorded at the
            # tick it will actually be latched at -- the pose STAMP is a tick,
            # so the grid is observable -- and superseded ones fall out because
            # `Actuator._target_at` keeps the last entry with equal stamps.
            # Measured (12 laps x 3 seeds, frame 0.104): off -> |e| p50 0.136,
            # 19 collisions; on -> 0.092 and 6.
            ('timer_sync_dead', True),
            # 0.0 = one tick period, which is what 'timer' mode means by
            # `apply_ahead_s`: the command published now is held for exactly
            # one tick, so the angle worth publishing is the plan's angle one
            # tick after it bites.  Set it explicitly to sweep it.
            ('timer_apply_ahead_s', 0.0),
            # --- safety --------------------------------------------------------
            ('stuck_seconds', 1.5),
            ('stuck_distance', 0.20),
            ('escape_seconds', 1.2),
            ('watchdog_hz', 20.0),
        ])
        v = {n.name: n.value for n in p}
        self.servo_decel_look_m = float(v['servo_decel_look_m'])
        self.servo_period_max = float(v['servo_period_max'])
        self.P = v

        # OPT-IN residual policy.  `load` returns None for an empty parameter
        # and RAISES for a path that does not exist: a policy that was asked
        # for and silently did not run would be the worst outcome of the two.
        from pit_crew import residual_policy as _rp
        self.residual = _rp.load(v['residual_policy'])
        if self.residual is not None:
            self.get_logger().info(
                f"residual policy {v['residual_policy']}: "
                f'{self.residual.n_obs} inputs, '
                f'steer +-{self.residual.steer_max:.3f} rad, '
                f'v x[{self.residual.v_lo:.2f},{self.residual.v_hi:.2f}], '
                f'no speed-up in {len(self.residual.zones)} zones or above '
                f'|kappa| {self.residual.kappa_gate:.2f}')

        self.path = RefPath(v['raceline_csv'], kappa_span_m=v['kappa_span_m'],
                            speed_scale=float(v['speed_scale']))
        prefix = v['map_prefix'] or derive_map_prefix(v['raceline_csv'])
        if v['use_barrier'] and prefix:
            try:
                from pit_crew import pf_core
                dist, _img, meta = pf_core.load_map(prefix)
                self.path.load_corridor(dist, meta)
                self.get_logger().info(
                    f'corridor from {prefix}: min half-width '
                    f'{min(self.path.w_left.min(), self.path.w_right.min()):.2f} m')
            except Exception as exc:                      # noqa: BLE001
                self.get_logger().error(f'no corridor barrier ({exc})')
        elif v['use_barrier']:
            self.get_logger().error(
                f"no map found for {v['raceline_csv']}; pass map_prefix")
        # `w_barrier` is 400 against `w_lat` 24: a silently disabled corridor is
        # not the configuration anything was validated in, and the only reason
        # it ever happened was a derived map prefix that did not resolve. Fail
        # loudly instead -- the entrypoint now passes `map_prefix` explicitly.
        if v['use_barrier'] and v['require_barrier'] and not self.path.has_corridor:
            raise RuntimeError(
                f"corridor barrier required but no map loaded (prefix "
                f"{prefix!r}); pass -p map_prefix:=<prefix> or "
                f"-p require_barrier:=false to race without it")

        self.mpc = MPC3(
            self.path,
            horizon=int(v['horizon']), step_dt=v['step_dt'],
            min_step_m=v['min_step_m'], iters=int(v['iters']),
            outer=int(v['outer']), w_lat=v['w_lat'], w_yaw=v['w_yaw'],
            w_steer=v['w_steer'], w_rate=v['w_rate'], w_jerk=v['w_jerk'],
            w_lat_T=v['w_lat_T'], w_yaw_T=v['w_yaw_T'],
            w_barrier=v['w_barrier'], barrier_margin=v['barrier_margin'],
            yaw_gain=v['yaw_gain'], ay_deliver=v['ay_deliver'],
            sat_knee=v['sat_knee'], ay_budget=v['ay_budget'],
            a_brake=v['a_brake'], a_accel=v['a_accel'], e_free=v['e_free'],
            k_e=v['k_e'], v_mod_min=v['v_mod_min'],
            e_cut_accel_scale=v['e_cut_accel_scale'],
            e_cut_accel_kappa=v['e_cut_accel_kappa'],
            thr_lead_s=v['thr_lead_s'], thr_lead_mode=v['thr_lead_mode'],
            thr_source=v['thr_source'],
            thr_brake_lead_s=v['thr_brake_lead_s'],
            thr_enc_tau=v['thr_enc_tau'], delay_s=v['delay_s'],
            apply_ahead_s=v['apply_ahead_s'],
            apply_floor_steps=v['apply_floor_steps'],
            apply_lead_steps=v['apply_lead_steps'],
            nd_mode=v['nd_mode'], nd_delay_s=v['nd_delay_s'],
            dead_tick_c=v['dead_tick_c'],
            w_dcmd=v['w_dcmd'], cmd_tau=v['cmd_tau'],
            cmd_notch=v['cmd_notch'], e_dead=v['e_dead'],
            pub_gate_corr=v['pub_gate_corr'], pub_gate_d=v['pub_gate_d'],
            pub_gate_kappa=v['pub_gate_kappa'],
            pub_gate_e=v['pub_gate_e'],
            weave_notch_f=v['weave_notch_f'], weave_notch_k=v['weave_notch_k'],
            weave_notch_q=v['weave_notch_q'],
            straight_w_lat_scale=v['straight_w_lat_scale'],
            straight_w_yaw_scale=v['straight_w_yaw_scale'],
            straight_hf_gain=v['straight_hf_gain'],
            straight_hf_tau=v['straight_hf_tau'],
            straight_lead_scale=v['straight_lead_scale'],
            straight_pp_k=v['straight_pp_k'], straight_pp_ld_min=v['straight_pp_ld_min'],
            straight_pp_ld_max=v['straight_pp_ld_max'], straight_pp_td=v['straight_pp_td'], straight_pp_tk=v['straight_pp_tk'], straight_pp_period_max=v['straight_pp_period_max'],
            barrier_margin_slow=v['barrier_margin_slow'], barrier_slow_period=v['barrier_slow_period'],
            straight_d=v['straight_d'], straight_kappa=v['straight_kappa'],
            straight_e=v['straight_e'],
            param_schedule=str(v['param_schedule']),
            sched_blend_m=v['sched_blend_m'],
            sched_hold_n=int(round(float(v['sched_hold_n']))),
            sched_jump_m=v['sched_jump_m'], v_scale=v['v_scale'],
            steer_tau=v['steer_tau'], steer_tau_k=v['steer_tau_k'],
            steer_tau_ref=v['steer_tau_ref'],
            model_cmd_tau=v['model_cmd_tau'],
            lead_mode=v['lead_mode'], lead_m0=v['lead_m0'],
            lead_k=v['lead_k'], lead_min_m=v['lead_min_m'],
            lead_t0=v['lead_t0'], lead_tk=v['lead_tk'],
            steer_rate=v['steer_rate_limit'])
        self.act = Actuator(dead_time=float(v['cmd_dead_time']),
                            rate=float(v['steer_rate_limit']),
                            steer_tau=float(v['steer_tau']))
        self.filt = PoseFilter(self.act, blend=float(v['pose_blend']),
                               gate_m=float(v['blend_gate_m']))
        # The brake is the single most load-bearing knob on this stack: without
        # `brake_mode:=zero` the finals config took 10 collisions in 11-14
        # display laps (a DQ), and `ServoThrottle` treats ANY string that is
        # not 'none' or 'pulse' as 'zero', so a typo brakes but an unknown
        # spelling is never reported. Reject it at construction, the same way
        # `control_mode` is, and say which mode is in force in the ready line
        # so a launch that lost the flag is visible in the first second of log.
        if str(v['brake_mode']) not in ('none', 'zero', 'pulse'):
            raise RuntimeError(
                f"brake_mode must be 'none', 'zero' or 'pulse', not "
                f"{v['brake_mode']!r}")
        self.thr = ServoThrottle(
            K=v['servo_k'], drag=v['servo_drag'], tau=v['cmd_dead_time'],
            a_auth=v['servo_a_auth'], a_accel=v['servo_a_accel'],
            k_p=v['servo_k_p'], head_max=v['servo_head_max'],
            head_kappa=v['servo_head_kappa'],
            lead_max=0.0, ay_limit=v['ay_budget'], v_floor=v['servo_v_floor'],
            u_max=v['servo_u_max'], servo_gain=v['servo_gain'],
            brake_mode=str(v['brake_mode']), brake_a_min=v['brake_a_min'],
            brake_dv=v['brake_dv'], brake_a_full=v['brake_a_full'],
            pi_kp=v['thr_pi_kp'], pi_ki=v['thr_pi_ki'],
            pi_prefilter_tau=v['thr_pi_prefilter_tau'],
            pi_slip_clamp=v['thr_pi_slip_clamp'])
        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)
        # Measures the pose period always; only ACTS on it when rate_adaptive.
        self.rate_ctl = RateControl(
            self.mpc, self.act, enabled=bool(v['rate_adaptive']),
            filt=self.filt, speed_est=self.speed_est, thr=self.thr,
            nominal=float(v['rate_nominal']), window=int(v['rate_window']),
            terms=tuple(str(v['rate_terms']).split(',')),
            min_change=float(v['rate_min_change']),
            rate_lo=float(v['rate_lo']), rate_hi=float(v['rate_hi']),
            rate_slew=float(v['rate_slew']),
            cmd_dead_time=float(v['cmd_dead_time']),
            delay_s=float(v['delay_s']),
            apply_ahead_s=float(v['apply_ahead_s']),
            thr_lead_s=float(v['thr_lead_s']),
            max_predict_s=float(v['max_predict_s']),
            stale_pose_s=float(v['stale_pose_s']),
            enc_alpha=self.speed_est.alpha, pose_blend=float(v['pose_blend']),
            enc_tau=float(v['thr_enc_tau']),
            dead_tick_c=float(v['dead_tick_c']))
        self._rate_logged = 0.0

        self.ilc = None
        self._ilc_prev_idx = None
        if v['ilc_enable']:
            store = str(v['ilc_store'])
            if store:
                store = (f"{store}_{os.path.splitext(os.path.basename(v['raceline_csv']))[0]}"
                         f".npz")
            self.ilc = ILC(
                float(self.path.s[-1] + self.path.spacing),
                bin_m=float(v['ilc_bin_m']), k_gain=float(v['ilc_k_gain']),
                q_span_m=float(v['ilc_q_span_m']), k_max=float(v['ilc_k_max']),
                shift_max_m=float(v['ilc_shift_max_m']),
                shift_len_m=float(v['ilc_shift_len_m']),
                clear_target=float(v['ilc_clear_target']),
                clear_band=float(v['ilc_clear_band']),
                clear_loss=float(v['ilc_clear_loss']),
                v_enable=bool(v['ilc_v_enable']),
                v_down=float(v['ilc_v_down']), v_up=float(v['ilc_v_up']),
                v_lo=float(v['ilc_v_lo']), v_hi=float(v['ilc_v_hi']),
                store=store)
            # WITHOUT this the speed target is a flat `ilc_clear_target`, and
            # on a line whose corridor is under that by construction every bin
            # scores "tight" every lap and the table ratchets to the `v_lo`
            # floor -- measured offline at 7.11 -> 8.28 s with the error still
            # converging.  The harness has always called it; the node did not.
            self.ilc.set_nominal_clearance(self.path)
            self.mpc.ilc = self.ilc
            self.get_logger().info(
                f'ILC on: {self.ilc.nb} bins of {v["ilc_bin_m"]:.2f} m, '
                f'|kff| <= {self.ilc.k_clamp:.3f} 1/m, speed table '
                f'{"on" if v["ilc_v_enable"] else "OFF"} '
                f'[{v["ilc_v_lo"]:.2f}, {v["ilc_v_hi"]:.2f}], clearance target '
                f'{"line-capped" if self.ilc.nominal_clear is not None else "flat"}'
                f', {self.ilc.laps} lap(s) restored from '
                f'{store or "(no store)"}')

        # The solve schedule.  Validated default is 'per_pose'; nothing below
        # runs at all in that mode, so it is bit-identical to the shipping
        # controller.
        self.control_mode = str(v['control_mode']).strip().lower()
        if self.control_mode not in ('per_pose', 'timer'):
            raise RuntimeError(
                f"control_mode must be 'per_pose' or 'timer', not "
                f"{v['control_mode']!r}")
        self.control_period = 1.0 / max(float(v['control_hz']), 1.0)
        # What is left of the measured command-to-movement delay once the
        # half-tick a latched command waits at the pickup boundary is taken
        # out; the same decomposition `RateAdapter` uses.
        self._bridge = (float(v['cmd_dead_time'])
                        - 0.5 * float(v['rate_nominal']))
        if self.control_mode == 'timer':
            # The two are alternative answers to the same problem and they
            # fight: `rate_adaptive` re-points the per-period constants at the
            # MEASURED sensor period, and in timer mode nothing the controller
            # does is timed by the sensor any more.  Timer mode wins; the
            # estimator keeps measuring so the period is still logged.
            if v['rate_adaptive']:
                self.rate_ctl.enabled = False
                self.get_logger().warn(
                    'rate_adaptive is ignored in control_mode=timer: the '
                    'control period is already fixed')
            ahead = (float(v['timer_apply_ahead_s'])
                     or self.control_period)
            # ONLY `apply_ahead_s`.  `delay_s` and `step_dt` are properties of
            # the actuator and of the horizon, not of the schedule, so `nd =
            # round(delay_s / step_dt)` is deliberately untouched.
            self.mpc.set_rate(apply_ahead_s=ahead)
            if v['timer_sync_dead']:
                # Commands are stamped with the tick they are latched at, so
                # what is left to model is the bridge alone.
                self.act.dead_time = self._bridge
        if str(v['lead_mode']).strip().lower() == 'spatial':
            # Not a conflict, but it changes what `rate_adaptive` is doing:
            # the publish index is a distance now, so the `apply` term has
            # nothing to scale.  Every other rate term still applies.
            self.get_logger().info(
                'lead_mode=spatial: publish index is '
                f"max({float(v['lead_min_m']):.2f}, "
                f"{float(v['lead_m0']):+.2f} + {float(v['lead_k']):.2f} v) m "
                'ahead of the first free step; apply_ahead_s (and '
                "rate_adaptive's `apply` term) is inert")

        # Telemetry-gap degraded mode (OPT-IN; inert at gap_predict_s 0.0).
        self.gap = GapRider(
            predict_s=float(v['gap_predict_s']), decel=float(v['gap_decel']),
            v_min=float(v['gap_v_min']),
            reacq_blend=float(v['gap_reacq_blend']),
            reacq_gate_m=float(v['gap_reacq_gate_m']),
            reacq_n=int(v['gap_reacq_n']),
            reacq_gap_s=float(v['gap_reacq_gap_s']))
        self._gap_solve_t = -1e9
        self._gap_ff = str(v['gap_mode']).strip().lower() == 'ff'

        self.pose = None
        self.gyro_z = 0.0
        self.v_plan0 = 0.0           # profile speed at the car (encoder fallback)
        self._enc_seen = False       # has the encoder ever reported motion?
        self.steer = 0.0
        self.steer_fb = None
        self._idx = None
        self._trail = []
        self._pose_prev = None       # (stamp, x, y) of the last ACCEPTED pose
        self._v_pose = None          # speed implied by the estimate itself
        self._gate_run = 0           # consecutive poses with the gate binding
        self._escape_until = None
        self._escape_steer = 0.0
        self._stuck_count = 0
        self._stale_warned = False
        self._solve_ms = 0.0
        self._solves = 0
        self._pose_drops = 0
        self._pose_ok = 0
        self._drop_log_t = 0.0
        self._solve_fail = 0
        self._nonfinite_cmd = 0
        self._cb_fail = 0
        self._slow_solves = 0

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(PoseStamped, '/pit_crew/pose',
                                 self._guarded(self.on_pose, 'pose'), 1)
        self.create_subscription(Imu, f'{ns}/imu',
                                 self._guarded(self.on_imu, 'imu'), qos)
        self.create_subscription(Float32, f'{ns}/steering',
                                 self._guarded(self.on_steer_fb, 'steer_fb'),
                                 qos)
        for side in ('left', 'right'):
            self.create_subscription(
                JointState, f'{ns}/{side}_encoder',
                self._guarded(lambda m, w=side: self.on_enc(w, m),
                              f'{side}_encoder'), qos)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.pub_cte = self.create_publisher(Float32, '/pit_crew/cte', 10)
        self.create_timer(1.0 / max(float(v['watchdog_hz']), 1.0),
                          self._guarded(self.watchdog, 'watchdog'))
        if self.control_mode == 'timer':
            self.create_timer(self.control_period,
                              self._guarded(self.on_control_tick, 'tick'))
            self.get_logger().info(
                f'control schedule: TIMER at {1.0 / self.control_period:.1f} Hz '
                f'(apply_ahead {self.mpc.cfg["apply_ahead_s"]:.3f} s, nd '
                f'{self.mpc.nd}); the pose callback only stores the estimate')
        self.get_logger().info(
            f"mpc3 ready: {self.path.n} pts, horizon {v['horizon']}x"
            f"{v['step_dt']:.3f}s ({self.mpc.nd} committed), delay "
            f"{v['delay_s']:.3f}s, ay_budget {v['ay_budget']:.1f}, ay_limit "
            f"{float(v['ay_limit']):.1f}, barrier "
            f"{'on' if self.path.has_corridor else 'off'}, brake "
            f"{v['brake_mode']}"
            + (f" (a_full {float(v['brake_a_full']):.1f}, dv "
               f"{float(v['brake_dv']):.2f}, a_min "
               f"{float(v['brake_a_min']):.1f})"
               if str(v['brake_mode']) != 'none' else ' — NO BRAKE'))

    # ------------------------------------------------------------------
    def _guarded(self, fn, what):
        """Wrap an rclpy callback so one exception cannot end the race.

        An exception raised inside a subscription or timer callback unwinds
        `rclpy.spin()` and the process exits; the submission entrypoint then
        sees a dead node, which is a DQ. `control_step` already guards the
        solve for exactly this reason, but everything AROUND it -- the pose
        filter, the reference search, the rate estimator, the encoder decode,
        the watchdog's own publish -- was unguarded, so a single bad message or
        an IndexError at a wrap could still take the node down mid-race.

        This wraps the callback only. It changes nothing on the normal path
        (no exception is raised there) and it deliberately does NOT publish
        anything: the watchdog, which runs on its own timer, is the one place
        the hold-steer/zero-throttle command comes from.
        """
        def run(*a, **kw):
            try:
                return fn(*a, **kw)
            except Exception as exc:                      # noqa: BLE001
                self._cb_fail += 1
                if self._cb_fail in (1, 10) or self._cb_fail % 100 == 0:
                    self.get_logger().error(
                        f'{what} callback failed ({exc!r}); the watchdog still '
                        f'holds the actuators (#{self._cb_fail})')
                return None
        return run

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg: Imu):
        if math.isfinite(msg.angular_velocity.z):
            self.gyro_z = float(msg.angular_velocity.z)

    def on_enc(self, wheel, msg: JointState):
        if msg.position:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(wheel, t, msg.position[0])

    def on_steer_fb(self, msg: Float32):
        """The real actuator angle, in radians -- a legal runtime input."""
        if not math.isfinite(msg.data):
            return
        t = self.now()
        self.steer_fb = (t, float(msg.data))
        if self.P['use_steer_feedback']:
            self.act.snap(t - float(self.P['steer_fb_age']), float(msg.data))

    def measured_speed(self):
        """Encoder speed, or None when it has nothing to say.

        mpc3 needs this: the speed plan starts from the speed the car HAS, and
        the whole point of the tracker is that it may choose a different one.
        The estimator is the calibrated one (bounds +9.0/-3.0, per-wheel
        filters, reject-with-persistence); a teleport clears it.
        """
        if self.speed_est.consume_jump():
            # A collision teleport puts the car AT REST at a checkpoint, so
            # everything the latch implies about the car is stale too. Clearing
            # it sends `start_speed` back to the walking-pace standing-start
            # branch; leaving it set planned the whole recovery at the profile
            # speed (6-7 m/s) with the car stopped, which is the lap-0 failure
            # this class of fix already removed once.
            self._enc_seen = False
            # The arc length the schedule indexes on is not trustworthy across
            # a teleport either: fall back to the global parameters and ramp
            # back in (a no-op unless `param_schedule` is set).
            _m = getattr(self, 'mpc', None)
            if _m is not None:
                _m.hold_schedule()
            ilc = getattr(self, 'ilc', None)
            if ilc is not None:
                ilc.invalidate()        # this lap is not repeatable evidence
            return None
        v = self.speed_est.speed
        gate = float(self.P['v_pose_gate'])
        if gate > 0.0 and self._v_pose is not None and v > self._v_pose + gate:
            # The encoder is claiming more ground speed than the localization
            # estimate has covered.  Believe the smaller one; see `v_pose_gate`.
            self._gate_run += 1
            v = self._v_pose
        else:
            self._gate_run = 0
        if v > float(self.P['v_meas_min']):
            self._enc_seen = True
            return v
        return None

    def start_speed(self):
        """What to plan from when the encoder says nothing.

        NOT the profile.  At a standing start the encoder is silent and the
        profile at the grid can be 7 m/s, and every consumer of `v` then gets
        it wrong at once: the horizon's arc steps are ten times too long, so
        1.2 s of preview covers 8 m of track instead of 0.6 m, and the
        delivered-curvature saturation `ay_deliver / v^2` is evaluated at 7 m/s
        (0.19 1/m) instead of at walking pace (effectively unlimited), so the
        model believes the car cannot turn in at all.  That is the lap-0
        collision at s 29.2 on safe_ay7.0 -- four of the five in that run, all
        in the first metres off the line, in a bend of only 0.18 1/m.

        Before the encoder has ever spoken, plan from a walking pace.  After it
        has, a momentary silence is a dropout and the profile is the right
        guess.
        """
        return self.v_plan0 if self._enc_seen else float(self.P['v_start'])

    # ------------------------------------------------------------------
    def on_pose(self, msg: PoseStamped):
        mx, my = msg.pose.position.x, msg.pose.position.y
        myaw = yaw_from_quat(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.now()
        if not all(math.isfinite(q) for q in (mx, my, myaw, stamp)):
            return self.drop_pose('non-finite', now)
        R = self.rate_ctl.values
        if stamp <= 0.0 or stamp > now + 0.1 or now - stamp > R['stale_pose_s']:
            return self.drop_pose(f'stale by {now - stamp:.2f}s', now)
        if self.pose is not None and stamp <= self.pose[3]:
            return self.drop_pose('out of order', now)
        # Only an ACCEPTED pose is evidence about the sensor period.  Feeding
        # the rejects in let a `stamp == 0` pose anchor the estimator, and made
        # it measure a stream the tracker does not actually control on.
        self.rate_ctl.update(stamp)
        self._pose_ok += 1
        if float(self.P['v_pose_gate']) > 0.0:
            pv = self._pose_prev
            dt = (stamp - pv[0]) if pv is not None else 0.0
            self._v_pose = (math.hypot(mx - pv[1], my - pv[2]) / dt
                            if dt > float(self.P['v_pose_min_dt']) else None)
            self._pose_prev = (stamp, mx, my)
        if self.v_plan0 <= 0.0:
            j = self.path.nearest(mx, my, None, yaw=myaw)
            self.v_plan0 = float(self.path.v_prof[j])
        v = self.measured_speed() or self.start_speed()
        # Re-acquisition guard: after a long gap the arriving pose is blended
        # against our own propagation for a few frames instead of snapped to.
        # Identity while `gap_reacq_blend` is 0.0.
        self.gap.see_pose(stamp, R['stale_pose_s'])
        keep = (self.filt.blend, self.filt.gate)
        self.filt.blend, self.filt.gate = self.gap.filter_gains(*keep)
        try:
            x, y, yaw = self.filt.update(stamp, mx, my, myaw, max(v, 0.4))[:3]
        finally:
            self.filt.blend, self.filt.gate = keep
        self.pose = (x, y, yaw, stamp)
        self._trail.append((stamp, mx, my))
        while self._trail and stamp - self._trail[0][0] > self.P['stuck_seconds']:
            self._trail.pop(0)
        if self.control_mode == 'per_pose':
            self.control_step()

    def pickup_time(self, t, ts):
        """The tick at which a command published at ``t`` is actually latched.

        ``ts`` is the newest pose stamp, which IS a telemetry tick, and the
        period is the one ``RateControl`` measures whether or not it is allowed
        to act on it.  Before the estimator is ready this is the identity, i.e.
        the per-pose behaviour.
        """
        p = float(self.rate_ctl.est.period)
        if not (p > 1e-3) or not math.isfinite(ts):
            return t
        k = max(1, int(math.ceil((t - ts) / p - 1e-9)))
        return ts + k * p

    def on_control_tick(self):
        """One solve on the wall clock (``control_mode`` 'timer' only).

        The pose callback has already stored and blended the newest estimate;
        this propagates it to NOW inside ``_control_step`` exactly as the
        per-pose path does -- same ``Actuator.predict``, same measured speed and
        gyro, only ``t_now`` is the tick instant instead of the callback
        instant.  Nothing here depends on the sensor period.

        The guard is the same one the watchdog uses: with no fresh pose there is
        nothing to solve from, so skip the tick and let the watchdog hold the
        steering and coast.  Skipping rather than publishing keeps the coast in
        one place.
        """
        if self.pose is None:
            return
        if self.now() - self.pose[3] > self.rate_ctl.values['stale_pose_s']:
            return
        self.control_step()

    def drop_pose(self, why, now):
        """Dropping every pose used to be silent, and a silent stack is a
        stopped car with an empty log. One line a second, no more."""
        self._pose_drops += 1
        if now - self._drop_log_t >= 1.0:
            self._drop_log_t = now
            log = self.get_logger()
            # Never having accepted one is the fatal case (a clock or topic
            # mismatch: the car simply never moves); the odd duplicate is not.
            (log.error if self._pose_ok == 0 else log.warn)(
                f'dropped pose ({why}); {self._pose_drops} dropped, '
                f'{self._pose_ok} accepted so far')

    # ------------------------------------------------------------------
    def is_stuck(self):
        if len(self._trail) < 5 or self.v_plan0 < 0.3:
            return False
        t0, x0, y0 = self._trail[0]
        if self._trail[-1][0] - t0 < self.P['stuck_seconds'] * 0.9:
            return False
        _, x1, y1 = self._trail[-1]
        return math.hypot(x1 - x0, y1 - y0) < self.P['stuck_distance']

    # ------------------------------------------------------------------
    # telemetry-gap degraded mode (OPT-IN: `gap_predict_s`; see GapRider)
    # ------------------------------------------------------------------
    def predict_window(self):
        """Seconds of dead reckoning allowed from the pose stamp.

        `max_predict_s` normally; while riding a gap, the whole gap window, so
        that the SAME solve runs on a pose propagated further rather than on a
        pose frozen at `max_predict_s` ago.  Identity with the feature off.
        """
        w = float(self.rate_ctl.values['max_predict_s'])
        if self.gap.enabled:
            w = max(w, self.gap.window(
                float(self.rate_ctl.values['stale_pose_s'])))
        return w

    def gap_v_cmd(self, v_cmd, now):
        """The plan speed, ramped down for every second driven blind."""
        if not self.gap.enabled or self.pose is None:
            return v_cmd
        return self.gap.v_cap(now - self.pose[3],
                              float(self.rate_ctl.values['stale_pose_s']),
                              v_cmd)

    def gap_ride(self, now, age):
        """One degraded control cycle, at the sensor's own period.

        The watchdog runs at 20 Hz and a solve is 3-6 ms, but publishing four
        times per sensor period would also be four `Actuator.record` entries
        the bridge never latches; keep the plan's own timing.
        """
        if not self.gap.ride(age, self.rate_ctl.values['stale_pose_s']):
            return False
        period = max(float(self.rate_ctl.est.period), 0.02)
        if now - self._gap_solve_t < period:
            return True                  # riding, just not solving this tick
        self._gap_solve_t = now
        self.gap.rides += 1
        if self._gap_ff:
            self.gap_publish_ff(now, age)
            return True
        if self.gap.rides == 1 or self.gap.rides % 20 == 0:
            self.get_logger().warn(
                f'telemetry gap {age:.2f}s — driving the plan open loop '
                f'(ride {self.gap.rides})')
        self.control_step()
        return True

    def gap_publish_ff(self, now, age):
        """`gap_mode` 'ff': the plan's own angle at the dead-reckoned point."""
        stale = float(self.rate_ctl.values['stale_pose_s'])
        v = self.measured_speed()
        v0 = max(v if v is not None else self.start_speed(), 0.4)
        i0 = self._idx if self._idx is not None else 0
        ds = self.gap.arc_ahead(age, stale, v0)
        v_cap = self.gap.v_cap(age, stale, v0)
        j, delta = gap_feedforward(self.path, i0, ds,
                                   yaw_gain=float(self.P['yaw_gain']),
                                   cap=math.atan(
                                       vehicle.WHEELBASE
                                       * float(self.P['ay_limit'])
                                       / max(v0 * v0, 0.25)))
        th = float(self.thr.command(min(float(self.path.v_prof[j]), v_cap),
                                    v, 0.0, now=now,
                                    kappa=float(self.path.kappa[j])))
        self.publish(delta, th)

    def watchdog(self):
        """Runs whether or not poses arrive; commands LATCH in this simulator."""
        if self.pose is None:
            return
        now_wd = self.now()
        age = now_wd - self.pose[3]
        if self.gap_ride(now_wd, age):
            return
        if age > self.rate_ctl.values['stale_pose_s']:
            # Hold the steering angle (mid-corner, zero steer points at the
            # outside wall) and coast.  Zero throttle is a ~5.8 m/s2 brake here,
            # which is exactly what is wanted when the pose has gone.
            self.publish(self.steer, 0.0)
            if not self._stale_warned:
                self.get_logger().error(
                    f"no pose for {self.rate_ctl.values['stale_pose_s']:.2f}s"
                    " — coasting to a stop")
                self._stale_warned = True
            return
        self._stale_warned = False
        now = self.now()
        if self._escape_until is not None and now < self._escape_until:
            self.publish(self._escape_steer, -0.25)

    def plan_ay(self, i0, v):
        """Largest lateral acceleration the LINE itself plans, just ahead.

        In the ay domain, not the steering-angle domain: the same budget is a
        large angle at hairpin speed and a small one at straight speed, so a
        preview that reaches into the next hairpin cannot licence a huge angle
        on the straight before it.
        """
        P = self.path
        span = max(float(self.P['cap_preview_min_m']),
                   v * self.mpc.N * self.mpc.dt)
        n = max(2, int(span / max(P.spacing, 1e-3)))
        idx = (i0 + np.arange(n)) % P.n
        return float((P.v_prof[idx] ** 2 * np.abs(P.kappa[idx])).max())

    # ------------------------------------------------------------------
    def control_step(self):
        """One control cycle, guarded.

        An exception in an rclpy callback unwinds `spin()` and kills the
        process, and the submission entrypoint treats a dead node as a dead
        race. Nothing in here is worth that: hold the steering and coast.
        """
        try:
            self._control_step()
        except Exception as exc:                          # noqa: BLE001
            self.get_logger().error(f'control step failed ({exc!r}); '
                                    'holding steer, zero throttle')
            self.publish(self.steer, 0.0)

    def _control_step(self):
        t_solve = time.perf_counter()
        now = self.now()
        if self._escape_until is not None:
            if now < self._escape_until:
                self.publish(self._escape_steer, -0.25)
                return
            self._escape_until = None
            self._trail.clear()

        x, y, yaw, t = self.pose
        v_meas = self.measured_speed()
        v0 = max(v_meas if v_meas is not None else self.start_speed(), 0.4)

        # Pose stamp -> now.  This is ESTIMATION lag only (the scan is stamped,
        # the filter takes ~45 ms): the actuator dead time is not handled here
        # but inside the horizon, as committed steering.
        t_now = min(now, t + self.predict_window())
        x, y, yaw, delta0 = self.act.predict(
            x, y, yaw, t, t_now, v0,
            kappa_of=lambda d: (self.P['yaw_gain'] * math.tan(d)
                                / vehicle.WHEELBASE))

        ra = int(self.P['reindex_after'])
        last = self._idx
        if ra > 0 and self._gate_run >= ra:
            last = None              # search the whole line, not the window
        i0 = self.path.nearest(x, y, last, int(self.P['track_window']),
                               int(self.P['track_back']), self.P['track_lost_m'],
                               yaw=yaw, yaw_tol=self.P['track_yaw_tol'])
        self._idx = i0
        pyaw = float(self.path.yaw[i0])
        dx, dy = x - self.path.xy[i0, 0], y - self.path.xy[i0, 1]
        e_lat = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
        e_yaw = wrap(yaw - pyaw)

        # The pose trail alone cannot tell a wedged car from a frozen filter,
        # and the escape below is a REVERSE setpoint at full lock. Require the
        # wheel encoder -- an independent sensor -- to agree we are stopped.
        if self.is_stuck() and (v_meas is None or v_meas < 0.5):
            self._escape_until = now + self.P['escape_seconds']
            self._stuck_count += 1
            if self.ilc is not None:
                self.ilc.invalidate()
            turn = e_yaw if abs(e_yaw) > 0.1 else e_lat
            self._escape_steer = math.copysign(vehicle.MAX_STEER, turn)
            self.get_logger().warn(
                f'stuck at ({x:.2f},{y:.2f}) e_lat={e_lat:.2f} '
                f'e_yaw={e_yaw:.2f} — reversing out (#{self._stuck_count})')
            return

        # The steering the in-flight commands will produce at each committed
        # step.  Data, not a decision -- see mpc3_core._rollout.
        dt = self.mpc.dt
        committed = [self.act.delta_at(t_now + (k + 1) * dt)
                     for k in range(self.mpc.nd)]

        # A hard cap on top of the model's own saturation.  Never below what the
        # LINE asks for over the preview, or the tracker has negative correction
        # authority and simply understeers into the outside wall (measured on
        # mpc2 at ay 7.0/7.5).
        v_cap = max(v0, v_meas) if (self.P['cap_use_measured'] and v_meas) else v0
        ay_cap = max(float(self.P['ay_limit']),
                     float(self.P['cap_ff_headroom']) * self.plan_ay(i0, v0))
        cap = min(vehicle.MAX_STEER,
                  math.atan(vehicle.WHEELBASE * ay_cap
                            / max(v_cap * v_cap, 0.25)))

        try:
            cmd, info = self.mpc.solve(e_lat, e_yaw, delta0, i0, v0, cap=cap,
                                       committed=committed)
        except ValueError as exc:
            # Non-finite state. Publishing nothing holds the last good command
            # (this simulator latches), which is the right thing for one frame;
            # the stale-pose watchdog covers a persistent fault.
            self._solve_fail += 1
            self.get_logger().error(
                f'solver rejected the state ({exc}); holding the last command '
                f'(#{self._solve_fail})')
            return
        if self.ilc is not None:
            # Lap boundary = the reference index wrapping.  `nearest` is
            # heading-gated and monotone in practice, so this is the same test
            # the offline harness uses.
            if (self._ilc_prev_idx is not None
                    and self._ilc_prev_idx > 0.8 * self.path.n
                    and i0 < 0.2 * self.path.n):
                st = self.ilc.finish_lap()
                if st:
                    self.get_logger().info(
                        f'ILC lap {st["lap"]}: mean |e| per bin '
                        f'{st["mean_abs_e"]:.3f} m (max {st["max_abs_e"]:.3f}), '
                        f'|kff| max {st["kff_max"]:.3f} 1/m (moved '
                        f'{st["d_kff"]:.3f}), v_scale '
                        f'[{st["v_scale_min"]:.3f}, {st["v_scale_max"]:.3f}], '
                        f'worst corridor clearance {st["min_clear"]:.3f} m')
            self._ilc_prev_idx = i0
            self.ilc.observe(float(self.path.s[i0]), e_lat,
                             corridor_clearance(self.path, i0, e_lat))

        # Fallback speed if the encoder ever goes quiet: the PROFILE here, not
        # the last measurement, which would be a stale hold.  Also what
        # `is_stuck` compares against.
        self.v_plan0 = float(self.path.v_prof[i0])

        # Throttle: the speed the PLAN wants when this command is in force, fed
        # to the servo-inverting law.  Never negative.
        head_scale = 1.0
        look_m = self.servo_decel_look_m
        if look_m > 0.0:
            nlook = max(1, int(round(look_m / max(self.path.spacing, 1e-3))))
            jj = (int(i0) + np.arange(nlook + 1)) % self.path.n
            if float(np.min(self.path.a_prof[jj])) < -0.5:
                head_scale = 0.0
        if self.servo_period_max > 0.0:
            head_scale *= min(1.0, max(0.0, (self.servo_period_max
                                              - float(self.rate_ctl.est.period)) / 0.004))
        v_cmd = self.gap_v_cmd(info['v_cmd'], now)
        # OPT-IN residual (None = off).  After the solve, before the publish
        # guard.  It may add at most +-0.03 rad and scale the speed setpoint
        # by 0.95-1.05 (never above 1.0 in a corner or a slow zone); the guard
        # in `publish` still has the last word, and the cap that the tyres can
        # pay for at this speed is re-applied so the residual cannot buy
        # steering the grip budget has already refused.
        if self.residual is not None:
            obs = residual_observation(
                self.path, i0, e_lat, e_yaw,
                v_meas if v_meas is not None else v0,
                float(self.rate_ctl.est.period),
                self.residual.preview_kappa_m, self.residual.preview_wall_m)
            d_steer, v_scale = self.residual.act(
                obs, s=float(self.path.s[i0]),
                kappa=float(self.path.kappa[i0]))
            cmd = float(np.clip(cmd + d_steer, -cap, cap))
            v_cmd = float(v_cmd) * v_scale
        th = float(self.thr.command(v_cmd,
                                    v_meas, 0.0, now=now,
                                    kappa=float(self.path.kappa[i0]),
                                    head_scale=head_scale))
        self.publish(cmd, th)
        self.pub_cte.publish(Float32(data=float(e_lat)))

        self._solve_ms = (time.perf_counter() - t_solve) * 1e3
        self._solves += 1
        # A solve longer than the sensor period means the next pose is already
        # waiting (QoS depth 1, BEST_EFFORT: it is dropped, not queued) and the
        # command in force is a frame older than the tune assumes. Counted, not
        # warned per solve -- reported in the periodic line below.
        if self._solve_ms > 1e3 * float(self.rate_ctl.est.period):
            self._slow_solves += 1
        if self._solves % 200 == 0:          # ~every 18 s at 11 Hz
            self.get_logger().info(
                f'solve {self._solve_ms:.1f} ms, e_lat {e_lat:+.3f} m, '
                f'v {v0:.2f} -> plan {info["v_cmd"]:.2f} '
                f'(profile {float(self.path.v_prof[i0]):.2f}); pose period '
                f'{self.rate_ctl.est.period * 1e3:.1f} ms '
                f'({"adapted" if self.rate_ctl.enabled else "measured only"}: '
                f'apply {self.mpc.cfg["apply_ahead_s"]:.3f} thr '
                f'{self.mpc.cfg["thr_lead_s"]:.3f} nd {self.mpc.nd}'
                # The clamp is otherwise SILENT: on a host outside
                # [rate_lo, rate_hi] x rate_nominal the constants in force are
                # the ones for the clamp EDGE, not the measured period, and
                # nothing in the log said so.  Count, not a warning per pose.
                f'{f", clamped x{self.rate_ctl.clamped}" if self.rate_ctl.clamped else ""}'
                f'{f", solve over period x{self._slow_solves}" if self._slow_solves else ""})')

    # ------------------------------------------------------------------
    def publish(self, steer, throttle):
        # A non-finite command must never reach the actuator, and must never be
        # LATCHED into `self.steer` -- the watchdog republishes that value at
        # 20 Hz, so one NaN would hold the wheel there for the rest of the
        # race. `np.clip` passes a NaN straight through and `steer_to_cmd` used
        # to turn it into full lock (see vehicle.steer_to_cmd). Hold the last
        # good angle and cut the throttle instead; a finite pair takes the
        # identical path, so nothing on the normal path moves.
        st, th = float(steer), float(throttle)
        if not (math.isfinite(st) and math.isfinite(th)):
            self._nonfinite_cmd += 1
            if self._nonfinite_cmd in (1, 10) or self._nonfinite_cmd % 100 == 0:
                self.get_logger().error(
                    f'non-finite command (steer={steer!r} throttle={throttle!r}); '
                    f'holding {self.steer:+.3f} rad, zero throttle '
                    f'(#{self._nonfinite_cmd})')
            st, th = self.steer, 0.0
        self.steer = float(np.clip(st, -vehicle.MAX_STEER, vehicle.MAX_STEER))
        self.pub_steer.publish(Float32(data=vehicle.steer_to_cmd(self.steer)))
        t_rec = self.now()
        if (self.control_mode == 'timer' and self.P['timer_sync_dead']
                and self.pose is not None):
            t_rec = self.pickup_time(t_rec, self.pose[3])
        self.act.record(t_rec, self.steer)
        self.pub_throttle.publish(Float32(data=th))


def main():
    rclpy.init()
    node = MPC3Tracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
