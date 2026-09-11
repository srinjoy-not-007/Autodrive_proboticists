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
from pit_crew.mpc3_core import MPC3, Actuator, PoseFilter, RefPath, wrap
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
            ('a_accel', 4.0),
            ('e_free', 0.18),
            ('k_e', 2.0),
            ('v_mod_min', 0.55),
            ('thr_lead_s', 0.225),
            # --- actuator / delay --------------------------------------------
            ('steer_rate_limit', 3.2),
            ('cmd_dead_time', 0.135),     # measured, structural (devkit bridge)
            ('delay_s', 0.180),           # dead time + estimation lag
            ('apply_ahead_s', 0.090),
            ('use_steer_feedback', True),
            ('steer_fb_age', 0.0),
            # --- steering authority ------------------------------------------
            ('ay_limit', 9.0),            # hard cap; the saturation model does
                                          # most of this work now
            ('cap_ff_headroom', 1.15),    # ...but never below the LINE's own
            ('cap_preview_min_m', 1.0),   #    feedforward over the preview
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
            # --- throttle (ServoThrottle) -------------------------------------
            ('servo_gain', 24.6),
            ('servo_k', 18.0),
            ('servo_drag', 0.273),
            ('servo_a_auth', 4.5),
            ('servo_a_accel', 5.0),
            ('servo_k_p', 0.0),
            ('servo_head_max', 0.5),
            ('servo_u_max', 0.60),
            ('servo_v_floor', 1.0),
            ('v_meas_min', 0.2),          # below this the encoder says nothing
            ('v_start', 0.5),             # plan speed before the encoder ever
                                          # reports -- see start_speed()
            # --- safety --------------------------------------------------------
            ('stuck_seconds', 1.5),
            ('stuck_distance', 0.20),
            ('escape_seconds', 1.2),
            ('watchdog_hz', 20.0),
        ])
        v = {n.name: n.value for n in p}
        self.P = v

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
            thr_lead_s=v['thr_lead_s'], delay_s=v['delay_s'],
            apply_ahead_s=v['apply_ahead_s'],
            steer_rate=v['steer_rate_limit'])
        self.act = Actuator(dead_time=float(v['cmd_dead_time']),
                            rate=float(v['steer_rate_limit']))
        self.filt = PoseFilter(self.act, blend=float(v['pose_blend']),
                               gate_m=float(v['blend_gate_m']))
        self.thr = ServoThrottle(
            K=v['servo_k'], drag=v['servo_drag'], tau=v['cmd_dead_time'],
            a_auth=v['servo_a_auth'], a_accel=v['servo_a_accel'],
            k_p=v['servo_k_p'], head_max=v['servo_head_max'],
            lead_max=0.0, ay_limit=v['ay_budget'], v_floor=v['servo_v_floor'],
            u_max=v['servo_u_max'], servo_gain=v['servo_gain'])
        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)

        self.pose = None
        self.gyro_z = 0.0
        self.v_plan0 = 0.0           # profile speed at the car (encoder fallback)
        self._enc_seen = False       # has the encoder ever reported motion?
        self.steer = 0.0
        self.steer_fb = None
        self._idx = None
        self._trail = []
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

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(PoseStamped, '/pit_crew/pose', self.on_pose, 1)
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.create_subscription(Float32, f'{ns}/steering', self.on_steer_fb, qos)
        for side in ('left', 'right'):
            self.create_subscription(
                JointState, f'{ns}/{side}_encoder',
                lambda m, w=side: self.on_enc(w, m), qos)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.pub_cte = self.create_publisher(Float32, '/pit_crew/cte', 10)
        self.create_timer(1.0 / max(float(v['watchdog_hz']), 1.0), self.watchdog)
        self.get_logger().info(
            f"mpc3 ready: {self.path.n} pts, horizon {v['horizon']}x"
            f"{v['step_dt']:.3f}s ({self.mpc.nd} committed), delay "
            f"{v['delay_s']:.3f}s, ay_budget {v['ay_budget']:.1f}, barrier "
            f"{'on' if self.path.has_corridor else 'off'}")

    # ------------------------------------------------------------------
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
            return None
        v = self.speed_est.speed
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
        if stamp <= 0.0 or stamp > now + 0.1 or now - stamp > self.P['stale_pose_s']:
            return self.drop_pose(f'stale by {now - stamp:.2f}s', now)
        if self.pose is not None and stamp <= self.pose[3]:
            return self.drop_pose('out of order', now)
        self._pose_ok += 1
        if self.v_plan0 <= 0.0:
            j = self.path.nearest(mx, my, None, yaw=myaw)
            self.v_plan0 = float(self.path.v_prof[j])
        v = self.measured_speed() or self.start_speed()
        x, y, yaw = self.filt.update(stamp, mx, my, myaw, max(v, 0.4))[:3]
        self.pose = (x, y, yaw, stamp)
        self._trail.append((stamp, mx, my))
        while self._trail and stamp - self._trail[0][0] > self.P['stuck_seconds']:
            self._trail.pop(0)
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

    def watchdog(self):
        """Runs whether or not poses arrive; commands LATCH in this simulator."""
        if self.pose is None:
            return
        if self.now() - self.pose[3] > self.P['stale_pose_s']:
            # Hold the steering angle (mid-corner, zero steer points at the
            # outside wall) and coast.  Zero throttle is a ~5.8 m/s2 brake here,
            # which is exactly what is wanted when the pose has gone.
            self.publish(self.steer, 0.0)
            if not self._stale_warned:
                self.get_logger().error(
                    f"no pose for {self.P['stale_pose_s']:.2f}s — coasting to a stop")
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
        t_now = min(now, t + float(self.P['max_predict_s']))
        x, y, yaw, delta0 = self.act.predict(
            x, y, yaw, t, t_now, v0,
            kappa_of=lambda d: (self.P['yaw_gain'] * math.tan(d)
                                / vehicle.WHEELBASE))

        i0 = self.path.nearest(x, y, self._idx, int(self.P['track_window']),
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
        # Fallback speed if the encoder ever goes quiet: the PROFILE here, not
        # the last measurement, which would be a stale hold.  Also what
        # `is_stuck` compares against.
        self.v_plan0 = float(self.path.v_prof[i0])

        # Throttle: the speed the PLAN wants when this command is in force, fed
        # to the servo-inverting law.  Never negative.
        th = float(self.thr.command(info['v_cmd'], v_meas, 0.0, now=now,
                                    kappa=float(self.path.kappa[i0])))
        self.publish(cmd, th)
        self.pub_cte.publish(Float32(data=float(e_lat)))

        self._solve_ms = (time.perf_counter() - t_solve) * 1e3
        self._solves += 1
        if self._solves % 200 == 0:          # ~every 18 s at 11 Hz
            self.get_logger().info(
                f'solve {self._solve_ms:.1f} ms, e_lat {e_lat:+.3f} m, '
                f'v {v0:.2f} -> plan {info["v_cmd"]:.2f} '
                f'(profile {float(self.path.v_prof[i0]):.2f})')

    # ------------------------------------------------------------------
    def publish(self, steer, throttle):
        self.steer = float(np.clip(steer, -vehicle.MAX_STEER, vehicle.MAX_STEER))
        self.pub_steer.publish(Float32(data=vehicle.steer_to_cmd(self.steer)))
        self.act.record(self.now(), self.steer)
        self.pub_throttle.publish(Float32(data=float(throttle)))


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
