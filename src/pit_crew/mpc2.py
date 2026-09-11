"""mpc2 — path tracker with a real finite-horizon solve and a measured actuator.

Launch exactly like the shipping tracker:

    CONTROLLER=mpc2 ./scripts/run_exp.sh <name> <line> [-p key:=value ...]
    CONTROLLER=mpc2 ./scripts/run_stanley.sh 1.0 track_v2 safe_ay6.0

It takes the same ``raceline_csv`` / ``speed_scale`` parameters the runner
already passes, publishes the same ``steering_command`` / ``throttle_command``
/ ``/pit_crew/cte`` topics, and keeps mpc.py's throttle, brake horizon, stale
pose watchdog and stuck/escape behaviour unchanged.

Three things differ from ``mpc``, all of them measured this session:

1. **The actuator model is anchored on the actuator.**
   ``/autodrive/roboracer_1/steering`` is the real steering angle in radians,
   it is a legal runtime input, and nothing used it.  mpc assumed the last
   *published command* was the current angle and integrated the dead time with
   that; when the steering is slewing (a hairpin, every time) the two differ by
   up to 0.29 rad — a whole frame of travel at 3.2 rad/s.  ``Actuator`` snaps to
   the feedback and replays the in-flight commands through the 0.135 s dead
   time and the 3.2 rad/s slew to work out what the steering will really be
   when the command chosen now bites.

2. **A finite-horizon optimisation replaces the candidate grid.**
   mpc searched 9 first steps and then ran a fixed proportional relaxation
   towards the curvature feedforward for the remaining 11.  Only the first step
   was a decision; the tail was a controller, and that tail always aims at the
   reference by the shortest route.  Through a right-hand hairpin the shortest
   route is the inside of it, which is where the measured 0.15–0.23 m cut and
   every s 13.6 collision come from.  mpc2 optimises the whole steering-RATE
   sequence by Gauss-Newton, with the 3.2 rad/s slew as the box on the decision
   variable, warm-started from the previous solution (~3 ms in numpy).

3. **Cutting is paid for, and the walls are visible.**
   e_lat is charged at every step, again at the terminal, with extra weight
   when the error is on the *inside* of the bend, plus a hinge barrier on the
   corridor read from the same distance field localization already loads.

Because the solve is one shot per pose and the sensors arrive at 11.1 Hz, it
runs on the pose callback rather than on a 50 Hz timer — solving five times per
sensor frame only re-answers the same question with the same data.  A slow
timer still runs, to keep the stale-pose watchdog and the escape manoeuvre
alive when poses stop.
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
from pit_crew.mpc2_core import (Actuator, CurvatureBias, MPC2, PoseFilter,
                                RefPath, wrap)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def derive_map_prefix(raceline_csv, maps_dir=None):
    """Guess the map that goes with a raceline path.

    ``run_stanley.sh`` passes the controller a raceline and nothing else, so
    the corridor barrier would be off by default on every existing runner.
    Strip ``_token`` suffixes off the raceline stem until a map of that name
    exists: ``track_v2_safe_ay6.0`` -> ``track_v2``, ``track_v2_c_safe`` ->
    ``track_v2_c``.  Returns '' when nothing matches, which disables the
    barrier rather than guessing wrong.
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


class MPC2Tracker(Node):

    def __init__(self):
        super().__init__('mpc2')
        p = self.declare_parameters('', [
            ('raceline_csv', '/home/data/racelines/track_v2_safe.csv'),
            ('speed_scale', 1.0),
            ('map_prefix', ''),           # '' = derive from raceline, see above
            ('use_barrier', True),
            ('barrier_margin', 0.20),     # extra clearance the barrier defends
            ('w_barrier', 400.0),
            ('horizon', 24),
            ('step_dt', 0.055),           # 0.88 s of horizon
            ('min_step_m', 0.10),
            ('iters', 8),
            ('w_lat', 14.0),
            ('w_yaw', 3.0),
            ('w_steer', 0.4),
            ('w_rate', 0.25),
            ('w_jerk', 0.06),
            ('w_cut', 2.0),               # extra w_lat inside of the target
            # Aim this far inside the reference in a bend, corridor-clipped.
            # mpc.py cuts 0.139 m and is 0.46 s/lap faster for it; this is the
            # same lap time bought deliberately, with the barrier still
            # holding the wall. 0.0 = track the line exactly.
            ('cut_offset_m', 0.0),
            ('cut_kappa_ref', 0.6),
            ('w_lat_T', 40.0),
            ('w_yaw_T', 8.0),
            ('apply_ahead_s', 0.09),      # which plan point to hand the actuator
            ('steer_rate_limit', 3.2),
            ('cmd_dead_time', 0.135),     # measured, structural (devkit bridge)
            ('extra_latency_s', 0.045),   # estimation lag on top of the dead time
            ('use_steer_feedback', True),
            # How old the value on /steering is when it reaches us. The bridge
            # publishes it with the rest of the frame, so ~0; raise it if a
            # latency probe ever shows the bridge returning the PREVIOUS
            # frame's actuator state. It is not the same thing as the pose
            # being old -- that is handled by stamping, and the Actuator now
            # interpolates its recorded samples instead of assuming the newest
            # one describes the pose's moment.
            ('steer_fb_age', 0.0),
            ('kappa_bias_gain', 0.8),    # offset-free correction; 0 disables
            ('kappa_bias_max', 0.10),     # 1/m, clamp on that correction
            ('pose_blend', 0.7),          # 1.0 = take every localization sample whole
            ('blend_gate_m', 0.5),        # innovation above this is a correction
            # The cap that mpc.py never reaches, mpc2 does: measured on the
            # first live run, 4% of samples sat within 5% of it and the
            # achieved/asked curvature ratio at the hairpins was 0.74 (against
            # 0.93 for mpc in the same session) -- the car was sliding on
            # authority the cap allowed and the tyres do not have.  Set it to
            # the grip that exists, not to 1.3x the profile.
            ('ay_limit', 6.0),            # steering cap from the grip that exists
            ('cap_use_measured', True),   # cap at the speed the car HAS
            ('kappa_span_m', 0.5),
            ('track_window', 90),
            ('track_back', 3),
            ('track_yaw_tol', 1.2),       # rad; a reference pointing further
                                          # off than this is the other leg of
                                          # a hairpin, not where the car is
            ('track_lost_m', 1.0),
            ('use_measured_speed', False),
            ('max_predict_s', 0.35),
            ('stale_pose_s', 0.35),
            ('stuck_seconds', 1.5),
            ('stuck_distance', 0.20),
            ('escape_seconds', 1.2),
            ('brake_horizon_s', 0.35),
            ('throttle_gain', 0.040),
            ('brake_gain', 0.05),
            ('throttle_kp', 0.0),
            ('throttle_a_gain', 0.0),
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
                dist, img, meta = pf_core.load_map(prefix)
                self.path.load_corridor(dist, meta)
                self.get_logger().info(
                    f'corridor from {prefix}: min half-width '
                    f'{min(self.path.w_left.min(), self.path.w_right.min()):.2f} m')
            except Exception as exc:                      # noqa: BLE001
                self.get_logger().warn(f'no corridor barrier ({exc})')
        elif v['use_barrier']:
            self.get_logger().warn('no map found for the raceline; barrier off')

        self.mpc = MPC2(
            self.path, horizon=int(v['horizon']), step_dt=v['step_dt'],
            min_step_m=v['min_step_m'], iters=int(v['iters']),
            w_lat=v['w_lat'], w_yaw=v['w_yaw'], w_steer=v['w_steer'],
            w_rate=v['w_rate'], w_jerk=v['w_jerk'], w_cut=v['w_cut'],
            w_lat_T=v['w_lat_T'], w_yaw_T=v['w_yaw_T'],
            cut_offset_m=v['cut_offset_m'], cut_kappa_ref=v['cut_kappa_ref'],
            w_barrier=v['w_barrier'], barrier_margin=v['barrier_margin'],
            steer_rate=v['steer_rate_limit'], apply_ahead_s=v['apply_ahead_s'])
        self.act = Actuator(dead_time=float(v['cmd_dead_time']),
                            rate=float(v['steer_rate_limit']))
        self.filt = PoseFilter(self.act, blend=float(v['pose_blend']),
                               gate_m=float(v['blend_gate_m']))
        self.bias = CurvatureBias(gain=float(v['kappa_bias_gain']),
                                  limit=float(v['kappa_bias_max']))

        self.throttle_ctl = vehicle.ThrottleController(
            ff_gain=v['throttle_gain'], brake_gain=v['brake_gain'],
            kp=v['throttle_kp'], a_gain=v['throttle_a_gain'])
        self.brake_gain = v['brake_gain']
        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)
        self.pose = None
        self.gyro_z = 0.0
        self.speed = 0.0
        self.accel = 0.0
        self.steer = 0.0                 # last published command (rad)
        self.steer_fb = None             # (t, rad) from the actuator feedback
        self._idx = None
        self._trail = []
        self._escape_until = None
        self._escape_steer = 0.0
        self._stuck_count = 0
        self._stale_warned = False
        self._solve_ms = 0.0
        self._solves = 0

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
            f"mpc2 ready: {self.path.n} pts, horizon {v['horizon']}x"
            f"{v['step_dt']:.3f}s, dead time {v['cmd_dead_time']:.3f}+"
            f"{v['extra_latency_s']:.3f}s, barrier "
            f"{'on' if self.path.has_corridor else 'off'}")

    # ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg: Imu):
        if math.isfinite(msg.angular_velocity.z):
            self.gyro_z = float(msg.angular_velocity.z)
            if self.steer_fb is not None:
                self.bias.update(self.gyro_z, self.speed_est.speed
                                 if self.speed_est.speed > 0.1 else self.speed,
                                 self.steer_fb[1])

    def on_enc(self, wheel, msg: JointState):
        if msg.position:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(wheel, t, msg.position[0])

    def on_steer_fb(self, msg: Float32):
        """The real steering angle, in radians (verified against the logs:
        it peaks at the ay_limit cap, not at the normalized command)."""
        if not math.isfinite(msg.data):
            return
        t = self.now()
        self.steer_fb = (t, float(msg.data))
        if self.P['use_steer_feedback']:
            self.act.snap(t - float(self.P['steer_fb_age']), float(msg.data))

    def measured_speed(self):
        if not self.P['use_measured_speed']:
            return self.speed
        if self.speed_est.consume_jump():
            return self.speed
        v = self.speed_est.speed
        return v if v > 1e-3 else self.speed

    # ------------------------------------------------------------------
    def on_pose(self, msg: PoseStamped):
        mx, my = msg.pose.position.x, msg.pose.position.y
        myaw = yaw_from_quat(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.now()
        if not all(math.isfinite(q) for q in (mx, my, myaw, stamp)):
            return
        if stamp <= 0.0 or stamp > now + 0.1 or now - stamp > self.P['stale_pose_s']:
            return
        if self.pose is not None and stamp <= self.pose[3]:
            return
        if self.speed <= 0.0:
            # Seed the speed target before the first prediction: the pose
            # filter and the delay compensation both propagate at `speed`, and
            # starting them at zero makes the first few frames lag by metres.
            j = self.path.nearest(mx, my, None, yaw=myaw)
            self.speed = float(self.path.v_prof[j])
        x, y, yaw = self.filt.update(stamp, mx, my, myaw,
                                     max(self.measured_speed(), 0.4))[:3]
        self.pose = (x, y, yaw, stamp)
        self._trail.append((stamp, mx, my))
        while self._trail and stamp - self._trail[0][0] > self.P['stuck_seconds']:
            self._trail.pop(0)
        self.control_step()

    # ------------------------------------------------------------------
    def is_stuck(self):
        if len(self._trail) < 5 or self.speed < 0.3:
            return False
        t0, x0, y0 = self._trail[0]
        if self._trail[-1][0] - t0 < self.P['stuck_seconds'] * 0.9:
            return False
        _, x1, y1 = self._trail[-1]
        return math.hypot(x1 - x0, y1 - y0) < self.P['stuck_distance']

    def watchdog(self):
        """Runs whether or not poses arrive; commands latch in this simulator."""
        if self.pose is None:
            return
        if self.now() - self.pose[3] > self.P['stale_pose_s']:
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

    # ------------------------------------------------------------------
    def control_step(self):
        t_solve = time.perf_counter()
        now = self.now()
        if self._escape_until is not None:
            if now < self._escape_until:
                self.publish(self._escape_steer, -0.25)
                return
            self._escape_until = None
            self._trail.clear()

        x, y, yaw, t = self.pose
        v = max(self.measured_speed(), 0.4)

        # Where the car will be, and what the steering will actually be, when
        # the command chosen now takes effect: pose stamp -> now + dead time.
        lag = float(self.P['cmd_dead_time']) + float(self.P['extra_latency_s'])
        t_bite = min(now, t + float(self.P['max_predict_s'])) + lag
        kb = self.bias.value
        x, y, yaw, delta0 = self.act.predict(
            x, y, yaw, t, t_bite, v,
            kappa_of=lambda d: math.tan(d) / vehicle.WHEELBASE + kb)

        i0 = self.path.nearest(x, y, self._idx, int(self.P['track_window']),
                               int(self.P['track_back']), self.P['track_lost_m'],
                               yaw=yaw, yaw_tol=self.P['track_yaw_tol'])
        self._idx = i0
        pyaw = float(self.path.yaw[i0])
        dx, dy = x - self.path.xy[i0, 0], y - self.path.xy[i0, 1]
        e_lat = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
        e_yaw = wrap(yaw - pyaw)

        if self.is_stuck():
            self._escape_until = now + self.P['escape_seconds']
            self._stuck_count += 1
            turn = e_yaw if abs(e_yaw) > 0.1 else e_lat
            self._escape_steer = math.copysign(vehicle.MAX_STEER, turn)
            self.get_logger().warn(
                f'stuck at ({x:.2f},{y:.2f}) e_lat={e_lat:.2f} '
                f'e_yaw={e_yaw:.2f} — reversing out (#{self._stuck_count})')
            return

        # Steering authority is not a constant: delta = atan(L*ay/v^2) is the
        # largest angle the tyres can pay for at this speed. Unchanged from mpc.
        # ...and at the speed the car HAS, not the one it was told to have.
        # `v` above is the profile target when use_measured_speed is off; when
        # the car is running over plan (mpc2 scrubs less than mpc, so it does)
        # a cap computed at the target permits more lateral demand than exists.
        v_cap = v
        if self.P['cap_use_measured'] and self.speed_est.speed > 0.1:
            v_cap = max(v, self.speed_est.speed)
        cap = vehicle.MAX_STEER
        if self.P['ay_limit'] > 0.0:
            cap = min(cap, math.atan(vehicle.WHEELBASE * self.P['ay_limit']
                                     / max(v_cap * v_cap, 0.25)))

        cmd, _info = self.mpc.solve(e_lat, e_yaw, delta0, i0, v, cap=cap,
                                    bias=self.bias.value)
        self._solve_ms = (time.perf_counter() - t_solve) * 1e3
        self._solves += 1
        if self._solves % 200 == 0:          # ~every 18 s at 11 Hz
            self.get_logger().info(
                f'solve {self._solve_ms:.1f} ms, e_lat {e_lat:+.3f} m, '
                f'kappa bias {self.bias.value:+.3f} 1/m')

        # Longitudinal: the minimum profile speed over the braking horizon,
        # with the planned acceleration fed forward only when the horizon is
        # not already holding the car back (see mpc.py for why).
        horizon = max(0.2, self.speed * self.P['brake_horizon_s'])
        n_ahead = max(2, int(horizon / max(self.path.spacing, 1e-3)))
        idx = (i0 + np.arange(n_ahead)) % self.path.n
        self.speed = float(self.path.v_prof[idx].min())
        here = float(self.path.v_prof[i0])
        self.accel = (float(self.path.a_prof[i0])
                      if self.speed >= here - 1e-3 else 0.0)

        v_fb = (self.speed_est.speed
                if self.brake_gain > 0.0 or self.throttle_ctl.kp > 0.0 else None)
        self.publish(cmd, float(self.throttle_ctl.command(
            self.speed, v_fb, self.accel)))
        self.pub_cte.publish(Float32(data=float(e_lat)))

    # ------------------------------------------------------------------
    def publish(self, steer, throttle):
        self.steer = float(np.clip(steer, -vehicle.MAX_STEER, vehicle.MAX_STEER))
        self.pub_steer.publish(Float32(data=vehicle.steer_to_cmd(self.steer)))
        self.act.record(self.now(), self.steer)
        self.pub_throttle.publish(Float32(data=float(throttle)))


def main():
    rclpy.init()
    node = MPC2Tracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
