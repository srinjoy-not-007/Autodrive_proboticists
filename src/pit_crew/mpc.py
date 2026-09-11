"""Kinematic MPC over the raceline, respecting the steering rate limit.

Why MPC here, specifically: the measured steering oscillation (1.68 reversals/s,
p95 steering rate 4.32 rad/s against a 3.2 rad/s actuator) is the signature of a
controller commanding faster than the wheels can move. Stanley can only be
calmed by lowering feedback gains, which costs tracking accuracy — measured at
12.06 s / 2 collisions versus 11.21 s / 0. MPC avoids that trade by optimising a
whole steering *sequence* subject to the rate limit, so it never asks for a
change the actuator cannot deliver.

Model: kinematic bicycle in the path-relative frame, which keeps the state small
(lateral offset, heading error) and the problem convex enough to solve at 50 Hz
with a plain sequential least-squares step.

  e_lat'  = v * sin(e_yaw)
  e_yaw'  = v * (tan(delta)/L - kappa_ref * cos(e_yaw) / (1 - kappa_ref*e_lat))

Constraints: |delta| <= MAX_STEER, |d(delta)/dt| <= steer_rate_limit.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy import ndimage
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32

from pit_crew import vehicle


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MPCTracker(Node):

    def __init__(self):
        super().__init__('mpc')
        p = self.declare_parameters('', [
            ('raceline_csv', '/home/data/racelines/track_v2_safe.csv'),
            ('speed_scale', 1.0),
            ('horizon', 12),             # steps
            ('step_dt', 0.06),           # s per step (~0.7 s horizon)
            ('min_step_m', 0.10),        # ...but never preview less than this
            ('stuck_seconds', 1.5),      # commanded to move but barely moving
            ('stuck_distance', 0.20),    # m of travel that counts as moving
            ('escape_seconds', 1.2),     # how long to reverse out
            ('stale_pose_s', 0.35),      # no pose for this long = coast to a stop
            ('understeer_model', False),  # roll out with measured tyre saturation
            ('track_window', 90),        # path points searched ahead of the last
            ('track_back', 12),          # ...and behind, allowing a small slip back
            ('track_lost_m', 1.0),       # nothing this close nearby = re-acquire
            ('reach_steps', 4.0),        # candidate spread, in rate-limit steps
            ('use_measured_speed', False),   # encoder speed for prediction
            ('rate_limited_rollout', False), # actuator limit inside the horizon
            ('windowed_index', False),   # local path search instead of global
            ('ay_limit', 4.5),           # m/s2 the tyres deliver; 0 disables
            ('grip_watchdog', False),    # shed speed when the tyres let go
            ('grip_ratio_min', 0.60),    # achieved/asked curvature below this = sliding
            ('grip_cut', 0.75),          # speed multiplier applied per bad sample
            ('grip_recover', 0.04),      # ...and clawed back per good one
            ('grip_floor', 0.45),        # never shed more speed than this
            ('w_lat', 12.0),             # weight on lateral offset
            ('w_yaw', 3.0),              # weight on heading error
            ('w_steer', 0.5),            # weight on steering magnitude
            ('w_rate', 4.0),             # weight on steering change
            ('n_candidates', 9),         # first-step options searched per cycle
            ('control_hz', 50.0),
            ('steer_rate_limit', 3.2),   # rad/s, the actuator's real limit
            ('pose_blend', 0.55),
            ('cmd_delay_s', 0.18),       # actuator dead time to predict through; 0 = off
            ('delay_model', 'bicycle'),  # 'gyro': measured yaw rate + change in commanded curvature
            ('blend_gate_m', 0.0),       # innovation above this takes the measurement whole; 0 = off
            ('rollout_profile_speed', False),  # advance v along the horizon from the profile
            ('kappa_span_m', 0.5),       # half-width of the kappa/yaw reference smoothing
            ('pose_qos_depth', 10),      # 1 = never replay a backlog of stale poses
            ('max_predict_s', 0.35),
            ('brake_horizon_s', 0.35),
            ('throttle_gain', 0.040),
            ('brake_gain', 0.05),        # real braking; 0 disables (see below)
            ('throttle_kp', 0.0),        # close the speed loop; 0 = open loop
            ('throttle_a_gain', 0.0),    # throttle per m/s2 of planned accel
        ])
        (raceline_csv, self.speed_scale, self.N, self.dt, self.min_step_m,
         self.stuck_seconds, self.stuck_distance, self.escape_seconds,
         self.stale_pose_s, self.understeer_model, self.track_window,
         self.track_back, self.track_lost_m, self.reach_steps,
         self.use_measured_speed, self.rate_limited_rollout,
         self.windowed_index, self.ay_limit, self.grip_watchdog,
         self.grip_ratio_min, self.grip_cut, self.grip_recover,
         self.grip_floor,
         self.w_lat, self.w_yaw, self.w_steer, self.w_rate, self.n_cand,
         control_hz, self.steer_rate, self.pose_blend, self.cmd_delay,
         self.delay_model, self.blend_gate, self.rollout_profile_speed, kappa_span_m,
         pose_qos_depth, self.max_predict,
         self.brake_horizon, throttle_gain, brake_gain,
         throttle_kp, throttle_a_gain) = [x.value for x in p]
        self.control_dt = 1.0 / control_hz

        rl = np.loadtxt(raceline_csv, delimiter=',')
        if (rl.ndim != 2 or rl.shape[0] < 3 or rl.shape[1] < 6
                or not np.isfinite(rl).all() or np.any(rl[:, 5] < 0)):
            raise ValueError('raceline needs >=3 finite rows and nonnegative speeds')
        self.xy = rl[:, 1:3]
        self.v_prof = rl[:, 5]
        self.a_prof = rl[:, 6] if rl.shape[1] > 6 else np.zeros(len(rl))
        seg = np.hypot(np.diff(self.xy[:, 0]), np.diff(self.xy[:, 1]))
        self.spacing = float(np.median(seg)) if len(seg) else 0.1
        span = max(1, int(round(kappa_span_m / max(self.spacing, 1e-3))))
        fwd = np.roll(self.xy, -span, axis=0) - np.roll(self.xy, span, axis=0)
        self.path_yaw = np.arctan2(fwd[:, 1], fwd[:, 0])
        self.kappa = ndimage.uniform_filter1d(rl[:, 4], size=2 * span + 1,
                                              mode='wrap')

        # Braking is OFF by default. With brake_gain 0 the command is
        # ff*v_target + idle, strictly positive -- the car sheds speed by
        # coasting, while the offline profile assumes 4.0 m/s2 of braking.
        # Enabling it feeds the ENCODER speed back (validated, scale
        # 0.993/0.999) and lets the controller command negative throttle
        # when it is above the profile by more than the deadband.
        #
        # `throttle_kp` closes the loop the other way. Measured on the Aug 29
        # brake-7.0 race trace: the car sat BELOW its own plan on 62% of
        # samples -- by 0.96 m/s wherever the profile asked for ax > 3, and by
        # 1.2 m/s wherever it asked for more than 5 m/s. Feedforward alone
        # encodes the target SPEED and knows nothing about the acceleration
        # needed to reach it, so on every corner exit the car cruises out at
        # the steady-state throttle for a speed it has not got yet. Default 0
        # keeps the loop open unless a caller asks.
        self.brake_gain = brake_gain
        self.throttle_ctl = vehicle.ThrottleController(
            ff_gain=throttle_gain, brake_gain=brake_gain, kp=throttle_kp,
            a_gain=throttle_a_gain)
        self.pose = None
        self.gyro_z = 0.0
        self.speed = 0.0            # commanded profile speed (throttle target)
        self.accel = 0.0            # planned longitudinal accel at the car
        self.speed_est = vehicle.SpeedEstimator(alpha=0.35)
        self.steer = 0.0
        self._trail = []            # (t, x, y) of the localized pose
        self._cmd_hist = []         # (t, steer) as published, for delay prediction
        self._escape_until = None
        self._escape_steer = 0.0
        self._stuck_count = 0
        self._stale_warned = False
        self._idx = None
        self.grip_scale = 1.0
        self._grip_bad = 0
        self._grip_events = 0

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(PoseStamped, '/pit_crew/pose', self.on_pose,
                                 int(pose_qos_depth))
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        for side in ('left', 'right'):
            self.create_subscription(
                JointState, f'{ns}/{side}_encoder',
                lambda m, w=side: self.on_enc(w, m), qos)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.pub_cte = self.create_publisher(Float32, '/pit_crew/cte', 10)
        self.create_timer(self.control_dt, self.control_step)
        self.get_logger().info(
            f'mpc ready: {len(self.xy)} pts, horizon {self.N}x{self.dt:.2f}s, '
            f'{self.n_cand} candidates, {control_hz:.0f} Hz')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg: Imu):
        if not math.isfinite(msg.angular_velocity.z):
            return
        self.gyro_z = float(msg.angular_velocity.z)
        self.update_grip()

    def update_grip(self):
        """Catch the tyres letting go, and back off before the wall arrives.

        `ay_limit` is open loop: it caps what the controller may *demand*
        against a constant. This is the closed-loop half — what the car is
        actually delivering. Measured on the open floor, the front tyres hold
        ~0.9 of the curvature asked of them right up to 9.9 m/s2 and then
        collapse to 0.24-0.35 within one 0.5 m/s step. Abrupt, but not
        invisible: the IMU sees the yaw rate stop following the steering.

        Evaluated per IMU sample rather than per control cycle, because the IMU
        is the only new information here and it arrives at ~10 Hz. Two
        consecutive bad samples is ~0.2 s — late, but a slide takes longer than
        that to reach a wall, and it is the earliest this platform can know.
        """
        if not self.grip_watchdog:
            return
        v = self.speed_est.speed
        asked = abs(math.tan(self.steer)) / vehicle.WHEELBASE
        # Only meaningful when actually cornering at speed: at low speed or near
        # zero steer the ratio is noise divided by noise.
        if v < 1.5 or asked < 0.25:
            self._grip_bad = 0
            self.grip_scale = min(1.0, self.grip_scale + 5.0 * self.grip_recover)
            return
        achieved = abs(self.gyro_z) / max(v, 0.1)
        ratio = achieved / asked
        if ratio < self.grip_ratio_min:
            self._grip_bad += 1
            if self._grip_bad >= 2:
                if self.grip_scale > 0.99:
                    self._grip_events += 1
                self.grip_scale = max(self.grip_floor,
                                      self.grip_scale * self.grip_cut)
        else:
            self._grip_bad = 0
            self.grip_scale = min(1.0, self.grip_scale + self.grip_recover)

    def on_enc(self, wheel, msg: JointState):
        if msg.position:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(wheel, t, msg.position[0])

    def measured_speed(self):
        """What the car is doing, not what it was told to do.

        Dead-reckoning and the rollout were both using the *commanded* profile
        speed. Measured against the profile on a clean run that is 0.38 m/s of
        mean error and 0.97 m/s at p95 — enough to shift where the controller
        thinks it is by the time the next scan lands, and enough to move the
        braking point. The encoders are validated (scale 0.993/0.999) and were
        already feeding localization; there is no reason for the controller to
        ignore them.
        """
        if not self.use_measured_speed:
            return self.speed
        if self.speed_est.consume_jump():
            return self.speed        # teleported: measurement says nothing yet
        v = self.speed_est.speed
        return v if v > 1e-3 else self.speed

    def on_pose(self, msg: PoseStamped):
        mx, my = msg.pose.position.x, msg.pose.position.y
        myaw = yaw_from_quat(msg.pose.orientation)
        # Use the pose's own stamp (localization stamps it with the scan time),
        # so prediction covers scan-to-now latency and not just the time since
        # this callback happened to fire. Fall back if the stamp looks unusable.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.now()
        # Relabeling an old pose as fresh defeats the stale-pose watchdog.
        if not all(math.isfinite(v) for v in (mx, my, myaw, stamp)):
            return
        if stamp <= 0.0 or stamp > now + 0.1 or now - stamp > self.stale_pose_s:
            return
        t = stamp
        if self.pose is not None and t <= self.pose[3]:
            return
        if self.pose is None:
            self.pose = (mx, my, myaw, t)
            return
        px, py, pyaw = self.predicted_state()
        a = self.pose_blend
        if self.blend_gate > 0.0 and math.hypot(mx - px, my - py) > self.blend_gate:
            a = 1.0          # a big innovation is a correction, not jitter
        self.pose = (px + a * (mx - px), py + a * (my - py),
                     pyaw + a * wrap(myaw - pyaw), t)
        # Measured travel, for stuck detection. `self.speed` is the *commanded*
        # profile speed, so it keeps reading 1.5 m/s while the car sits wedged
        # against a wall — it can never reveal that the car is not moving.
        self._trail.append((t, mx, my))
        while self._trail and t - self._trail[0][0] > self.stuck_seconds:
            self._trail.pop(0)

    def is_stuck(self):
        """Commanded to drive, but the pose has hardly moved."""
        if len(self._trail) < 5 or self.speed < 0.3:
            return False
        t0, x0, y0 = self._trail[0]
        _, x1, y1 = self._trail[-1]
        if self._trail[-1][0] - t0 < self.stuck_seconds * 0.9:
            return False
        return math.hypot(x1 - x0, y1 - y0) < self.stuck_distance

    def nearest_index(self, x, y):
        """Path index, searched near the last one instead of globally.

        A global argmin is free to jump anywhere on the track — onto an opposing
        straight, across a hairpin, or wherever a localization correction lands.
        Every such jump instantly changes the reference heading, curvature,
        cross-track sign and speed target, and the controller answers with a
        full-rate correction to an error that was never real. Search a window
        around the last index instead, and fall back to a global search only
        when nothing nearby fits — which is what an actual teleport looks like.
        """
        n = len(self.xy)
        if not self.windowed_index:
            return int(np.argmin(np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)))
        if self._idx is None:
            self._idx = int(np.argmin(np.hypot(self.xy[:, 0] - x,
                                               self.xy[:, 1] - y)))
            return self._idx
        win = (np.arange(self._idx - self.track_back,
                         self._idx - self.track_back + self.track_window) % n)
        d = np.hypot(self.xy[win, 0] - x, self.xy[win, 1] - y)
        j = int(np.argmin(d))
        if float(d[j]) > self.track_lost_m:
            self._idx = int(np.argmin(np.hypot(self.xy[:, 0] - x,
                                               self.xy[:, 1] - y)))
        else:
            self._idx = int(win[j])
        return self._idx

    def predicted_state(self):
        x, y, yaw, t = self.pose
        dt = min(max(self.now() - t, 0.0), self.max_predict)
        yaw_mid = yaw + 0.5 * self.gyro_z * dt
        v = self.measured_speed()
        return (x + v * math.cos(yaw_mid) * dt,
                y + v * math.sin(yaw_mid) * dt,
                yaw + self.gyro_z * dt)

    def record_cmd(self, steer):
        t = self.now()
        self._cmd_hist.append((t, float(steer)))
        keep = t - max(self.cmd_delay, 0.0) - 0.5
        while len(self._cmd_hist) > 2 and self._cmd_hist[0][0] < keep:
            self._cmd_hist.pop(0)

    def predict_through_delay(self, x, y, yaw):
        """Where the car will be when the command chosen NOW takes effect.

        Measured (latency_probe.py): a steering command needs a median 0.135 s
        before the actuator feedback moves at all, because the devkit bridge is
        request/response -- it hands the simulator whatever we last published,
        once per ~11 Hz frame. During that dead time the car keeps driving on
        the commands already in flight. Optimising from the *current* pose
        therefore chooses a steering angle for a place the car has already
        left: at 3.4 m/s that is 0.46 m, which is 40% of the curvature ramp at
        the one corner that produces every collision.

        Integrate the kinematic bicycle forward by `cmd_delay` using, at each
        sub-step, the command that will be acting on the actuator then -- the
        one published `cmd_delay` earlier. The candidate search then starts
        from the last published angle, which is what the actuator will hold
        when the new one arrives.

        Swept on Porto, 32 laps each, same session (Sep 11): 0 -> 89.61 s,
        0.09 -> 88.32, 0.135 -> 88.03, **0.18 -> 87.32**, 0.24 -> 88.00, all
        zero collisions. 0.18 is the measured 0.135 s plus ~0.04 s of
        estimation lag, which is where a review put the effective delay too.
        Re-measure with latency_probe.py on any image whose sensor rate
        differs (the -compete images are reported to run faster).
        """
        v = max(self.measured_speed(), 0.4)
        now = self.now()
        n = max(1, int(round(self.cmd_delay / 0.01)))
        dt = self.cmd_delay / n
        hist = self._cmd_hist
        j = 0
        kap0 = None
        for k in range(n):
            t_eff = now + k * dt - self.cmd_delay
            while j + 1 < len(hist) and hist[j + 1][0] <= t_eff:
                j += 1
            delta = hist[j][1] if hist else self.steer
            kap = (vehicle.achieved_curvature(delta)
                   if self.delay_model == 'understeer'
                   else math.tan(delta) / vehicle.WHEELBASE)
            if self.delay_model == 'gyro':
                # The bicycle model over-rotates at large steering angles
                # (measured: hairpin cross-track p50 0.07 -> 0.15 m when the
                # pure model was used here). The gyro says what the car is
                # really doing under the command acting now; add only the
                # change the in-flight commands will make on top of that.
                if kap0 is None:
                    kap0 = kap
                yaw_rate = self.gyro_z + v * (kap - kap0)
            else:
                yaw_rate = v * kap
            yaw_mid = yaw + 0.5 * yaw_rate * dt
            x += v * math.cos(yaw_mid) * dt
            y += v * math.sin(yaw_mid) * dt
            yaw = wrap(yaw + yaw_rate * dt)
        return x, y, yaw

    def rollout_cost(self, delta0, e_lat, e_yaw, i0, v):
        """Cost of holding delta0 first, then relaxing towards the path.

        A full nonlinear program is overkill at this scale: the first step is
        what actually gets applied, so search over feasible first steps and
        evaluate each by simulating the model forward with a simple
        proportional relaxation afterwards. That keeps the rate constraint
        exact where it matters and stays inside the 20 ms control budget.
        """
        cost = 0.0
        delta = delta0
        prev = self.steer
        idx = i0
        s_travelled = 0.0
        # A fixed *time* horizon shrinks to nothing spatially as the car slows:
        # at 0.5 m/s, 12 x 0.06 s previews only 0.36 m, which is less than the
        # radius of a hairpin, so the controller cannot see the corner at all
        # and turns in far too late. Measured on ICRA: slowing from scale 0.5
        # to 0.3 made things WORSE (17 -> 83 collisions), which is the opposite
        # of what a grip or actuator limit would do. Hold a minimum arc length
        # per step instead; at racing speed step_dt is already the longer of
        # the two, so fast tracks are unaffected.
        ds = max(v * self.dt, self.min_step_m)
        for k in range(self.N):
            if self.rollout_profile_speed and k > 0:
                # The car brakes into corners: hold the preview to the path
                # it will actually cover in each step, not a constant speed.
                vk = min(v, float(self.v_prof[idx % len(self.v_prof)]) * self.speed_scale)
                ds = max(vk * self.dt, self.min_step_m)
            kap = float(self.kappa[idx % len(self.kappa)])
            denom = max(1.0 - kap * e_lat, 0.2)
            e_lat = e_lat + math.sin(e_yaw) * ds
            # The curvature this steering angle will actually produce. Below
            # ~0.15 rad the two agree; past it the kinematic term promises
            # turn-in the tyres do not have (see vehicle.achieved_curvature).
            kap_cmd = (vehicle.achieved_curvature(delta) if self.understeer_model
                       else math.tan(delta) / vehicle.WHEELBASE)
            e_yaw = e_yaw + (kap_cmd
                             - kap * math.cos(e_yaw) / denom) * ds
            e_yaw = wrap(e_yaw)
            cost += (self.w_lat * e_lat ** 2 + self.w_yaw * e_yaw ** 2
                     + self.w_steer * delta ** 2
                     + self.w_rate * (delta - prev) ** 2)
            prev = delta
            # after the first step, relax towards the feedforward steer
            # Relax towards the curvature feedforward, but no faster than the
            # actuator can actually move. The old fixed 50% step let the rollout
            # imagine reaching any angle in one step, so the first action was
            # judged under a future the car cannot deliver.
            ff = math.atan(vehicle.WHEELBASE * kap)
            if self.rate_limited_rollout:
                reachable = self.steer_rate * (ds / max(v, 0.1))
                delta = delta + max(-reachable, min(reachable, ff - delta))
            else:
                delta = delta + 0.5 * (ff - delta)
            s_travelled += ds
            idx = i0 + int(s_travelled / max(self.spacing, 1e-3))
        return cost

    def control_step(self):
        if self.pose is None:
            return

        # Pose has stopped arriving — localization died, the bridge dropped, or
        # the simulator went away. Commands LATCH here, so returning early would
        # leave the last throttle applied and drive the car into a wall at
        # speed. Coast instead, and *hold* the steering angle rather than
        # snapping to zero: mid-corner, zero steer points at the outside wall.
        if self.now() - self.pose[3] > self.stale_pose_s:
            self.publish(self.steer, 0.0)
            if not self._stale_warned:
                self.get_logger().error(
                    f'no pose for {self.stale_pose_s:.2f}s — coasting to a stop')
                self._stale_warned = True
            return
        self._stale_warned = False

        x, y, yaw = self.predicted_state()
        if self.cmd_delay > 0.0:
            x, y, yaw = self.predict_through_delay(x, y, yaw)

        i0 = self.nearest_index(x, y)
        path_yaw = float(self.path_yaw[i0])
        dx, dy = x - self.xy[i0, 0], y - self.xy[i0, 1]
        e_lat = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy
        e_yaw = wrap(yaw - path_yaw)

        # Wedged against a wall: no amount of forward steering frees the car,
        # it just grinds. Reverse out and re-approach. Reversing flips the sign
        # of the yaw rate, so steering towards +e_yaw is what rotates the nose
        # back onto the path.
        now = self.now()
        if self._escape_until is not None and now < self._escape_until:
            self.publish(self._escape_steer, -0.25)
            return
        if self._escape_until is not None:
            self._escape_until = None
            self._trail.clear()
        if self.is_stuck():
            self._escape_until = now + self.escape_seconds
            self._stuck_count += 1
            turn = e_yaw if abs(e_yaw) > 0.1 else e_lat
            self._escape_steer = math.copysign(vehicle.MAX_STEER, turn)
            self.get_logger().warn(
                f'stuck at ({x:.2f},{y:.2f}) e_lat={e_lat:.2f} '
                f'e_yaw={e_yaw:.2f} — reversing out (#{self._stuck_count})')
            return

        v = max(self.measured_speed(), 0.4)

        # Steering authority is not a constant — it is whatever the tyres can
        # pay for at this speed. delta = atan(L * ay_max / v^2) is the largest
        # angle that does not demand more lateral acceleration than exists.
        #
        # Without it the controller can ask for anything. Measured on the
        # ax-6.5 run: 0.202 rad commanded at 5.45 m/s, which through the
        # measured understeer curve demands ~15 m/s2 against the ~4.5 the tyres
        # give. The car slid wide into the wall while sitting 0.054 m from its
        # own line — the tracking was fine, the command was unpayable. This is
        # the cheap half of what ForzaETH's MAP controller does structurally:
        # derive steering from the lateral acceleration you can afford.
        #
        # It binds only where it should. Against an ay-3.0 plan it leaves ~50%
        # headroom everywhere: 0.319 rad allowed at 2.1 m/s where the hairpin
        # asks 0.319, 0.050 allowed at 5.4 m/s where the straight asks ~0.
        steer_cap = vehicle.MAX_STEER
        if self.ay_limit > 0.0:
            steer_cap = min(vehicle.MAX_STEER,
                            math.atan(vehicle.WHEELBASE
                                      * self.ay_limit / max(v * v, 0.25)))

        # Candidate steering TARGETS, not applied commands. A review flagged the
        # 4x as a bug — candidates land a rate-limit step apart, so several clip
        # to the same applied value — and narrowing it to exactly one step was
        # measured far worse: 27 collisions against 3 on the same line. The wide
        # range is doing real work. It picks a target to move towards, and the
        # rate limiter below is the actuator model; shrinking it to one step
        # leaves the rollout unable to evaluate committing to a larger angle,
        # and the cost differences between neighbouring candidates become
        # noise. Keep 4x. `reach_steps` exists to re-test, not to tune blindly.
        #
        # But that fixed the wrong variable. With reach_steps 4 and 9
        # candidates the grid spacing is 2*(4*rate*dt)/8 = rate*dt, EXACTLY the
        # step the applied command is clipped to below -- so however the search
        # votes, only three applied values are reachable: one step left, hold,
        # one step right. The search has 9 options and the actuator has 3.
        # Range and resolution are separate knobs; keep the range that was
        # measured to matter and buy resolution with `n_candidates` instead.
        reach = self.steer_rate * self.control_dt * self.reach_steps
        lo = max(-steer_cap, self.steer - reach)
        hi = min(steer_cap, self.steer + reach)
        if lo > hi:            # already beyond the cap: unwind towards it
            lo = hi = float(np.clip(self.steer, -steer_cap, steer_cap))
        best, best_cost = self.steer, float('inf')
        for cand in np.linspace(lo, hi, int(self.n_cand)):
            c = self.rollout_cost(float(cand), e_lat, e_yaw, i0, v)
            if c < best_cost:
                best_cost, best = c, float(cand)

        # Apply, still hard-limited by the actuator slew rate
        max_step = self.steer_rate * self.control_dt
        self.steer += float(np.clip(best - self.steer, -max_step, max_step))
        self.steer = float(np.clip(self.steer, -vehicle.MAX_STEER,
                                   vehicle.MAX_STEER))

        horizon = max(0.2, self.speed * self.brake_horizon)
        n_ahead = max(2, int(horizon / max(self.spacing, 1e-3)))
        idx = (i0 + np.arange(n_ahead)) % len(self.v_prof)
        self.speed = float(self.v_prof[idx].min()) * self.speed_scale
        # Planned acceleration for the throttle feedforward. Only pay it when
        # the braking horizon is NOT already holding the car back: `self.speed`
        # above is the MINIMUM profile speed over the horizon, so on the run-up
        # to a corner the target has already dropped while the plan's ax at the
        # car is still positive. Feeding that forward adds throttle exactly
        # where the car should be lifting -- measured, it put 12-14 collisions
        # a run into the same corner complex that every other collision this
        # session landed in.
        here = float(self.v_prof[i0 % len(self.v_prof)]) * self.speed_scale
        self.accel = (float(self.a_prof[i0 % len(self.a_prof)])
                      if self.speed >= here - 1e-3 else 0.0)

        self.pub_steer.publish(Float32(data=vehicle.steer_to_cmd(self.steer)))
        self.record_cmd(self.steer)
        v_fb = (self.speed_est.speed
                if self.brake_gain > 0.0 or self.throttle_ctl.kp > 0.0 else None)
        self.pub_throttle.publish(
            Float32(data=float(self.throttle_ctl.command(
                self.speed * self.grip_scale, v_fb, self.accel))))
        self.pub_cte.publish(Float32(data=float(e_lat)))

    def publish(self, steer, throttle):
        self.steer = float(np.clip(steer, -vehicle.MAX_STEER, vehicle.MAX_STEER))
        self.pub_steer.publish(Float32(data=vehicle.steer_to_cmd(self.steer)))
        self.record_cmd(self.steer)
        self.pub_throttle.publish(Float32(data=float(throttle)))


def main():
    rclpy.init()
    node = MPCTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
