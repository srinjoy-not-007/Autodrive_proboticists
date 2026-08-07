"""Stanley raceline tracker with curvature feedforward and pose prediction.

Why not pure pursuit: it derives steering solely from a lookahead point on a
pose that is up to 0.1 s old, and measured on this car it cannot hold the
1.3 m corridor above ~1 m/s average (see docs/STATUS.md). Two changes here:

* **Feedforward.** The raceline's curvature is known in advance, so most of the
  steering is computed from the plan (atan(L*kappa)) rather than inferred from
  cross-track geometry. Feedback then only has to correct the residual.
* **Prediction.** Every sensor on this platform arrives at ~10 Hz over one
  WebSocket, so there is no extra information between scans — but there is
  extra *latency*. The pose is dead-reckoned forward to the current instant and
  the control loop runs at `control_hz`, so steering is neither stale nor
  quantised to 10 Hz.

Stanley's reference point is the front axle, which is what makes it stable
under heading error where pure pursuit tends to cut corners.
"""

import math

import numpy as np
import rclpy
from scipy import ndimage
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from pit_crew import vehicle


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class StanleyTracker(Node):

    def __init__(self):
        super().__init__('stanley')
        p = self.declare_parameters('', [
            ('raceline_csv', '/home/data/racelines/track_v2_safe.csv'),
            ('speed_scale', 0.30),
            ('k_cte', 1.6),              # cross-track gain
            ('k_soft', 0.6),             # m/s, keeps gain finite at low speed
            ('k_heading', 1.0),          # heading-error gain
            ('k_ff', 1.0),               # curvature feedforward gain
            ('k_damp', 0.18),            # damping on steering rate
            ('pose_blend', 0.55),        # how hard a new fix corrects prediction
            ('steer_rate_limit', 3.2),   # rad/s, the actuator's real slew rate
            ('control_hz', 50.0),
            ('brake_horizon_s', 0.35),   # short: the profile already brakes
            ('throttle_gain', 0.040),
            ('max_predict_s', 0.35),     # never extrapolate further than this
            ('steer_alpha', 0.6),
        ])
        (raceline_csv, self.speed_scale, self.k_cte, self.k_soft,
         self.k_heading, self.k_ff, self.k_damp, self.pose_blend,
         self.steer_rate, control_hz, self.brake_horizon, throttle_gain,
         self.max_predict, self.steer_alpha) = [x.value for x in p]
        self.control_dt = 1.0 / control_hz

        rl = np.loadtxt(raceline_csv, delimiter=',')
        self.xy = rl[:, 1:3]
        self.kappa = rl[:, 4]
        self.v_prof = rl[:, 5]
        seg = np.hypot(np.diff(self.xy[:, 0]), np.diff(self.xy[:, 1]))
        self.spacing = float(np.median(seg)) if len(seg) else 0.1
        # Path heading per point, from the geometry (TUM's psi column is
        # north-zero, not standard math angles). Taken over a ~0.5 m baseline
        # rather than between adjacent points: at 0.1 m spacing the spline's
        # centimetre-level wiggle dominates the heading estimate, and that noise
        # goes straight into the steering command as visible left-right swing.
        span = max(1, int(round(0.5 / max(self.spacing, 1e-3))))
        fwd = np.roll(self.xy, -span, axis=0) - np.roll(self.xy, span, axis=0)
        self.path_yaw = np.unwrap(np.arctan2(fwd[:, 1], fwd[:, 0]))
        # Curvature likewise smoothed over the same baseline
        self.kappa = ndimage.uniform_filter1d(self.kappa, size=2 * span + 1,
                                              mode='wrap')

        self.throttle_ctl = vehicle.ThrottleController(ff_gain=throttle_gain)
        self.pose = None          # (x, y, yaw, stamp_monotonic)
        self.gyro_z = 0.0
        self.speed = 0.0          # commanded, used for prediction
        self.steer = 0.0

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(PoseStamped, '/pit_crew/pose', self.on_pose, 10)
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.pub_cte = self.create_publisher(Float32, '/pit_crew/cte', 10)
        self.create_timer(1.0 / control_hz, self.control_step)
        self.get_logger().info(
            f'stanley ready: {len(self.xy)} pts, v {self.v_prof.min():.1f}-'
            f'{self.v_prof.max():.1f} m/s x{self.speed_scale:.2f}, '
            f'{control_hz:.0f} Hz control')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg: Imu):
        self.gyro_z = float(msg.angular_velocity.z)

    def on_pose(self, msg: PoseStamped):
        """Blend a new fix into the running state rather than snapping to it.

        The filter publishes a weighted mean of resampled particles at 10 Hz, so
        consecutive fixes differ by centimetres. Snapping made the cross-track
        error jump every 100 ms and the car visibly swung left-right chasing it.
        Correcting a fraction of the innovation keeps the state continuous.
        """
        mx, my = msg.pose.position.x, msg.pose.position.y
        myaw = yaw_from_quat(msg.pose.orientation)
        t = self.now()
        if self.pose is None:
            self.pose = (mx, my, myaw, t)
            return
        px, py, pyaw = self.predicted_state()
        a = self.pose_blend
        self.pose = (px + a * (mx - px),
                     py + a * (my - py),
                     pyaw + a * wrap(myaw - pyaw),
                     t)

    def predicted_state(self):
        """Dead-reckon the last filter pose forward to now."""
        x, y, yaw, t = self.pose
        dt = min(max(self.now() - t, 0.0), self.max_predict)
        yaw_mid = yaw + 0.5 * self.gyro_z * dt
        return (x + self.speed * math.cos(yaw_mid) * dt,
                y + self.speed * math.sin(yaw_mid) * dt,
                yaw + self.gyro_z * dt)

    def control_step(self):
        if self.pose is None:
            return
        x, y, yaw = self.predicted_state()

        # Stanley references the FRONT axle
        fx = x + vehicle.WHEELBASE * math.cos(yaw)
        fy = y + vehicle.WHEELBASE * math.sin(yaw)

        d = np.hypot(self.xy[:, 0] - fx, self.xy[:, 1] - fy)
        i = int(np.argmin(d))
        path_yaw = float(self.path_yaw[i])

        # Signed cross-track error: left of the path is positive
        dx, dy = fx - self.xy[i, 0], fy - self.xy[i, 1]
        cte = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

        heading_err = wrap(path_yaw - yaw)
        v_eff = max(self.speed, 0.3)
        delta = (self.k_heading * heading_err
                 + math.atan2(self.k_cte * -cte, v_eff + self.k_soft)
                 + self.k_ff * math.atan(vehicle.WHEELBASE * float(self.kappa[i]))
                 - self.k_damp * self.gyro_z)

        # Never command faster than the actuator can move (3.2 rad/s):
        # commanding past the slew rate just adds phase lag, and phase lag in a
        # feedback loop is what turns a correction into an oscillation.
        target = float(self.steer + self.steer_alpha * (delta - self.steer))
        max_step = self.steer_rate * self.control_dt
        self.steer += float(np.clip(target - self.steer, -max_step, max_step))
        self.steer = float(np.clip(self.steer, -vehicle.MAX_STEER, vehicle.MAX_STEER))

        # A short defensive horizon only. The tph velocity profile already
        # includes braking distance, so a long horizon double-counts it and
        # pins the car near the profile's global minimum (measured: 52 s/lap
        # at a 1.2 s horizon versus 32 s with none).
        horizon = max(0.2, self.speed * self.brake_horizon)
        n_ahead = max(2, int(horizon / max(self.spacing, 1e-3)))
        idx = (i + np.arange(n_ahead)) % len(self.v_prof)
        self.speed = float(self.v_prof[idx].min()) * self.speed_scale

        self.pub_steer.publish(Float32(data=vehicle.steer_to_cmd(self.steer)))
        self.pub_throttle.publish(
            Float32(data=float(self.throttle_ctl.command(self.speed))))
        self.pub_cte.publish(Float32(data=float(cte)))


def main():
    rclpy.init()
    node = StanleyTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
