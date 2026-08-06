"""Pure Pursuit raceline tracker.

Consumes the localization pose and the optimized raceline (with its velocity
profile) and emits steering/throttle. Speed is commanded open loop from the
profile because this platform has no usable speed feedback (see
vehicle.SpeedEstimator); the localization pose derivative provides a coarse
measured speed for the lookahead schedule only.

Falls back to publishing nothing when localization is unhealthy, so the
arbiter/safety layer can hand control to the reactive driver.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, Bool

from pit_crew import vehicle


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        p = self.declare_parameters('', [
            ('raceline_csv', '/home/data/racelines/track_v2_raceline.csv'),
            ('lookahead_min', 0.35),
            ('lookahead_max', 1.00),
            ('lookahead_per_mps', 0.15),
            ('speed_scale', 0.6),        # global scale on the profile
            ('throttle_gain', 0.040),     # open-loop throttle per m/s
            ('steer_alpha', 0.75),
            ('min_loc_score', 0.15),     # below this, stop commanding
            ('pose_timeout', 0.5),       # s without a pose = unhealthy
        ])
        (raceline_csv, self.la_min, self.la_max, self.la_per_mps,
         self.speed_scale, throttle_gain, self.steer_alpha,
         self.min_score, self.pose_timeout) = [x.value for x in p]

        rl = np.loadtxt(raceline_csv, delimiter=',')
        self.rl_xy = rl[:, 1:3]
        self.rl_v = rl[:, 5]
        self.rl_s = rl[:, 0]
        self.throttle_ctl = vehicle.ThrottleController(ff_gain=throttle_gain)
        self._steer_filt = 0.0
        self._last_pose = None
        self._last_t = None
        self._speed = 0.0
        self.loc_score = 1.0

        ns = '/autodrive/roboracer_1'
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, '/pit_crew/pose', self.on_pose, 10)
        self.create_subscription(Float32, '/pit_crew/loc_score',
                                 lambda m: setattr(self, 'loc_score', m.data), 10)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.pub_healthy = self.create_publisher(Bool, '/pit_crew/pp_active', 1)
        self.get_logger().info(
            f'pure pursuit ready: {len(self.rl_xy)} raceline pts, '
            f'v {self.rl_v.min():.1f}-{self.rl_v.max():.1f} m/s '
            f'x{self.speed_scale:.2f}')

    def on_pose(self, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y = msg.pose.position.x, msg.pose.position.y
        yaw = yaw_from_quat(msg.pose.orientation)

        if self._last_pose is not None and self._last_t is not None:
            dt = t - self._last_t
            if 1e-3 < dt < 0.5:
                v = math.hypot(x - self._last_pose[0], y - self._last_pose[1]) / dt
                if v < vehicle.TOP_SPEED:
                    self._speed += 0.3 * (v - self._speed)
        self._last_pose = (x, y)
        self._last_t = t

        if self.loc_score < self.min_score:
            self.pub_healthy.publish(Bool(data=False))
            return

        # Nearest raceline point, then walk forward by the lookahead distance
        d = np.hypot(self.rl_xy[:, 0] - x, self.rl_xy[:, 1] - y)
        i0 = int(np.argmin(d))
        lookahead = min(self.la_max,
                        max(self.la_min, self.la_min + self.la_per_mps * self._speed))
        n = len(self.rl_xy)
        i = i0
        acc = 0.0
        while acc < lookahead:
            j = (i + 1) % n
            acc += float(np.hypot(*(self.rl_xy[j] - self.rl_xy[i])))
            i = j
            if i == i0:
                break
        tx, ty = self.rl_xy[i]

        # Pure pursuit curvature in the vehicle frame
        dx, dy = tx - x, ty - y
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        ld2 = dx * dx + dy * dy
        curvature = 2.0 * ly / ld2 if ld2 > 1e-6 else 0.0
        steer_angle = math.atan(vehicle.WHEELBASE * curvature)
        self._steer_filt += self.steer_alpha * (steer_angle - self._steer_filt)

        v_target = float(self.rl_v[i0]) * self.speed_scale
        self.pub_steer.publish(
            Float32(data=float(vehicle.steer_to_cmd(self._steer_filt))))
        self.pub_throttle.publish(
            Float32(data=float(self.throttle_ctl.command(v_target))))
        self.pub_healthy.publish(Bool(data=True))


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
