"""Command arbiter and safety layer — the only node that drives the car.

Pure pursuit follows the raceline open-loop against a map; it has no idea a
wall is in front of it. That is fine until something puts the car somewhere the
raceline does not expect — a checkpoint teleport after a collision, or a
localization wobble — at which point it will drive into the wall and keep doing
it. Observed as runaway collision cascades (57 -> 93 -> 129 -> 162).

So nothing reaches the actuators without passing through here:

  RACELINE  pure pursuit commands, when localization is healthy and the path
            ahead is clear
  REACTIVE  Follow-the-Gap, when localization is unhealthy/stale
  BRAKE     hard stop and steer to the largest gap, when time-to-collision in
            the direction of travel drops below the threshold

Inputs are LiDAR plus the two controllers' proposed commands; only permitted
runtime topics are used.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from pit_crew import vehicle


class Arbiter(Node):

    def __init__(self):
        super().__init__('arbiter')
        p = self.declare_parameters('', [
            ('min_loc_score', 0.12),      # below -> reactive
            ('pose_timeout', 0.6),        # s without a pose -> reactive
            ('ttc_brake', 0.45),          # s to impact -> enter BRAKE
            ('ttc_clear', 0.90),          # s to impact -> may leave BRAKE
            ('min_clearance', 0.30),      # m ahead -> enter BRAKE regardless
            ('clear_clearance', 0.55),    # m ahead -> may leave BRAKE
            ('brake_throttle', -0.3),
            ('corridor_half_width', 0.22),  # m; beams within this are "ahead"
            ('min_dwell_s', 0.4),         # min time in a mode before leaving it
            ('brake_dwell_s', 0.5),       # min time braking once engaged
        ])
        (self.min_score, self.pose_timeout, self.ttc_brake, self.ttc_clear,
         self.min_clear, self.clear_clear, self.brake_throttle,
         self.corridor_hw, self.min_dwell, self.brake_dwell) = \
            [x.value for x in p]
        self.mode_since = 0.0

        self.pp = None            # (steer, throttle, stamp)
        self.ftg = None
        self.loc_score = 0.0
        self.last_pose_t = 0.0
        self.speed = 0.0
        self.mode = 'REACTIVE'
        self._front = 10.0
        self._gap_steer = 0.0

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(LaserScan, f'{ns}/lidar', self.on_scan, qos)
        self.create_subscription(Float32, f'{ns}/throttle', self.on_throttle, qos)
        self.create_subscription(Float32, '/pit_crew/loc_score',
                                 self.on_score, 10)
        self.create_subscription(Float32, '/pit_crew/pp/steering',
                                 lambda m: self._set('pp', 0, m.data), 10)
        self.create_subscription(Float32, '/pit_crew/pp/throttle',
                                 lambda m: self._set('pp', 1, m.data), 10)
        self.create_subscription(Float32, '/pit_crew/ftg/steering',
                                 lambda m: self._set('ftg', 0, m.data), 10)
        self.create_subscription(Float32, '/pit_crew/ftg/throttle',
                                 lambda m: self._set('ftg', 1, m.data), 10)

        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.pub_mode = self.create_publisher(String, '/pit_crew/mode', 1)
        self.get_logger().info('arbiter ready — sole publisher to actuators')

    def _set(self, which, idx, value):
        cur = getattr(self, which) or [0.0, 0.0]
        cur = list(cur)
        cur[idx] = float(value)
        setattr(self, which, cur)

    def on_score(self, msg):
        self.loc_score = float(msg.data)
        self.last_pose_t = self.now()

    def on_throttle(self, msg):
        self.speed = max(0.0, float(msg.data)) * 25.0   # calibrated curve

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, scan: LaserScan):
        r = np.asarray(scan.ranges, dtype=np.float64)
        r = np.where(np.isfinite(r), r, scan.range_max)
        ang = scan.angle_min + np.arange(r.size) * scan.angle_increment

        # Clearance along the corridor the car actually occupies: a beam counts
        # only if its lateral offset is within the car's half width.
        lateral = np.abs(r * np.sin(ang))
        forward = r * np.cos(ang)
        in_corridor = (lateral < self.corridor_hw) & (forward > 0) & \
                      (np.abs(ang) < math.radians(70))
        self._front = float(np.min(forward[in_corridor])) if in_corridor.any() \
            else scan.range_max

        # Largest-gap direction, used while braking
        wide = np.abs(ang) < math.radians(90)
        idx = np.flatnonzero(wide)
        self._gap_steer = float(ang[idx[np.argmax(r[idx])]]) if idx.size else 0.0

        self.decide()

    def decide(self):
        """Pick a mode with hysteresis and a dwell time.

        Deciding afresh each scan makes the modes chatter: an earlier version
        logged ~3200 RACELINE->BRAKE and ~3200 REACTIVE->RACELINE switches in a
        single run, so the car alternately accelerated and braked and ground
        along the wall. Entering a mode is therefore cheap, leaving it is not:
        separate enter/exit thresholds, plus a minimum time in mode.
        """
        now = self.now()
        ttc = self._front / max(self.speed, 0.05)
        loc_ok = (self.loc_score >= self.min_score and
                  (now - self.last_pose_t) < self.pose_timeout)
        in_mode = now - self.mode_since

        danger = (self._front < self.min_clear) or (ttc < self.ttc_brake)
        still_risky = (self._front < self.clear_clear) or (ttc < self.ttc_clear)

        if self.mode == 'BRAKE':
            # Leave only once genuinely clear and after the dwell time
            mode = 'BRAKE' if (still_risky or in_mode < self.brake_dwell) \
                else ('RACELINE' if loc_ok and self.pp is not None else 'REACTIVE')
        elif danger:
            mode = 'BRAKE'                      # entry is always immediate
        elif self.mode == 'RACELINE':
            mode = 'RACELINE' if (loc_ok or in_mode < self.min_dwell) else 'REACTIVE'
        else:                                    # REACTIVE
            mode = 'RACELINE' if (loc_ok and self.pp is not None and
                                  in_mode >= self.min_dwell) else 'REACTIVE'

        if mode == 'BRAKE':
            steer = vehicle.steer_to_cmd(self._gap_steer)
            throttle = self.brake_throttle
        elif mode == 'RACELINE' and self.pp is not None:
            steer, throttle = self.pp
        elif self.ftg is not None:
            mode = 'REACTIVE'
            steer, throttle = self.ftg
        else:
            mode = 'REACTIVE'
            steer, throttle = 0.0, 0.0

        if mode != self.mode:
            self.get_logger().info(
                f'{self.mode} -> {mode} (front {self._front:.2f} m, '
                f'ttc {ttc:.2f} s, loc {self.loc_score:.2f})')
            self.mode = mode
            self.mode_since = now

        self.pub_steer.publish(Float32(data=float(np.clip(steer, -1, 1))))
        self.pub_throttle.publish(Float32(data=float(np.clip(throttle, -1, 1))))
        self.pub_mode.publish(String(data=mode))


def main():
    rclpy.init()
    node = Arbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
