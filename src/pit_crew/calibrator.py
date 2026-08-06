"""Vehicle dynamics calibration experiments (task #3).

Runs open-loop throttle experiments while logging speed, then resets the sim
(via /autodrive/reset_command) when the wall gets close. Produces a CSV in
/home/data (mounted from repo data/) for offline fitting of:
  - throttle -> steady-state speed curve
  - acceleration / braking envelopes

Usage (inside devkit container, sim running):
  ros2 run pit_crew calibrator
  ros2 run pit_crew calibrator --ros-args -p mode:=brake
"""

import csv
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float32, Bool

from pit_crew import vehicle


class Calibrator(Node):

    def __init__(self):
        super().__init__('calibrator')
        self.mode = self.declare_parameter('mode', 'throttle').value
        self.levels = self.declare_parameter(
            'levels', [0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.60, 0.80, 1.00]).value
        self.hold_s = self.declare_parameter('hold_s', 12.0).value
        self.wall_stop_m = self.declare_parameter('wall_stop_m', 2.5).value
        self.brake_speed = self.declare_parameter('brake_speed', 5.0).value
        out_dir = self.declare_parameter('out_dir', '/home/data/calibration').value

        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(out_dir, f'{self.mode}_{stamp}.csv')
        self.csv = csv.writer(open(self.csv_path, 'w', newline=''))
        self.csv.writerow(['t', 'phase', 'level', 'throttle_cmd', 'speed',
                           'front_clearance'])

        self.speed_est = vehicle.SpeedEstimator()
        self.front_clear = 10.0
        self.level_idx = 0
        self.phase = 'accelerate'         # brake mode: accelerate -> brake
        self.phase_t0 = None
        self.resetting_until = 0.0

        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(LaserScan, f'{ns}/lidar', self.on_scan, qos)
        self.create_subscription(JointState, f'{ns}/left_encoder',
                                 lambda m: self.on_encoder('L', m), qos)
        self.create_subscription(JointState, f'{ns}/right_encoder',
                                 lambda m: self.on_encoder('R', m), qos)
        self.pub_steer = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_throttle = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.pub_reset = self.create_publisher(Bool, '/autodrive/reset_command', 1)

        self.timer = self.create_timer(0.025, self.step)   # 40 Hz
        self.get_logger().info(
            f'Calibrator mode={self.mode}, logging to {self.csv_path}')

    def on_encoder(self, wheel, msg: JointState):
        if msg.position:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(wheel, t, msg.position[0])

    def on_scan(self, scan: LaserScan):
        r = np.asarray(scan.ranges, dtype=np.float64)
        r = np.where(np.isfinite(r), r, vehicle.LIDAR_RANGE_MAX)
        n = r.size
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        ahead = np.abs(angles) <= math.radians(5.0)
        self.front_clear = float(np.min(r[ahead]))

    # ------------------------------------------------------------------ #

    def step(self):
        now = time.monotonic()
        if self.level_idx >= len(self.levels):
            self.pub_throttle.publish(Float32(data=0.0))
            self.get_logger().info(f'DONE. Data: {self.csv_path}')
            self.timer.cancel()
            return
        if now < self.resetting_until:
            self.pub_throttle.publish(Float32(data=0.0))
            return
        if self.phase_t0 is None:
            self.phase_t0 = now

        level = float(self.levels[self.level_idx])
        elapsed = now - self.phase_t0
        v = self.speed_est.speed

        if self.mode == 'throttle':
            cmd = level
            done = elapsed > self.hold_s
        else:  # brake mode: accelerate to brake_speed, then full brake to stop
            if self.phase == 'accelerate':
                cmd = 1.0
                if v >= self.brake_speed:
                    self.phase = 'brake'
            else:
                cmd = -level          # braking level (throttle reversal)
                if v < 0.15:
                    self.phase = 'accelerate'
            done = self.phase == 'accelerate' and elapsed > 1.0 and v < 0.15

        # Abort the run before hitting the wall
        if self.front_clear < self.wall_stop_m or done:
            self.next_level(now)
            return

        self.pub_steer.publish(Float32(data=0.0))
        self.pub_throttle.publish(Float32(data=cmd))
        self.csv.writerow([f'{now:.3f}', self.phase, level, f'{cmd:.2f}',
                           f'{v:.3f}', f'{self.front_clear:.2f}'])

    def next_level(self, now):
        self.get_logger().info(
            f'level {self.levels[self.level_idx]} done '
            f'(peak speed {self.speed_est.speed:.2f} m/s); resetting sim')
        self.level_idx += 1
        self.phase = 'accelerate'
        self.phase_t0 = None
        self.pub_throttle.publish(Float32(data=0.0))
        self.pub_reset.publish(Bool(data=True))
        self.resetting_until = now + 3.0


def main():
    rclpy.init()
    node = Calibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
