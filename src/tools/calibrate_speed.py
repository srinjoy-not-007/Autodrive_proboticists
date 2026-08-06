"""Measure the throttle -> speed curve in the simulator (development only).

Uses IPS for ground-truth speed, which is fine offline but must never run at
race time. Holds each throttle level with zero steering until the LiDAR says a
wall is close, then resets the sim and moves to the next level.

Usage (inside devkit container, sim running and connected):
  python3 calibrate_speed.py [out_csv]
"""

import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool

LEVELS = [0.03, 0.05, 0.07, 0.10, 0.15, 0.20, 0.30, 0.50, 0.80]
WALL_STOP = 2.0        # m of clearance ahead at which to abort a level
MAX_HOLD = 6.0         # s per level


class SpeedCalibration(Node):

    def __init__(self, out_csv):
        super().__init__('calibrate_speed')
        self.out_csv = out_csv
        self.pos = None
        self.pos_t = None
        self.speed = 0.0
        self.front = 10.0
        self.samples = []          # (level, t_since_start, speed)
        self.results = []
        self.level_i = 0
        self.t0 = None
        self.resume_at = 0.0

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(Point, f'{ns}/ips', self.on_ips, qos)
        self.create_subscription(LaserScan, f'{ns}/lidar', self.on_scan, qos)
        self.pub_thr = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.pub_str = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_reset = self.create_publisher(Bool, '/autodrive/reset_command', 1)
        self.timer = self.create_timer(0.02, self.step)
        self.get_logger().info(f'calibrating levels {LEVELS}')

    def on_ips(self, msg):
        t = time.monotonic()
        if self.pos is not None and self.pos_t is not None:
            dt = t - self.pos_t
            if dt > 1e-3:
                v = math.hypot(msg.x - self.pos[0], msg.y - self.pos[1]) / dt
                if v < 30.0:
                    self.speed += 0.4 * (v - self.speed)
        self.pos = (msg.x, msg.y)
        self.pos_t = t

    def on_scan(self, scan):
        r = np.asarray(scan.ranges, dtype=np.float64)
        r = np.where(np.isfinite(r), r, scan.range_max)
        ang = scan.angle_min + np.arange(r.size) * scan.angle_increment
        self.front = float(np.min(r[np.abs(ang) <= math.radians(6.0)]))

    def step(self):
        now = time.monotonic()
        if self.level_i >= len(LEVELS):
            self.pub_thr.publish(Float32(data=0.0))
            self.report()
            self.timer.cancel()
            rclpy.shutdown()
            return
        if now < self.resume_at:
            self.pub_thr.publish(Float32(data=0.0))
            self.pub_str.publish(Float32(data=0.0))
            self.t0 = None
            return
        if self.t0 is None:
            self.t0 = now
            self.speed = 0.0
            self.samples = []

        level = LEVELS[self.level_i]
        elapsed = now - self.t0
        self.pub_str.publish(Float32(data=0.0))
        self.pub_thr.publish(Float32(data=float(level)))
        self.samples.append((elapsed, self.speed))

        if self.front < WALL_STOP or elapsed > MAX_HOLD:
            self.finish_level(level, now)

    def finish_level(self, level, now):
        s = np.array(self.samples)
        held = s[s[:, 0] > 0.4] if len(s) else s
        peak = float(held[:, 1].max()) if len(held) else 0.0
        # steady state = median of the last 0.8 s before abort
        tail = held[held[:, 0] > held[:, 0].max() - 0.8] if len(held) else held
        steady = float(np.median(tail[:, 1])) if len(tail) else 0.0
        # acceleration over the first 0.5 s of motion
        early = held[held[:, 0] < 1.0] if len(held) else held
        accel = 0.0
        if len(early) > 5:
            accel = float(np.polyfit(early[:, 0], early[:, 1], 1)[0])
        dur = float(held[:, 0].max()) if len(held) else 0.0
        self.results.append((level, steady, peak, accel, dur))
        self.get_logger().info(
            f'throttle {level:.2f}: steady {steady:.2f} peak {peak:.2f} m/s, '
            f'accel {accel:.2f} m/s^2 over {dur:.1f}s')
        self.level_i += 1
        self.t0 = None
        self.pub_thr.publish(Float32(data=0.0))
        self.pub_reset.publish(Bool(data=True))
        self.resume_at = now + 3.0

    def report(self):
        r = np.array(self.results)
        np.savetxt(self.out_csv, r, delimiter=',', fmt='%.4f',
                   header='throttle,steady_mps,peak_mps,accel_mps2,duration_s',
                   comments='# ')
        print('\n=== throttle -> speed ===')
        for level, steady, peak, accel, dur in self.results:
            print(f'  {level:.2f} -> steady {steady:5.2f}  peak {peak:5.2f} m/s'
                  f'  accel {accel:5.2f} m/s^2  ({dur:.1f}s)')
        use = r[(r[:, 1] > 0.2) & (r[:, 3] > 0)]
        if len(use) >= 2:
            k = np.polyfit(use[:, 1], use[:, 0], 1)
            print(f'\nthrottle ~= {k[0]:.4f} * v + {k[1]:.4f}   '
                  f'(ThrottleController ff_gain={k[0]:.4f}, idle={k[1]:.4f})')
        print(f'max accel observed: {r[:, 3].max():.2f} m/s^2')
        print(f'wrote {self.out_csv}')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/home/data/calibration/speed.csv'
    import os
    os.makedirs(os.path.dirname(out), exist_ok=True)
    rclpy.init()
    node = SpeedCalibration(out)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
