"""Print LiDAR argmin index over time — used to verify beam ordering.

While the car yaws LEFT (CCW), wall features must move to LOWER indices if
the ranges array follows standard ROS ordering (index 0 = angle_min = right).
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    rclpy.init()
    node = Node('probe_scan')
    state = {'last_print': 0.0}

    def cb(s):
        now = time.monotonic()
        if now - state['last_print'] < 0.4:
            return
        state['last_print'] = now
        r = np.asarray(s.ranges)
        i = int(np.argmin(r))
        print(f't={now:.2f} argmin_idx={i} min={r[i]:.3f} '
              f'r0={r[0]:.2f} r270={r[270]:.2f} r540={r[540]:.2f} '
              f'r810={r[810]:.2f} r1079={r[1079]:.2f}', flush=True)

    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(LaserScan, '/autodrive/roboracer_1/lidar', cb, qos)
    end = time.monotonic() + duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
