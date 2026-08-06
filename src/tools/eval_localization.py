"""Compare particle-filter pose against IPS ground truth (offline eval only).

Run alongside the localization node while replaying a bag; prints error
statistics on shutdown.

Usage: python3 eval_localization.py [out_csv]
"""

import csv
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32


class EvalLocalization(Node):

    def __init__(self, out_csv):
        super().__init__('eval_localization')
        self.rows = []
        self.truth = None
        self.score = 0.0
        self.out_csv = out_csv
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Point, '/autodrive/roboracer_1/ips',
                                 self.on_ips, qos)
        self.create_subscription(Float32, '/pit_crew/loc_score',
                                 lambda m: setattr(self, 'score', m.data), 10)
        self.create_subscription(PoseStamped, '/pit_crew/pose', self.on_pose, 10)

    def on_ips(self, msg):
        self.truth = (msg.x, msg.y)

    def on_pose(self, msg):
        if self.truth is None:
            return
        ex = msg.pose.position.x - self.truth[0]
        ey = msg.pose.position.y - self.truth[1]
        self.rows.append((time.time(), msg.pose.position.x, msg.pose.position.y,
                          self.truth[0], self.truth[1], math.hypot(ex, ey),
                          self.score))

    def report(self):
        if not self.rows:
            print('NO POSE ESTIMATES RECEIVED')
            return
        err = np.array([r[5] for r in self.rows])
        score = np.array([r[6] for r in self.rows])
        n = len(err)
        # Ignore the initial convergence transient
        settled = err[int(n * 0.1):]
        print(f'\n=== localization vs IPS ({n} samples) ===')
        print(f'error  mean {err.mean():.3f}  median {np.median(err):.3f}  '
              f'p95 {np.percentile(err, 95):.3f}  max {err.max():.3f} m')
        print(f'settled (after first 10%): mean {settled.mean():.3f}  '
              f'p95 {np.percentile(settled, 95):.3f} m')
        print(f'within 0.10 m: {100*np.mean(err < 0.10):.1f}%   '
              f'within 0.25 m: {100*np.mean(err < 0.25):.1f}%   '
              f'within 0.50 m: {100*np.mean(err < 0.50):.1f}%')
        print(f'loc_score mean {score.mean():.3f} min {score.min():.3f}')
        with open(self.out_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'est_x', 'est_y', 'true_x', 'true_y', 'err', 'score'])
            w.writerows(self.rows)
        print(f'wrote {self.out_csv}')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/home/data/logs/loc_eval.csv'
    rclpy.init()
    node = EvalLocalization(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
