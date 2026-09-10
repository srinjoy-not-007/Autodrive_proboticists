"""Race logger: trajectory + collision/lap events to CSV (offline eval only).

Uses sim debug topics (ips, collision_count, lap metrics) which are permitted
for development/evaluation but NOT at race runtime — never launch this from
the submission entrypoint.

Usage: python3 race_logger.py [out_csv]   (default /home/data/logs/race_<ts>.csv)
"""

import csv
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32


class RaceLogger(Node):

    WHEEL_RADIUS = 0.059

    def __init__(self, out_csv):
        super().__init__('race_logger')
        os.makedirs(os.path.dirname(out_csv), exist_ok=True)
        self.f = open(out_csv, 'w', newline='', buffering=1)
        self.csv = csv.writer(self.f)
        self.csv.writerow(['t', 'x', 'y', 'speed', 'steering', 'throttle',
                           'lap_count', 'lap_time', 'collision_count',
                           'collision_event'])
        self.state = {'x': None, 'y': None, 'speed': 0.0, 'steering': 0.0,
                      'throttle': 0.0, 'lap_count': 0, 'lap_time': 0.0,
                      'collision_count': 0}
        self._enc_last = None
        self._col_last = None

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(Point, f'{ns}/ips', self.on_ips, qos)
        self.create_subscription(JointState, f'{ns}/left_encoder', self.on_enc, qos)
        self.create_subscription(Float32, f'{ns}/steering',
                                 lambda m: self.state.update(steering=m.data), qos)
        self.create_subscription(Float32, f'{ns}/throttle',
                                 lambda m: self.state.update(throttle=m.data), qos)
        self.create_subscription(Int32, f'{ns}/lap_count',
                                 lambda m: self.state.update(lap_count=m.data), qos)
        self.create_subscription(Float32, f'{ns}/lap_time',
                                 lambda m: self.state.update(lap_time=m.data), qos)
        self.create_subscription(Int32, f'{ns}/collision_count', self.on_col, qos)
        self.get_logger().info(f'logging to {out_csv}')

    def on_enc(self, msg):
        if msg.position:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self._enc_last:
                dt = t - self._enc_last[0]
                if 1e-4 < dt < 0.5:
                    v = self.WHEEL_RADIUS * (msg.position[0] - self._enc_last[1]) / dt
                    if abs(v) <= 25.0:
                        inn = max(-2.0, min(2.0, v - self.state['speed']))
                        self.state['speed'] += 0.3 * inn
            self._enc_last = (t, msg.position[0])

    def on_col(self, msg):
        prev = self._col_last
        self._col_last = msg.data
        self.state['collision_count'] = msg.data
        if prev is not None and msg.data > prev:
            self.write_row(collision_event=1)
            self.get_logger().info(
                f"COLLISION #{msg.data} at ({self.state['x']}, {self.state['y']}) "
                f"v={self.state['speed']:.2f} steer={self.state['steering']:.2f}")

    def on_ips(self, msg):
        self.state['x'], self.state['y'] = round(msg.x, 4), round(msg.y, 4)
        self.write_row()

    def write_row(self, collision_event=0):
        s = self.state
        if s['x'] is None:
            return
        self.csv.writerow([f'{time.time():.3f}', s['x'], s['y'],
                           f"{s['speed']:.3f}", f"{s['steering']:.3f}",
                           f"{s['throttle']:.3f}", s['lap_count'],
                           f"{s['lap_time']:.2f}", s['collision_count'],
                           collision_event])

    def destroy_node(self):
        self.f.flush()
        self.f.close()
        super().destroy_node()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else \
        f'/home/data/logs/race_{time.strftime("%Y%m%d_%H%M%S")}.csv'
    rclpy.init()
    node = RaceLogger(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
