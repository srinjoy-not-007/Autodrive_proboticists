"""Race logger: trajectory + collision/lap events to CSV (offline eval only).

Uses sim debug topics (ips, collision_count, lap metrics) which are permitted
for development/evaluation but NOT at race runtime — never launch this from
the submission entrypoint.

Usage: python3 race_logger.py [out_csv]   (default /home/data/logs/race_<ts>.csv)

COLUMN SEMANTICS (read this before analysing a log; `tools/racelog.py` is the
shared loader that encodes all of it, and offline tools should use it):

* `x, y` -- the IPS pose: **GROUND TRUTH** from the simulator. This is the
  opposite of the natural reading of the header; `px, py` look like "position"
  and are not.
* `px, py, pyaw, pstamp` -- the **LOCALIZATION ESTIMATE** (`/pit_crew/pose`)
  and the stamp of when it was true. The row carries the last received pose,
  about one sensor frame stale: compare it against the truth interpolated at
  `pstamp`, never at `t` (see on_pose below).
* `speed` -- the filtered **wheel-encoder** estimate written here, not truth.
  Truth speed is the IPS trajectory differentiated.
* `steering` -- the simulator's actuator feedback, in **RADIANS**.
  `cmd` -- the published `steering_command`, **NORMALIZED to [-1, 1]**
  (x MAX_STEER = 0.5236 for radians). The controllers publish
  `vehicle.steer_to_cmd()` and the devkit bridge passes that [-1, 1] value
  straight to Unity, while republishing `/steering` from the sim's own state.
* `cte` -- the controller's own reported cross-track error, in metres.
* `lap_count`, `collision_count` -- **cumulative across runs**; difference them
  against the run's first row.

THE c-2 RULE. When `collision_count` increments, `on_col` writes an extra row
(`collision_event` 1) -- but the simulator has already teleported the car to its
checkpoint and published that pose, so that row and the one before it both
carry the TELEPORTED position. For a collision whose counter steps at row `c`:
row `c` is the collision row (teleported pose), row `c-1` is the first
teleported IPS row, and row **`c-2` is the last pre-impact sample** -- the only
one that says where the car actually hit. Reading `c-1` piles every collision
onto the checkpoint. Attribution must also run on the RAW rows: filtering out
repeated IPS poses deletes row `c` itself.
"""

import csv
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
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
                           'collision_event', 'px', 'py', 'pyaw', 'cte',
                           'last_lap_time', 'best_lap_time', 'pstamp', 'cmd'])
        self.state = {'x': None, 'y': None, 'speed': 0.0, 'steering': 0.0, 'cmd': 0.0,
                      'throttle': 0.0, 'lap_count': 0, 'lap_time': 0.0,
                      'last_lap_time': 0.0, 'best_lap_time': 0.0,
                      'collision_count': 0, 'px': float('nan'),
                      'py': float('nan'), 'pyaw': float('nan'),
                      'pstamp': float('nan'), 'cte': float('nan')}
        self._enc_last = None
        self._col_last = None

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(Point, f'{ns}/ips', self.on_ips, qos)
        self.create_subscription(JointState, f'{ns}/left_encoder', self.on_enc, qos)
        self.create_subscription(Float32, f'{ns}/steering',
                                 lambda m: self.state.update(steering=m.data), qos)
        # The published steering COMMAND, next to the sim's actuator feedback
        # above, so command-vs-response can be read off one log.
        self.create_subscription(Float32, f'{ns}/steering_command',
                                 lambda m: self.state.update(cmd=m.data), qos)
        self.create_subscription(Float32, f'{ns}/throttle',
                                 lambda m: self.state.update(throttle=m.data), qos)
        self.create_subscription(Int32, f'{ns}/lap_count',
                                 lambda m: self.state.update(lap_count=m.data), qos)
        self.create_subscription(Float32, f'{ns}/lap_time',
                                 lambda m: self.state.update(lap_time=m.data), qos)
        # The official score is the sum of the simulator's own per-lap times:
        # `last_lap_time` latches the completed lap's duration at each crossing
        # (see the devkit bridge's publish_last_lap_time_data), which is what
        # the organisers' Foxglove race_time script accumulates.
        self.create_subscription(Float32, f'{ns}/last_lap_time',
                                 lambda m: self.state.update(last_lap_time=m.data), qos)
        self.create_subscription(Float32, f'{ns}/best_lap_time',
                                 lambda m: self.state.update(best_lap_time=m.data), qos)
        self.create_subscription(Int32, f'{ns}/collision_count', self.on_col, qos)
        # Localization estimate (evaluation only): lets a log show estimate
        # vs IPS truth, e.g. whether the hairpin inside cut is a pose bias.
        self.create_subscription(PoseStamped, '/pit_crew/pose', self.on_pose, 10)
        self.create_subscription(Float32, '/pit_crew/cte',
                                 lambda m: self.state.update(cte=round(m.data, 4)), 10)
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

    def on_pose(self, msg):
        import math
        q = msg.pose.orientation
        self.state['px'] = round(msg.pose.position.x, 4)
        self.state['py'] = round(msg.pose.position.y, 4)
        # WHEN that pose was true, not when it was written. Each row carries the
        # LAST RECEIVED pose, i.e. about one sensor frame of staleness, so at
        # 3.9 m/s the logged estimate sits 0.40 m behind the IPS row at a 0.104 s
        # period and 0.34 m behind at 0.088 s. Without this column that
        # period-proportional AGE reads as a period-proportional forward BIAS --
        # it was measured as one (0.15 m at the hairpin) and it is not: de-staled
        # by one frame the two rates agree to 0.007 m (medians +0.100 / +0.093,
        # p95 0.201 / 0.197). `t - pstamp` is the age; compare the estimate with
        # the IPS trajectory interpolated at `pstamp`, never at `t`.
        self.state['pstamp'] = round(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 4)
        self.state['pyaw'] = round(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                              1 - 2 * (q.y * q.y + q.z * q.z)), 4)

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
                           collision_event, s['px'], s['py'], s['pyaw'], s['cte'],
                           f"{s['last_lap_time']:.3f}",
                           f"{s['best_lap_time']:.3f}", s['pstamp'],
                           f"{s['cmd']:.3f}"])

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
