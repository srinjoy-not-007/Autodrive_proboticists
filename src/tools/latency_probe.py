"""Command-to-actuator latency probe. Runs INSIDE the devkit container.

Steps the steering command between full left and full right at standstill and
times how long the /steering feedback takes to start moving, plus the sensor
rates. Measured 2026-09-11 (RTX 5090 host, Xvfb :99): feedback/LiDAR/IMU all
11.36 Hz; command -> first feedback movement median 0.135 s (0.090-0.180 s);
then the actuator slews at ~3.2 rad/s. The devkit bridge is request/response
(one command returned per simulator frame), so this dead time is structural.

Usage (sim connected, car stationary):
  docker cp src/tools/latency_probe.py autodrive_roboracer_api:/tmp/
  docker exec autodrive_roboracer_api bash -c \
      "source /opt/ros/humble/setup.bash && python3 /tmp/latency_probe.py"
"""
import time, statistics, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu

class Probe(Node):
    def __init__(self):
        super().__init__('latency_probe')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.pub = self.create_publisher(Float32, f'{ns}/steering_command', 1)
        self.pub_t = self.create_publisher(Float32, f'{ns}/throttle_command', 1)
        self.create_subscription(Float32, f'{ns}/steering', self.on_fb, qos)
        self.create_subscription(LaserScan, f'{ns}/lidar', self.on_scan, qos)
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.fb = None; self.fb_t = None; self.fb_series = []
        self.scan_t = []; self.imu_t = []
        self.target = 0.0; self.t_cmd = None
        self.lat_first = []; self.lat_settle = []
        self.first_seen = False
    def on_fb(self, m):
        t = time.monotonic()
        self.fb_series.append((t, m.data))
        if self.t_cmd is not None:
            if not self.first_seen and abs(m.data - self.prev_fb) > 0.02 and \
               (m.data - self.prev_fb) * (self.target - self.prev_fb) > 0:
                self.first_seen = True; self.lat_first.append(t - self.t_cmd)
            if abs(m.data - self.target) < 0.02 and self.settle_t is None:
                self.settle_t = t; self.lat_settle.append(t - self.t_cmd)
        self.fb = m.data
    def on_scan(self, m): self.scan_t.append(time.monotonic())
    def on_imu(self, m): self.imu_t.append(time.monotonic())

def rate(ts):
    if len(ts) < 3: return float('nan')
    d = [b - a for a, b in zip(ts, ts[1:])]
    return 1.0 / statistics.median(d)

def main():
    rclpy.init(); n = Probe()
    n.pub_t.publish(Float32(data=0.0))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0: rclpy.spin_once(n, timeout_sec=0.01)
    steps = 12
    for i in range(steps):
        n.target = 1.0 if i % 2 == 0 else -1.0
        n.prev_fb = n.fb if n.fb is not None else 0.0
        n.first_seen = False; n.settle_t = None
        n.t_cmd = time.monotonic()
        n.pub.publish(Float32(data=n.target))
        while time.monotonic() - n.t_cmd < 1.2: rclpy.spin_once(n, timeout_sec=0.005)
    n.t_cmd = None
    n.pub.publish(Float32(data=0.0))
    for _ in range(50): rclpy.spin_once(n, timeout_sec=0.01)
    print(f'feedback topic rate {rate([t for t,_ in n.fb_series]):.2f} Hz, lidar {rate(n.scan_t):.2f} Hz, imu {rate(n.imu_t):.2f} Hz')
    print('cmd -> first movement (s):', ' '.join(f'{x:.3f}' for x in n.lat_first))
    print('cmd -> settled at target (s):', ' '.join(f'{x:.3f}' for x in n.lat_settle))
    if n.lat_first: print(f'first-move median {statistics.median(n.lat_first):.3f} min {min(n.lat_first):.3f} max {max(n.lat_first):.3f}')
    if n.lat_settle: print(f'settle median {statistics.median(n.lat_settle):.3f}')
    # steering rate: slope of feedback across a full sweep -1 -> 1
    fs = n.fb_series
    seg = [(t, v) for t, v in fs]
    # find largest monotone run
    best = 0; bestslope = 0
    i = 0
    while i < len(seg) - 1:
        j = i
        while j + 1 < len(seg) and (seg[j+1][1] - seg[j][1]) * (seg[i+1][1] - seg[i][1]) > 0: j += 1
        if j - i >= 2 and abs(seg[j][1] - seg[i][1]) > 0.5:
            sl = abs(seg[j][1] - seg[i][1]) / (seg[j][0] - seg[i][0])
            if abs(seg[j][1]-seg[i][1]) > best: best = abs(seg[j][1]-seg[i][1]); bestslope = sl
        i = j + 1
    print(f'largest monotone sweep {best:.2f} (normalized) at {bestslope:.2f} /s = {bestslope*0.5236:.2f} rad/s')
    print('raw feedback samples after a step (t rel cmd, value) for last step:')
    last = [(t - fs[0][0], v) for t, v in fs[-25:]]
    print(' '.join(f'{t:.2f}:{v:.2f}' for t, v in last))
    rclpy.shutdown()
main()
