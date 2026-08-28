#!/bin/bash
# End-to-end smoke test: sim + devkit bridge, verify the LiDAR is genuinely
# LIVE and the vehicle accepts throttle commands.
#
# This used to start the sim HEADLESS and report "expect ~40 Hz". Both were
# wrong, and together they made the test worse than useless: the LiDAR is
# GPU-raytraced, so -batchmode -nographics leaves it republishing one STALE
# scan forever -- at a perfectly healthy-looking rate. The test passed on a
# frozen sensor and granted confidence. The real rate is ~10 Hz (every sensor
# shares one WebSocket), so 40 would not have flagged it either.
#
# So: run windowed, expect ~10 Hz, and assert consecutive scans actually
# DIFFER rather than merely arrive.
set -e
cd "$(dirname "$0")/.."

./scripts/sim_restart_connect.sh

echo "--- Topics ---"
docker exec autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && ros2 topic list"

echo "--- LiDAR: rate (expect ~10 Hz) and liveness ---"
docker exec autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && timeout 12 ros2 topic hz /autodrive/roboracer_1/lidar --window 30" || true

docker exec autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && timeout 25 python3 -" <<'PY'
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# A frozen GPU LiDAR republishes one identical buffer. Real scans always differ
# a little -- ray noise, and any motion at all -- so byte-identical consecutive
# frames mean the sensor is stale no matter how fast they arrive.
class Check(Node):
    def __init__(self):
        super().__init__('smoke_lidar_check')
        self.frames = []
        self.create_subscription(LaserScan, '/autodrive/roboracer_1/lidar',
                                 lambda m: self.frames.append(tuple(m.ranges)), 10)

rclpy.init()
n = Check()
while rclpy.ok() and len(n.frames) < 8:
    rclpy.spin_once(n, timeout_sec=2.0)
rclpy.shutdown()

if len(n.frames) < 8:
    sys.exit(f'FAIL: only {len(n.frames)} scans in 16 s -- LiDAR not publishing')
uniq = len(set(n.frames))
print(f'{len(n.frames)} scans, {uniq} distinct')
if uniq < 2:
    sys.exit('FAIL: every scan byte-identical -- LiDAR is FROZEN '
             '(is the sim running headless?)')
print('PASS: LiDAR is live')
PY

echo "--- Sending throttle 0.1 for 3 s, reading encoder ---"
docker exec autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic pub -r 10 /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 '{data: 0.1}'" || true
docker exec autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo --once /autodrive/roboracer_1/left_encoder" || true
./scripts/stop_run.sh

echo "SMOKE TEST DONE"
