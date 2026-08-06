#!/bin/bash
# End-to-end smoke test: headless sim + headless devkit bridge, verify LiDAR flows
# and the vehicle accepts throttle commands.
set -e
cd "$(dirname "$0")/.."

./scripts/run_sim.sh headless
./scripts/run_devkit.sh headless
echo "Waiting 15 s for sim + bridge to connect..."
sleep 15

echo "--- Topics ---"
docker exec autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && ros2 topic list"

echo "--- LiDAR rate (expect ~40 Hz) ---"
docker exec autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /autodrive/roboracer_1/lidar --window 50" || true

echo "--- Sending throttle 0.1 for 3 s, reading encoder ---"
docker exec autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic pub -r 10 /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 '{data: 0.1}'" || true
docker exec autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo --once /autodrive/roboracer_1/left_encoder" || true

echo "--- Cleaning up ---"
docker rm -f autodrive_roboracer_sim autodrive_roboracer_api
echo "SMOKE TEST DONE"
