#!/bin/bash
# Record a slow, clean FTG traversal for SLAM mapping.
# Restarts sim + devkit clean, launches FTG (mapping profile) + race logger +
# rosbag of all sensor topics. Stop with: ./scripts/stop_run.sh
set -e
cd "$(dirname "$0")/.."

./scripts/sim_restart_connect.sh

docker exec autodrive_roboracer_api /bin/bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /home/autodrive_devkit &&
    colcon build --packages-select pit_crew --symlink-install >/dev/null 2>&1 &&
    source install/setup.bash &&
    mkdir -p /home/data/bags &&
    (nohup ros2 run pit_crew follow_the_gap --ros-args \
        --params-file /home/autodrive_devkit/src/pit_crew/config/ftg_mapping.yaml \
        > /tmp/ftg.log 2>&1 &) &&
    (nohup python3 /home/autodrive_devkit/src/pit_crew/tools/race_logger.py \
        > /tmp/logger.log 2>&1 &) &&
    (nohup ros2 bag record -o /home/data/bags/mapping_\$(date +%Y%m%d_%H%M%S) \
        /autodrive/roboracer_1/lidar \
        /autodrive/roboracer_1/imu \
        /autodrive/roboracer_1/left_encoder \
        /autodrive/roboracer_1/right_encoder \
        /autodrive/roboracer_1/steering \
        /autodrive/roboracer_1/throttle \
        /autodrive/roboracer_1/ips \
        > /tmp/bag.log 2>&1 &) &&
    sleep 5 && grep -h INFO /tmp/ftg.log /tmp/bag.log | head -5"
echo "Mapping run recording. Watch collisions; stop with scripts/stop_run.sh"
