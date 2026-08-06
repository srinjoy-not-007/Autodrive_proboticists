#!/bin/bash
# Build an occupancy map from a recorded mapping bag.
# Usage: ./scripts/build_map.sh <bag_dir_name_under_data/bags> [map_name]
# Track-agnostic: this is the first step of the new-track drill.
set -e
BAG="${1:?usage: build_map.sh <bag dir under data/bags> [map_name]}"
MAP="${2:-track}"

docker exec autodrive_roboracer_api /bin/bash -c "
    source /opt/ros/humble/setup.bash &&
    source /home/autodrive_devkit/install/setup.bash &&
    mkdir -p /home/data/maps &&
    pkill -f slam_toolbox 2>/dev/null; pkill -f 'pit_crew odometry' 2>/dev/null;
    (nohup ros2 launch pit_crew mapping_offline.launch.py > /tmp/slam.log 2>&1 &) &&
    sleep 6 &&
    ros2 bag play /home/data/bags/${BAG} --clock --rate 1.0 >/dev/null 2>&1 &&
    sleep 3 &&
    ros2 run nav2_map_server map_saver_cli -f /home/data/maps/${MAP} \
        --ros-args -p save_map_timeout:=10.0 &&
    pkill -f slam_toolbox; pkill -f mapping_offline; true"
echo "Map saved to data/maps/${MAP}.pgm/.yaml"
