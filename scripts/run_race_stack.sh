#!/bin/bash
# Run the full race stack: localization + pure pursuit (+ race logger).
# Usage: ./scripts/run_race_stack.sh [speed_scale] [map_name]
#
# Nodes are started detached so a later `ros2 bag play`/CLI signal cannot take
# them down with it.
set +e
cd "$(dirname "$0")/.."
SCALE="${1:-0.6}"
MAP="${2:-track_v2}"

INIT=$(head -2 data/maps/${MAP}_traj.csv | tail -1 | cut -d, -f2,3)
IX=$(echo "$INIT" | cut -d, -f1)
IY=$(echo "$INIT" | cut -d, -f2)
echo "map=$MAP  init=($IX,$IY)  speed_scale=$SCALE"

./scripts/sim_restart_connect.sh || exit 1

R="source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"
docker exec autodrive_roboracer_api /bin/bash -c \
    "$R && cd /home/autodrive_devkit && colcon build --packages-select pit_crew --symlink-install >/dev/null 2>&1"

docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew localization --ros-args \
    -p map_prefix:=/home/data/maps/${MAP} \
    -p centerline_csv:=/home/data/racelines/${MAP}_centerline.csv \
    -p init_x:=${IX} -p init_y:=${IY} > /tmp/loc.log 2>&1"
sleep 6
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew pure_pursuit --ros-args \
    -p raceline_csv:=/home/data/racelines/${MAP}_raceline.csv \
    -p speed_scale:=${SCALE} > /tmp/pp.log 2>&1"
docker exec -d autodrive_roboracer_api /bin/bash -c \
    "$R && python3 /home/autodrive_devkit/src/pit_crew/tools/race_logger.py > /tmp/logger.log 2>&1"
sleep 4
docker exec autodrive_roboracer_api /bin/bash -c "grep -h -E 'INFO|ERROR|Traceback' /tmp/loc.log /tmp/pp.log | head -6"
echo "Race stack running. Stop with ./scripts/stop_run.sh"
