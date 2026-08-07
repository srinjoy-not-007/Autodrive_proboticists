#!/bin/bash
# Localization + pure pursuit driving the car directly (no arbiter).
#
# The arbiter is deliberately NOT in this path: measured, it caused the
# collisions it was meant to prevent (see docs/STATUS.md). This is the
# configuration that laps cleanly.
# Usage: ./scripts/run_pp_only.sh [speed_scale] [map] [line]
set +e
cd "$(dirname "$0")/.."
SCALE="${1:-0.30}"; MAP="${2:-track_v2}"; LINE="${3:-safe}"
INIT=$(head -2 data/maps/${MAP}_traj.csv | tail -1 | cut -d, -f2,3)
IX=$(echo "$INIT" | cut -d, -f1); IY=$(echo "$INIT" | cut -d, -f2)
echo "map=$MAP line=$LINE scale=$SCALE init=($IX,$IY)"

./scripts/sim_restart_connect.sh || exit 1
R="source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"
docker exec autodrive_roboracer_api /bin/bash -c \
    "$R && cd /home/autodrive_devkit && colcon build --packages-select pit_crew --symlink-install 2>&1|tail -1"
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew localization --ros-args \
    -p map_prefix:=/home/data/maps/${MAP} \
    -p centerline_csv:=/home/data/racelines/${MAP}_centerline.csv \
    -p init_x:=${IX} -p init_y:=${IY} > /tmp/loc.log 2>&1"
sleep 7
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew pure_pursuit --ros-args \
    -p raceline_csv:=/home/data/racelines/${MAP}_${LINE}.csv \
    -p speed_scale:=${SCALE} > /tmp/pp.log 2>&1"
docker exec -d autodrive_roboracer_api /bin/bash -c \
    "$R && python3 /home/autodrive_devkit/src/pit_crew/tools/race_logger.py > /tmp/logger.log 2>&1"
sleep 5
docker exec autodrive_roboracer_api /bin/bash -c "grep -hE 'ready|Traceback' /tmp/loc.log /tmp/pp.log|head -4"
echo "Running. Stop with ./scripts/stop_run.sh"
