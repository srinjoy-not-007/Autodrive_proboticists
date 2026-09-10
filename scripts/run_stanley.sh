#!/bin/bash
# Localization + a raceline tracker.
# Usage: ./scripts/run_stanley.sh [speed_scale] [map] [line] [extra ros args...]
#   CONTROLLER=mpc ./scripts/run_stanley.sh ...   to use MPC instead of Stanley
set +e
cd "$(dirname "$0")/.."
SCALE="${1:-0.30}"; MAP="${2:-track_v2}"; LINE="${3:-safe}"
shift 3 2>/dev/null || true
EXTRA="$*"
INIT=$(head -2 data/maps/${MAP}_traj.csv | tail -1 | cut -d, -f2,3)
IX=$(echo "$INIT" | cut -d, -f1); IY=$(echo "$INIT" | cut -d, -f2)
echo "map=$MAP line=$LINE scale=$SCALE $EXTRA"
./scripts/sim_restart_connect.sh || exit 1
R="source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"
docker exec autodrive_roboracer_api /bin/bash -c \
    "$R && cd /home/autodrive_devkit && colcon build --packages-select pit_crew --symlink-install 2>&1|tail -1"
# The LiDAR sits 0.1933 m ahead of the pose point. A map built with that
# correction MUST be localized against with the same offset -- the two have to
# agree or the sharper map makes things worse, not better (mismatched, total
# lateral p95 regresses to 0.370 m). new_track_drill.sh records the offset its
# promoted map was built with in data/maps/<map>.offset, so the right value is
# used without anyone having to remember; set LIDAR_OFFSET to override.
if [ -z "${LIDAR_OFFSET:-}" ] && [ -f "data/maps/${MAP}.offset" ]; then
    LIDAR_OFFSET=$(tr -d '[:space:]' < "data/maps/${MAP}.offset")
    echo "sensor origin from data/maps/${MAP}.offset: ${LIDAR_OFFSET} m"
fi
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew localization --ros-args \
    -p map_prefix:=/home/data/maps/${MAP} \
    -p centerline_csv:=/home/data/racelines/${MAP}_centerline.csv \
    -p lidar_offset:=${LIDAR_OFFSET:-0.0} \
    -p init_x:=${IX} -p init_y:=${IY} > /tmp/loc.log 2>&1"
sleep 7
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew ${CONTROLLER:-stanley} --ros-args \
    -p raceline_csv:=/home/data/racelines/${MAP}_${LINE}.csv \
    -p speed_scale:=${SCALE} ${EXTRA} > /tmp/stanley.log 2>&1"
docker exec -d autodrive_roboracer_api /bin/bash -c \
    "$R && python3 /home/autodrive_devkit/src/pit_crew/tools/race_logger.py > /tmp/logger.log 2>&1"
sleep 5
docker exec autodrive_roboracer_api /bin/bash -c "grep -hE 'ready|Traceback|Error' /tmp/loc.log /tmp/stanley.log|head -5"
echo "Running. Stop with ./scripts/stop_run.sh"
