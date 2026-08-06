#!/bin/bash
# Offline localization test: replay a bag through the particle filter and
# compare against the IPS ground truth recorded in that bag.
# Usage: ./scripts/test_localization.sh [bag_dir_name] [rate] [extra ros args...]
#
# Each stage is its own `docker exec` on purpose: `ros2 bag play` signals its
# process group when it finishes, which SIGTERMs any node started from the
# same shell. Detached (-d) nodes are children of container init and survive.
set +e
cd "$(dirname "$0")/.."
BAG="${1:-mapping_20260806_161512}"
RATE="${2:-1.0}"
shift 2 2>/dev/null || true
EXTRA="$*"

INIT=$(head -2 data/maps/track_v1_traj.csv | tail -1 | cut -d, -f2,3)
IX=$(echo "$INIT" | cut -d, -f1)
IY=$(echo "$INIT" | cut -d, -f2)
echo "init pose: ($IX, $IY)   bag: $BAG   rate: $RATE   extra: $EXTRA"

R="source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"

# Clean up stragglers from previous runs (bracketed patterns so the pattern
# cannot match this wrapper's own command line)
docker exec autodrive_roboracer_api /bin/bash -c \
    "pkill -f '[i]nstall/pit_crew/lib' 2>/dev/null; pkill -f '[e]val_localization' 2>/dev/null; true"

docker exec -d autodrive_roboracer_api /bin/bash -c \
    "$R && ros2 run pit_crew localization --ros-args -p init_x:=${IX} -p init_y:=${IY} ${EXTRA} > /tmp/loc.log 2>&1"
docker exec -d autodrive_roboracer_api /bin/bash -c \
    "$R && python3 /home/autodrive_devkit/src/pit_crew/tools/eval_localization.py /home/data/logs/loc_eval_${BAG}.csv > /tmp/loceval.log 2>&1"
sleep 6

echo "replaying bag..."
docker exec autodrive_roboracer_api /bin/bash -c \
    "$R && ros2 bag play /home/data/bags/${BAG} --rate ${RATE} > /dev/null 2>&1; true"

sleep 2
docker exec autodrive_roboracer_api /bin/bash -c \
    "pkill -INT -f '[e]val_localization'; sleep 3; pkill -f '[i]nstall/pit_crew/lib'; true"
docker exec autodrive_roboracer_api /bin/bash -c \
    "cat /tmp/loceval.log; echo '--- localization node log (last 3) ---'; tail -3 /tmp/loc.log"
