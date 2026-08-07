#!/bin/bash
# The finals drill: take a track we have never seen and produce a tuned,
# collision-free racing stack for it. Every step is timed so we know whether
# it fits inside the 3-day window between the track release and the deadline.
#
# Usage: ./scripts/new_track_drill.sh <track_name> [sim_tag] [record_seconds]
#
# Stages
#   1. reconnaissance  drive the unknown track with Follow-the-Gap and record
#   2. map             occupancy grid from the recording
#   3. centerline      corridor -> skeleton -> ordered centerline + widths
#   4. lines           optimized raceline + safe centerline line
#   5. verify          footprint clearance check before ever driving it
# Then run it with: ./scripts/run_pp_only.sh <scale> <track_name> safe
set +e
cd "$(dirname "$0")/.."
TRACK="${1:?usage: new_track_drill.sh <track_name> [sim_tag] [record_seconds]}"
TAG="${2:-2026-iros-practice}"
RECORD_S="${3:-150}"
T0=$(date +%s)
stage() { echo ""; echo "=== [$(( $(date +%s) - T0 ))s] $* ==="; }

stage "1/5 reconnaissance: driving $TRACK blind with Follow-the-Gap for ${RECORD_S}s"
TAG="$TAG" ./scripts/sim_restart_connect.sh || exit 1
R="source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"
docker exec autodrive_roboracer_api /bin/bash -c \
    "$R && cd /home/autodrive_devkit && colcon build --packages-select pit_crew --symlink-install 2>&1|tail -1"
BAG="recon_${TRACK}_$(date +%H%M%S)"
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew follow_the_gap \
    --ros-args --params-file /home/autodrive_devkit/src/pit_crew/config/ftg_mapping.yaml > /tmp/ftg.log 2>&1"
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 bag record -o /home/data/bags/${BAG} \
    /autodrive/roboracer_1/lidar /autodrive/roboracer_1/imu \
    /autodrive/roboracer_1/left_encoder /autodrive/roboracer_1/right_encoder \
    /autodrive/roboracer_1/throttle /autodrive/roboracer_1/steering \
    /autodrive/roboracer_1/ips > /tmp/bag.log 2>&1"
sleep "$RECORD_S"
docker exec autodrive_roboracer_api /bin/bash -c "pkill -INT -f '[r]os2 bag record'; sleep 4; true"
./scripts/stop_run.sh > /dev/null
docker exec autodrive_roboracer_api /bin/bash -c \
    "$R && ros2 bag reindex /home/data/bags/${BAG} > /dev/null 2>&1; true"

stage "2/5 map"
docker exec autodrive_roboracer_api /bin/bash -c "$R && python3 \
    /home/autodrive_devkit/src/pit_crew/tools/build_map_from_bag.py \
    /home/data/bags/${BAG} /home/data/maps/${TRACK}" || exit 1

stage "3/5 centerline"
.venv/bin/python tools/extract_centerline.py \
    "data/maps/${TRACK}" "data/racelines/${TRACK}_centerline" || exit 1

stage "4/5 racing lines"
.venv/bin/python tools/optimize_raceline.py \
    "data/racelines/${TRACK}_centerline.csv" --out "data/racelines/${TRACK}_raceline"
.venv/bin/python tools/optimize_raceline.py \
    "data/racelines/${TRACK}_centerline.csv" --centerline-only \
    --out "data/racelines/${TRACK}_safe" || exit 1

stage "5/5 footprint clearance check"
.venv/bin/python tools/footprint_analysis.py "data/maps/${TRACK}" \
    "data/maps/${TRACK}_traj.csv"

stage "DRILL COMPLETE in $(( $(date +%s) - T0 ))s"
echo "Next: ./scripts/run_pp_only.sh 0.30 ${TRACK} safe"
