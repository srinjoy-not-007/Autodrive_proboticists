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

# Stages 2-4 used to call extract_centerline and optimize_raceline directly with
# their DEFAULT parameters -- no centerline-method selection, no drivability
# gate, and the stock ay 3.0 / ax 3.0 profile rather than the validated one. A
# finals recon would have produced a slower, ungated line than the pipeline we
# actually race. They now go through make_track_lines.py, which owns all of it.
stage "2/5 map (both sensor origins)"
# The LiDAR sits 0.1933 m ahead of the pose point. Correcting for it sharpens
# the map -- Porto wall pixels 6.53% -> 4.66% -- and was worth 1.6 s a race
# there. On ICRA the sharper map yields a line that fails the steering-lock
# gate. Which one wins is a property of the track, so build both and let the
# gate decide rather than guessing three days before a deadline.
for VARIANT in "" "_c"; do
    OFF=0.0; [ -n "$VARIANT" ] && OFF=0.1933
    docker exec autodrive_roboracer_api /bin/bash -c \
        "$R && PIT_CREW_LIDAR_OFFSET=$OFF python3 \
         /home/autodrive_devkit/src/pit_crew/tools/build_map_from_bag.py \
         /home/data/bags/${BAG} /home/data/maps/${TRACK}${VARIANT}" || exit 1
done

stage "3/5 racing lines from each map, keep whichever is drivable and faster"
BEST=""; BEST_T=99999
for VARIANT in "" "_c"; do
    OUT=$(.venv/bin/python tools/make_track_lines.py "${TRACK}${VARIANT}" 2>&1)
    echo "$OUT" | sed "s/^/  [${TRACK}${VARIANT}] /"
    T=$(echo "$OUT" | sed -n 's/.*race line: .* ideal \([0-9.]*\) s.*/\1/p' | tail -1)
    if [ -n "$T" ] && awk "BEGIN{exit !($T < $BEST_T)}"; then BEST="$VARIANT"; BEST_T="$T"; fi
done
[ -z "$BEST$BEST_T" ] && { echo "no drivable line from either map"; exit 1; }
if [ "$BEST_T" = "99999" ]; then echo "no drivable line from either map"; exit 1; fi
echo "chosen map: ${TRACK}${BEST:-" (uncorrected)"}  ideal ${BEST_T} s"

stage "4/5 promote the winner to the canonical names"
if [ -n "$BEST" ]; then
    docker exec autodrive_roboracer_api /bin/bash -c \
        "for e in pgm yaml; do cp /home/data/maps/${TRACK}${BEST}.\$e /home/data/maps/${TRACK}.\$e; done; \
         cp /home/data/maps/${TRACK}${BEST}_traj.csv /home/data/maps/${TRACK}_traj.csv" || exit 1
    for f in centerline safe raceline race; do
        cp "data/racelines/${TRACK}${BEST}_${f}.csv" "data/racelines/${TRACK}_${f}.csv" 2>/dev/null
    done
fi

stage "5/5 footprint clearance check"
.venv/bin/python tools/footprint_analysis.py "data/maps/${TRACK}" \
    "data/maps/${TRACK}_traj.csv"

stage "DRILL COMPLETE in $(( $(date +%s) - T0 ))s"
echo "Next: ./scripts/run_pp_only.sh 0.30 ${TRACK} safe"
