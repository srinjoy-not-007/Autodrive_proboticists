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
# The simulator's telemetry period is a per-start lottery on this host
# (0.088-0.091 s on some starts, 0.103-0.106 s on others, stable within a
# run). RATE_MAX_DT / RATE_MIN_DT select an instance in the wanted band by
# restarting the sim (up to RATE_TRIES times) -- e.g. RATE_MAX_DT=0.095 for
# a fast (11 Hz) instance, RATE_MIN_DT=0.100 for a slow one.
if [ -n "${RATE_PRE_SELECT:-}" ]; then   # superseded by run_exp.sh (measures with the stack running)
    for try in $(seq 1 "${RATE_TRIES:-5}"); do
        DT=$(docker exec autodrive_roboracer_api /bin/bash -c "source /opt/ros/humble/setup.bash; timeout 8 ros2 topic hz /autodrive/roboracer_1/imu 2>&1 | grep average | tail -1" | awk '{printf "%.4f", 1.0/$3}')
        echo "sensor period ${DT:-?} s (try $try)"
        OK=1
        [ -n "${RATE_MAX_DT:-}" ] && awk "BEGIN{exit !(${DT:-1} > $RATE_MAX_DT)}" && OK=0
        [ -n "${RATE_MIN_DT:-}" ] && awk "BEGIN{exit !(${DT:-0} < $RATE_MIN_DT)}" && OK=0
        [ "$OK" = 1 ] && break
        ./scripts/sim_restart_connect.sh || exit 1
    done
fi
# SIM_QUALITY=ultra reproduces the organisers' setting (they run Ultra); the
# sensor cadence differs by quality, and the tracker is sensitive to it.
if [ "${SIM_QUALITY:-}" = "ultra" ] && [ "${MANUAL_CONNECT:-0}" != 1 ]; then DISP="$([ "${VISIBLE:-0}" = 1 ] && echo "${DISPLAY:-:0}" || echo :99)" ./scripts/sim_set_quality.sh ultra > /tmp/quality.log 2>&1 || { echo "WARN: could not set Ultra"; tail -3 /tmp/quality.log; }; sleep 2; fi
R="export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"
docker exec autodrive_roboracer_api /bin/bash -c \
    "$R && cd /home/autodrive_devkit && colcon build --packages-select pit_crew --symlink-install 2>&1|tail -1"
# The LiDAR sits 0.1933 m ahead of the pose point. A map built with that
# correction MUST be localized against with the same offset -- the two have to
# agree or the sharper map makes things worse, not better. Set
# LIDAR_OFFSET=0.1933 when racing a *_c (corrected) map. LOC_ARGS="-p k:=v"
# passes extra parameters to localization (the positional extras go to the
# controller only).
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew localization --ros-args \
    -p map_prefix:=/home/data/maps/${MAP} \
    -p centerline_csv:=/home/data/racelines/${MAP}_centerline.csv \
    -p lidar_offset:=${LIDAR_OFFSET:-0.0} \
    -p init_x:=${IX} -p init_y:=${IY} ${LOC_ARGS:-} > /tmp/loc.log 2>&1"
sleep 7
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew ${CONTROLLER:-stanley} --ros-args \
    -p raceline_csv:=/home/data/racelines/${MAP}_${LINE}.csv \
    -p speed_scale:=${SCALE} ${EXTRA} > /tmp/stanley.log 2>&1"
docker exec -d autodrive_roboracer_api /bin/bash -c \
    "$R && python3 /home/autodrive_devkit/src/pit_crew/tools/race_logger.py > /tmp/logger.log 2>&1"
sleep 5
docker exec autodrive_roboracer_api /bin/bash -c "grep -hE 'ready|Traceback|Error' /tmp/loc.log /tmp/stanley.log|head -5"
echo "Running. Stop with ./scripts/stop_run.sh"
