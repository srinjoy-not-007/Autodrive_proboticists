#!/bin/bash
# Measure the pose-stamp-to-yaw-rate delay IN THE RACE LOOP, at whatever
# sensor period this start of the simulator happens to give.
#
#   VISIBLE=1 ./scripts/turnin_probe.sh          # the real display (~0.058 s)
#             ./scripts/turnin_probe.sh          # Xvfb :99 (~0.095-0.104 s)
#
# Then fit, and compare the two:
#   .venv/bin/python tools/turnin_fit.py data/logs/turnin_<ts>.csv --per-pulse
#   .venv/bin/python tools/turnin_fit.py --compare <display.csv> <xvfb.csv>
#
# Env:
#   VISIBLE=1        run on $DISPLAY instead of Xvfb (steals the pointer)
#   SIM_QUALITY      'ultra' reproduces the organisers' setting
#   MAP / LINE       default porto_mesh / safe -- NOT the shipping min-curvature
#                    line, which measured 0.3 m of admissible straight on the
#                    whole lap because it hugs the inside; the safe line has 4.9 m
#   V_PROBE          probe speed on the straights, m/s (default 3.0)
#   AMP / PULSE_S    ramp height in rad and the hold duration (default 0.15 / 0.65)
#   N_PULSES         trigger points per lap (default 3)
#   RUN_S            wall-clock length of the run (default 240)
#   PROBE_ARGS       extra "-p name:=value" passed verbatim to the node
#   NO_LOC=1         skip localization; trigger straight off IPS instead
#                    (a control: shows whether localization carries the shift)
#
# Safety, in the node and not in this script: a pulse only fires on a stretch
# flatter than kappa_max with >= clear_min of corridor either side, with the
# car within cte_gate of the line and within v_gate of the probe speed; the
# step reverts to zero after pulse_s (or sooner past cte_abort); the run stops
# on lidar_min < lidar_stop_m; and the actuators are zeroed on exit, because
# commands latch.
set -u
cd "$(dirname "$0")/.."
MAP="${MAP:-porto_mesh}"; LINE="${LINE:-safe}"
RUN_S="${RUN_S:-240}"
LOAD=$(cut -d" " -f1 /proc/loadavg)
if [ "${LOAD_OK:-0}" != 1 ] && awk "BEGIN{exit !($LOAD > 12)}"; then
    echo "REFUSING: host load $LOAD; a timing measurement needs a quiet host"; exit 3
fi
mkdir -p data/logs

./scripts/sim_restart_connect.sh || exit 1
if [ "${SIM_QUALITY:-}" = "ultra" ]; then
    DISP="$([ "${VISIBLE:-0}" = 1 ] && echo "${DISPLAY:-:0}" || echo :99)" \
        ./scripts/sim_set_quality.sh ultra > /tmp/quality.log 2>&1 \
        || { echo "WARN: could not set Ultra"; tail -3 /tmp/quality.log; }
    sleep 2
fi

R="export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"
docker exec autodrive_roboracer_api /bin/bash -c \
    "$R && cd /home/autodrive_devkit && colcon build --packages-select pit_crew --symlink-install 2>&1|tail -1"

POSE_ARG="-p pose_topic:=/pit_crew/pose"
if [ "${NO_LOC:-0}" = 1 ]; then
    POSE_ARG="-p pose_topic:=ips"
else
    INIT=$(head -2 data/maps/${MAP}_traj.csv | tail -1 | cut -d, -f2,3)
    IX=$(echo "$INIT" | cut -d, -f1); IY=$(echo "$INIT" | cut -d, -f2)
    docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew localization --ros-args \
        -p map_prefix:=/home/data/maps/${MAP} \
        -p centerline_csv:=/home/data/racelines/${MAP}_centerline.csv \
        -p lidar_offset:=${LIDAR_OFFSET:-0.0} \
        -p init_x:=${IX} -p init_y:=${IY} ${LOC_ARGS:-} > /tmp/loc.log 2>&1"
    sleep 7
fi

docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew turnin_probe --ros-args \
    -p raceline_csv:=/home/data/racelines/${MAP}_${LINE}.csv \
    -p map_prefix:=/home/data/maps/${MAP} \
    $POSE_ARG \
    -p v_probe:=${V_PROBE:-3.0} -p amp_rad:=${AMP:-0.15} \
    -p pulse_s:=${PULSE_S:-0.65} -p n_pulses:=${N_PULSES:-3} \
    -p max_run_s:=${RUN_S} ${PROBE_ARGS:-} > /tmp/turnin.log 2>&1"
sleep 6
docker exec autodrive_roboracer_api /bin/bash -c \
    "grep -hE 'turnin_probe:|Traceback|Error|ERROR' /tmp/turnin.log /tmp/loc.log 2>/dev/null | head -8"

echo "probing for ${RUN_S}s ..."
sleep "$RUN_S"
./scripts/stop_run.sh > /dev/null 2>&1
docker exec autodrive_roboracer_api /bin/bash -c "grep -cE '^\[INFO\].*pulse ' /tmp/turnin.log" \
    | sed 's/^/pulses fired: /'

LOG=$(ls -t data/logs/turnin_*.csv 2>/dev/null | head -1)
if [ -z "$LOG" ]; then echo "no log written; see /tmp/turnin.log"; exit 1; fi
echo "log: $LOG"
.venv/bin/python tools/turnin_fit.py "$LOG"
