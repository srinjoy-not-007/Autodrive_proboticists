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
#   V_PROBE          probe speed on the straights, m/s
#   AMP / PULSE_S    ramp height in rad and the hold duration
#   N_PULSES         trigger points per lap (Porto admits only 1, see below)
#                    -- all three are passed ONLY if set; unset means the
#                    node's own default, so the two can never drift apart
#                    again (the first live run got v_probe 3.0 from here
#                    against the node's 2.5, which resized every window)
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

OPT=""
[ -n "${V_PROBE:-}" ]  && OPT="$OPT -p v_probe:=$(printf '%.3f' "$V_PROBE")"
[ -n "${AMP:-}" ]      && OPT="$OPT -p amp_rad:=$(printf '%.3f' "$AMP")"
[ -n "${PULSE_S:-}" ]  && OPT="$OPT -p pulse_s:=$(printf '%.3f' "$PULSE_S")"
[ -n "${N_PULSES:-}" ] && OPT="$OPT -p n_pulses:=${N_PULSES}"
docker exec -d autodrive_roboracer_api /bin/bash -c "$R && ros2 run pit_crew turnin_probe --ros-args \
    -p raceline_csv:=/home/data/racelines/${MAP}_${LINE}.csv \
    -p map_prefix:=/home/data/maps/${MAP} \
    $POSE_ARG $OPT \
    -p max_run_s:=$(printf '%.1f' "$RUN_S") ${PROBE_ARGS:-} > /tmp/turnin.log 2>&1"
sleep 6
docker exec autodrive_roboracer_api /bin/bash -c \
    "grep -hE 'turnin_probe:|Traceback|Error|ERROR' /tmp/turnin.log /tmp/loc.log 2>/dev/null | head -8"

echo "probing for ${RUN_S}s ..."
sleep "$RUN_S"
# SIGINT first so the node runs its own shutdown (summary line, the
# "# STOPPED:" trailer in the CSV, zeroed actuators) -- stop_run.sh sends
# SIGTERM, which Python takes without running `finally`.
docker exec autodrive_roboracer_api /bin/bash -c \
    "pkill -INT -f '[i]nstall/pit_crew/lib/pit_crew/turnin_probe' || true"
sleep 3
./scripts/stop_run.sh > /dev/null 2>&1

LOG=$(ls -t data/logs/turnin_*.csv 2>/dev/null | head -1)
# Preserve the node's stdout NOW.  The container's /tmp is wiped by the next
# devkit restart, and the first live run's reason for stopping was lost that
# way; the copy lands next to the CSV on the host mount.
if [ -n "$LOG" ]; then
    NODELOG="${LOG%.csv}.log"
else
    NODELOG="data/logs/turnin_$(date +%Y%m%d_%H%M%S).log"
fi
docker cp autodrive_roboracer_api:/tmp/turnin.log "$NODELOG" 2>/dev/null \
    || echo "(could not copy /tmp/turnin.log out of the container)"
echo "node log: $NODELOG"

echo "--- why it stopped ---"
{ grep -hE 'STOPPED|ABORT|Traceback|Error|ERROR|turnin_probe:|pulse [0-9]+:' "$NODELOG" 2>/dev/null | tail -12
  grep -h '^# STOPPED' "$LOG" 2>/dev/null; } | sed 's/^/  /'
[ -n "$LOG" ] && echo "  pulses fired: $(grep -c '^pose,.*,pulse,' "$LOG" 2>/dev/null || echo 0) pulse-phase rows"

if [ -z "$LOG" ]; then echo "no CSV written; see $NODELOG"; exit 1; fi
echo "log: $LOG"
.venv/bin/python tools/turnin_fit.py "$LOG" --per-pulse
