#!/bin/bash
# Measure what a NEGATIVE throttle actually does at speed, on the open floor.
#
# The only brake the stack owns is the exact float 0.0, which locks all four
# wheels for a held ~4.55 m/s2 -- past the tyre's longitudinal peak of ~7.1.
# A negative command is reverse DRIVE torque, which on a wheel still rolling
# forward may retard the car proportionally and WITHOUT locking. August's
# disaster with `brake_gain` 0.3 was measured against a broken speed estimate
# and mostly at walking pace, so it does not answer this. `brake_probe` does.
#
#   ./scripts/brake_probe.sh                     # Xvfb :99 (~0.09-0.10 s)
#   VISIBLE=1 ./scripts/brake_probe.sh           # the real display (~0.058 s)
#
# THE IMAGE MATTERS. This needs the `-explore` world, which has no track at
# all, only `Floor` -- the same world `sysid_run` uses. Pointed at a circuit
# the car brakes into a wall and the measurement is scrap (`skidpad.py` learned
# that the expensive way).
#
# Env:
#   TAG          sim image tag, default 2026-iros-explore -- do not point this
#                at a track tag without reading the paragraph above
#   U_BRAKES     the sweep, e.g. "[0.0,-0.1,-0.2]"; 0.0 is the locked-brake
#                reference and every table is read against it, so keep it in
#   V_TARGET     entry speed for each brake window, m/s (node default 7.0)
#   REPEATS      passes over the list
#   RUN_S        wall-clock cap, default 400
#   PROBE_ARGS   extra "-p name:=value" passed verbatim
#
# Safety lives in the node, not here: a geofence around the spawn point, an
# ay-limited drive home between trials, a yaw-rate abort while it is supposed
# to be going straight, an IPS-dropout abort, and actuators zeroed on exit --
# which is itself the full brake, because zero throttle IS the brake.
set -u
cd "$(dirname "$0")/.."
TAG="${TAG:-2026-iros-explore}"
RUN_S="${RUN_S:-400}"
mkdir -p data/logs

TAG="$TAG" ./scripts/sim_restart_connect.sh || exit 1

R="export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"
# sim_restart_connect.sh restarts the devkit container, which wipes the colcon
# build: rebuild or the node is simply not there.
docker exec autodrive_roboracer_api /bin/bash -c \
    "$R && cd /home/autodrive_devkit && colcon build --packages-select pit_crew --symlink-install 2>&1|tail -1"

OPT=""
[ -n "${U_BRAKES:-}" ] && OPT="$OPT -p u_brakes:=${U_BRAKES}"
[ -n "${V_TARGET:-}" ] && OPT="$OPT -p v_target:=$(printf '%.3f' "$V_TARGET")"
[ -n "${REPEATS:-}" ]  && OPT="$OPT -p repeats:=${REPEATS}"
docker exec -d autodrive_roboracer_api /bin/bash -c \
    "$R && ros2 run pit_crew brake_probe --ros-args $OPT \
    -p max_run_s:=$(printf '%.1f' "$RUN_S") ${PROBE_ARGS:-} > /tmp/brake.log 2>&1"
sleep 5
docker exec autodrive_roboracer_api /bin/bash -c \
    "grep -hE 'brake_probe:|Traceback|Error|ERROR' /tmp/brake.log 2>/dev/null | head -8"

echo "probing for up to ${RUN_S}s ..."
# Poll rather than sleep the whole window: the node stops itself when the
# sweep is done, and a finished sweep should not cost another five minutes.
for _ in $(seq 1 "$((RUN_S / 5 + 1))"); do
    sleep 5
    docker exec autodrive_roboracer_api /bin/bash -c \
        "pgrep -f '[i]nstall/pit_crew/lib/pit_crew/brake_probe' >/dev/null" || break
done
# SIGINT first so the node runs its own shutdown (summary table, the
# "# STOPPED:" trailer, zeroed actuators); stop_run.sh sends SIGTERM, which
# Python takes without running `finally`.
docker exec autodrive_roboracer_api /bin/bash -c \
    "pkill -INT -f '[i]nstall/pit_crew/lib/pit_crew/brake_probe' || true"
sleep 3
./scripts/stop_run.sh > /dev/null 2>&1

LOG=$(ls -t data/logs/brake_*.csv 2>/dev/null | head -1)
if [ -n "$LOG" ]; then NODELOG="${LOG%.csv}.log"
else NODELOG="data/logs/brake_$(date +%Y%m%d_%H%M%S).log"; fi
# The container's /tmp is wiped by the next devkit restart; keep the summary.
docker cp autodrive_roboracer_api:/tmp/brake.log "$NODELOG" 2>/dev/null \
    || echo "(could not copy /tmp/brake.log out of the container)"

echo "--- summary ---"
sed -n '/brake_probe summary/,$p' "$NODELOG" 2>/dev/null | sed 's/^/  /'
grep -hE 'ABORTED|Traceback|Error|ERROR' "$NODELOG" 2>/dev/null | head -5
[ -n "$LOG" ] && echo "log: $LOG" && echo "node log: $NODELOG"
[ -z "$LOG" ] && { echo "no CSV written; see $NODELOG"; exit 1; }
exit 0
