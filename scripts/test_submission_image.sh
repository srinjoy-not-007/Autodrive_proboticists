#!/bin/bash
# Race the SUBMISSION IMAGE exactly as the organisers run it: no source or
# data mounts, the image's own entrypoint, the official docker flags. A helper
# container (the dev image) does what the organisers do by hand -- clicks
# Connection -- and records the evaluation log.
#
# VISIBLE=1 races the same image on THIS HOST'S REAL DISPLAY instead of Xvfb --
# the organisers' own setup, where the telemetry period is 0.058 s rather than
# Xvfb's 0.088-0.106 s, and where four configurations have failed that were
# clean on Xvfb (docs/STATUS.md, Sept 12). It is the only way to run Gate D
# against the BUILT IMAGE rather than mounted source, and it steals the pointer
# to click Connection: do not touch the machine while it runs.
#
# Usage: ./scripts/test_submission_image.sh <image:tag> [laps] [sim_tag]
#        VISIBLE=1 ./scripts/test_submission_image.sh <image:tag> 32
set -u
cd "$(dirname "$0")/.."
. "$(dirname "$0")/lib_display.sh"   # display_select / click / canvas geometry
IMAGE="${1:?image:tag}"; LAPS="${2:-12}"; TAG="${3:-2026-iros-practice}"
REPO_DIR="$(pwd)"; GEOM="1920x1080"
display_select                       # -> DISP (":99", or $DISPLAY if VISIBLE=1)
R="source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"

# The rules' own preamble. Harmless if there is no X server to talk to, and
# without it the organisers' `-v /tmp/.X11-unix --env DISPLAY` container is
# refused by the host's X access control.
xhost local:root >/dev/null 2>&1 || true

docker rm -f autodrive_roboracer_api autodrive_roboracer_sim pit_helper >/dev/null 2>&1
# Helper: dev image with the repo mounted, used only for xdotool + the logger.
docker run --name pit_helper --rm -d --network=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "${REPO_DIR}/src:/home/autodrive_devkit/src/pit_crew" \
    -v "${REPO_DIR}/data:/home/data" --privileged \
    "autodriveecosystem/autodrive_roboracer_api:${TAG}" -c "sleep 7200" >/dev/null
docker exec pit_helper /bin/bash -c "which xdotool >/dev/null 2>&1 || (apt-get update -qq && apt-get install -y -qq xdotool) >/dev/null 2>&1; pip3 install -q 'numpy<2' 'scipy==1.10.1' >/dev/null 2>&1; true"

# Simulator on Xvfb :99 (see sim_restart_connect.sh for why), or on the host's
# real display under VISIBLE=1, where there is already an X server to join.
if [ "$DISP" = ":99" ]; then
    SIM_CMD="Xvfb :99 -screen 0 ${GEOM}x24 >/dev/null 2>&1 & sleep 3; cd /home/autodrive_simulator &&
        DISPLAY=:99 ./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567 -logFile /home/unity.log; sleep 3600"
else
    SIM_CMD="cd /home/autodrive_simulator &&
        DISPLAY=${DISP} ./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567 -logFile /home/unity.log; sleep 3600"
fi
docker run --name autodrive_roboracer_sim --rm -d --network=host --ipc=host --privileged --gpus all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw "autodriveecosystem/autodrive_roboracer_sim:${TAG}" \
    -c "$SIM_CMD" >/dev/null
for i in $(seq 1 30); do
    WID=$(display_find_window pit_helper "${DISP}")
    [ -n "$WID" ] && break; sleep 2
done
[ -z "$WID" ] && { echo "ERROR: simulator window not found"; exit 1; }
sleep 8
# Click geometry: the bare 1920x1080 canvas on :99, or -- on a real display --
# resized to 1920x1080 and located from a screenshot (tools/find_sim_ui.py),
# which also leaves the canvas origin in /tmp/sim_ui_origin for
# sim_set_quality.sh to reuse.
display_resolve_click_coords pit_helper "${DISP}" "$WID" "${REPO_DIR}"
# Organisers run Ultra. The helper container has xdotool; sim_set_quality.sh
# execs into autodrive_roboracer_api by default, so point it at the helper.
if [ "${SIM_QUALITY:-ultra}" = "ultra" ]; then
    DISP="$DISP" SIM_API=pit_helper ./scripts/sim_set_quality.sh ultra >/dev/null 2>&1 || echo "WARN: could not set Ultra"
    sleep 2
fi

# The submission image, as the organisers run it.
#
# ENTRY_MODE picks WHICH documented path is under test, and the default is the
# organisers' own: the official repo's "Competition Execution" uses
# `--entrypoint /bin/bash -it` -- which lands in a shell and starts NOTHING --
# and then runs `/home/autodrive_devkit.sh` from that shell. `-c "exec ..."` is
# that shell, non-interactively, so the image's CMD is bypassed exactly as it
# is for them. ENTRY_MODE=cmd tests the other path, the argument-less
# `docker run <image>` that boots the stack from the image's default CMD.
# Both must work; only the first is what the rules describe.
ENTRY_MODE="${ENTRY_MODE:-bash}"
case "$ENTRY_MODE" in
    bash) RUN_ARGS=(--entrypoint /bin/bash) ; RUN_CMD=(-c "exec /home/autodrive_devkit.sh") ;;
    cmd)  RUN_ARGS=()                       ; RUN_CMD=() ;;
    *)    echo "ENTRY_MODE must be bash or cmd"; exit 2 ;;
esac
echo "entry mode: $ENTRY_MODE"
docker run --name autodrive_roboracer_api --rm -d "${RUN_ARGS[@]}" \
    --network=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
    "$IMAGE" "${RUN_CMD[@]}" >/dev/null
sleep 12
docker exec pit_helper /bin/bash -c "$R && ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; true"
# The organisers may open extra shells in the running container. Nothing in the
# image may start on shell entry -- the base image's ~/.bashrc sources the two
# ROS overlays and nothing else, and our Dockerfile never touches it -- so a
# second shell must not add a node. (Run AFTER the daemon restart, or the two
# node lists are not comparable.)
nodes() { docker exec pit_helper /bin/bash -c "$R && ros2 node list 2>/dev/null | sort | tr '\n' ' '"; }
# Sample only once the stack is up: the nodes take a variable time to appear
# in discovery, and a BEFORE taken mid-startup fails the check spuriously.
for i in $(seq 1 30); do
    BEFORE="$(nodes)"
    case "$BEFORE" in *mpc*|*follow_the_gap*|*stanley*) break ;; esac
    sleep 2
done
docker exec autodrive_roboracer_api bash -lc 'true' >/dev/null 2>&1
sleep 3
AFTER="$(nodes)"
if [ "$BEFORE" != "$AFTER" ]; then
    echo "ERROR: a second shell changed the node list:"
    echo "  before: $BEFORE"
    echo "  after:  $AFTER"
    exit 1
fi
echo "second shell started no extra node: $AFTER"
LOG="/home/data/logs/submission_$(date +%Y%m%d_%H%M%S).csv"
docker exec -d pit_helper /bin/bash -c "$R && python3 /home/autodrive_devkit/src/pit_crew/tools/race_logger.py $LOG > /tmp/logger.log 2>&1"

click_at() { display_click_at pit_helper "${DISP}" "$1" "$2"; }
connected() {
    docker exec pit_helper /bin/bash -c "$R && timeout 6 ros2 topic echo --once /autodrive/roboracer_1/ips --field x >/dev/null 2>&1"
}
OK=0
for attempt in 1 2 3; do
    # Keep a screenshot per attempt on a real display (data/logs/display_click_*).
    if [ "${SHOT:-0}" = 1 ]; then
        docker exec pit_helper /bin/bash -c \
            "DISPLAY=${DISP} import -window root /tmp/click_$attempt.png 2>/dev/null; true"
        docker cp pit_helper:/tmp/click_$attempt.png \
            "${REPO_DIR}/data/logs/display_click_$(date +%H%M%S)_$attempt.png" >/dev/null 2>&1 || true
    fi
    click_at $CONN_X $CONN_Y; sleep 3; connected && { OK=1; break; }
    click_at $HAMB_X $HAMB_Y; sleep 1.5; click_at $CONN_X $CONN_Y; sleep 3; connected && { OK=1; break; }
done
[ "$OK" = 1 ] || { echo "ERROR: could not connect"; exit 1; }
echo "CONNECTED; racing the image for $LAPS laps (log ${LOG#/home/})"
HOST_LOG="${REPO_DIR}/data/logs/$(basename "$LOG")"
T0=$(date +%s)
while true; do
    sleep 5
    [ "$(wc -l < "$HOST_LOG" 2>/dev/null || echo 0)" -lt 3 ] && continue
    L=$(( $(tail -1 "$HOST_LOG" | cut -d, -f7) - $(sed -n 2p "$HOST_LOG" | cut -d, -f7) ))
    C=$(( $(tail -1 "$HOST_LOG" | cut -d, -f9) - $(sed -n 2p "$HOST_LOG" | cut -d, -f9) ))
    if [ "$L" -ge "$LAPS" ] || [ "$C" -ge 10 ] || [ $(( $(date +%s) - T0 )) -ge 900 ]; then break; fi
done
docker exec pit_helper /bin/bash -c "$R && ros2 node list 2>/dev/null | sort" | sed 's/^/  node: /'
docker stop --time 10 autodrive_roboracer_api >/dev/null 2>&1
docker rm -f pit_helper autodrive_roboracer_sim >/dev/null 2>&1
.venv/bin/python tools/score_race.py "$HOST_LOG"
