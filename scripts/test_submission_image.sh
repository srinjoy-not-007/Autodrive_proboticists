#!/bin/bash
# Race the SUBMISSION IMAGE exactly as the organisers run it: no source or
# data mounts, the image's own entrypoint, the official docker flags. A helper
# container (the dev image) does what the organisers do by hand -- clicks
# Connection -- and records the evaluation log.
#
# Usage: ./scripts/test_submission_image.sh <image:tag> [laps] [sim_tag]
set -u
cd "$(dirname "$0")/.."
IMAGE="${1:?image:tag}"; LAPS="${2:-12}"; TAG="${3:-2026-iros-practice}"
REPO_DIR="$(pwd)"; DISP=":99"; GEOM="1920x1080"
R="source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"

docker rm -f autodrive_roboracer_api autodrive_roboracer_sim pit_helper >/dev/null 2>&1
# Helper: dev image with the repo mounted, used only for xdotool + the logger.
docker run --name pit_helper --rm -d --network=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "${REPO_DIR}/src:/home/autodrive_devkit/src/pit_crew" \
    -v "${REPO_DIR}/data:/home/data" --privileged \
    "autodriveecosystem/autodrive_roboracer_api:${TAG}" -c "sleep 7200" >/dev/null
docker exec pit_helper /bin/bash -c "which xdotool >/dev/null 2>&1 || (apt-get update -qq && apt-get install -y -qq xdotool) >/dev/null 2>&1; pip3 install -q 'numpy<2' 'scipy==1.10.1' >/dev/null 2>&1; true"

# Simulator on Xvfb :99 (see sim_restart_connect.sh for why).
docker run --name autodrive_roboracer_sim --rm -d --network=host --ipc=host --privileged --gpus all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw "autodriveecosystem/autodrive_roboracer_sim:${TAG}" \
    -c "Xvfb :99 -screen 0 ${GEOM}x24 >/dev/null 2>&1 & sleep 3; cd /home/autodrive_simulator &&
        DISPLAY=:99 ./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567 -logFile /home/unity.log; sleep 3600" >/dev/null
for i in $(seq 1 30); do
    WID=$(docker exec pit_helper /bin/bash -c "DISPLAY=${DISP} xdotool search --name '^AutoDRIVE Simulator\$' 2>/dev/null | head -1" || true)
    [ -n "$WID" ] && break; sleep 2
done
[ -z "$WID" ] && { echo "ERROR: simulator window not found"; exit 1; }
sleep 8
# Organisers run Ultra. The helper container has xdotool; sim_set_quality.sh
# execs into autodrive_roboracer_api by default, so point it at the helper.
if [ "${SIM_QUALITY:-ultra}" = "ultra" ]; then
    SIM_API=pit_helper ./scripts/sim_set_quality.sh ultra >/dev/null 2>&1 || echo "WARN: could not set Ultra"
    sleep 2
fi

# The submission image, as the organisers run it (plus -d instead of -it).
docker run --name autodrive_roboracer_api --rm -d --network=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all "$IMAGE" >/dev/null
sleep 12
docker exec pit_helper /bin/bash -c "$R && ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; true"
LOG="/home/data/logs/submission_$(date +%Y%m%d_%H%M%S).csv"
docker exec -d pit_helper /bin/bash -c "$R && python3 /home/autodrive_devkit/src/pit_crew/tools/race_logger.py $LOG > /tmp/logger.log 2>&1"

click_at() {
    docker exec pit_helper /bin/bash -c "export DISPLAY=${DISP}
         xdotool mousemove 900 600; sleep 0.4; xdotool mousemove $1 $2; sleep 1.2
         xdotool mousedown 1; sleep 0.2; xdotool mouseup 1"
}
connected() {
    docker exec pit_helper /bin/bash -c "$R && timeout 6 ros2 topic echo --once /autodrive/roboracer_1/ips --field x >/dev/null 2>&1"
}
OK=0
for attempt in 1 2 3; do
    click_at 75 409; sleep 3; connected && { OK=1; break; }
    click_at 75 43; sleep 1.5; click_at 75 409; sleep 3; connected && { OK=1; break; }
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
