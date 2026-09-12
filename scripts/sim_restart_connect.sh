#!/bin/bash
# Restart the simulator and click Connection, on a private virtual display.
#
# WHY XVFB, NOT THE HOST DISPLAY
# ------------------------------
# The simulator only links to the devkit bridge when its GUI "Connection"
# button is physically clicked. Verified, so nobody re-derives it:
#   - `-ip` / `-port` only PRE-FILL the fields. Launched with them and no
#     click, the Unity log records zero WebSocket.Connect attempts.
#   - Unity's PlayerPrefs hold window geometry only -- no connection state to
#     seed, no autoconnect flag.
#   - `xdotool --window` (XSendEvent, which does not move the pointer) is
#     ignored by Unity, as by most games. Only real XTEST events land.
#
# So a genuine pointer click is required -- but it does not have to be on the
# user's display. Driving the host pointer teleported the mouse out from under
# whoever was using the machine, and needed the sim window kept on top. Here
# the sim renders on Xvfb :99 INSIDE its own container: no host X socket, no
# xhost, no DISPLAY, no window manager, nothing on any monitor.
#
# This is NOT the frozen-LiDAR trap. That is `-batchmode -nographics`, which
# skips rendering entirely and republishes one stale scan forever. Xvfb is a
# real X server and Unity really renders; the GPU raytracer is reached through
# CUDA and does not care that the framebuffer is virtual. Measured on this
# setup: 10.97 Hz, ten consecutive scans all DISTINCT.
#
# VISIBLE=1 puts the sim on the real display instead, for watching a run. It is
# opt-in precisely because it takes over the pointer to click Connection, so it
# should only ever be used when somebody has ASKED to watch.
set -e
. "$(dirname "$0")/lib_display.sh"   # display_select / click / canvas geometry
TAG="${TAG:-2026-iros-practice}"
GEOM="1920x1080"
display_select

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# Restart the devkit bridge first: a sim restart leaves it holding a dead socket
"${REPO_DIR}/scripts/run_devkit.sh" headless

docker rm -f autodrive_roboracer_sim 2>/dev/null || true
# /tmp/.X11-unix is mounted so the Xvfb socket for :99 is visible to the devkit
# container too -- that is where xdotool already lives (run_devkit.sh installs
# it once per bridge restart), so nothing has to be apt-installed into this
# throwaway sim container on every single run. Nothing is displayed on any real
# screen: :99 exists only in memory.
# The telemetry period is set by ONE CPU-saturated Unity render thread (GPU
# idle, no vsync on Xvfb), so which cores it lands on decides the rate
# (0.088-0.106 s per start). SIM_CPUSET pins the container, e.g.
# SIM_CPUSET=0-7,16-23 (CCD0, fast) or a narrower set to slow it.
docker run --name autodrive_roboracer_sim --rm -d \
    ${SIM_CPUSET:+--cpuset-cpus=$SIM_CPUSET} \
    --network=host --ipc=host --privileged --gpus all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "autodriveecosystem/autodrive_roboracer_sim:${TAG}" \
    -c "if [ '${DISP}' = ':99' ]; then Xvfb :99 -screen 0 ${GEOM}x24 >/dev/null 2>&1 & sleep 3; fi
        cd /home/autodrive_simulator &&
        DISPLAY=${DISP} ./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567 \
            -logFile /home/unity.log; sleep 3600"

echo "Waiting for simulator window on ${DISP}..."
for i in $(seq 1 30); do
    WID=$(display_find_window autodrive_roboracer_api "${DISP}")
    [ -n "$WID" ] && break
    sleep 2
done
[ -z "$WID" ] && { echo "ERROR: simulator window not found on ${DISP}"; exit 1; }
sleep 8    # let the scene finish loading

connected() {
    docker exec autodrive_roboracer_api /bin/bash -c \
        "source /opt/ros/humble/setup.bash && ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; timeout 6 ros2 topic echo --once /autodrive/roboracer_1/ips --field x >/dev/null 2>&1"
}

# Unity's UI only reacts to a click on a control it has seen the pointer ENTER,
# and wants the press and release in separate frames. Approach from elsewhere,
# then hold the button down briefly; `xdotool click` alone frequently misses.
# Coordinates are for a 1920x1080 canvas with no window-manager decoration.
click_at() { display_click_at autodrive_roboracer_api "${DISP}" "$1" "$2"; }

display_resolve_click_coords autodrive_roboracer_api "${DISP}" "$WID" "${REPO_DIR}"

# MANUAL_CONNECT=1: a human clicks Connection (and sets graphics quality),
# exactly as the organisers do. Wait for data instead of clicking.
if [ "${MANUAL_CONNECT:-0}" = "1" ]; then
    echo "MANUAL_CONNECT=1: waiting up to ${MANUAL_WAIT_S:-300} s for a human to click Connection on ${DISP}..."
    for i in $(seq 1 $(( ${MANUAL_WAIT_S:-300} / 5 ))); do
        if connected; then echo "CONNECTED (IPS data flowing)"; exit 0; fi
        sleep 2
    done
    echo "ERROR: nobody connected within ${MANUAL_WAIT_S:-300} s."; exit 1
fi
echo "Clicking Connection on ${DISP} (window $WID)..."
for attempt in 1 2 3; do
    # The sidebar may be open or collapsed and there is no way to ask which.
    # The hamburger (75,43) toggles it; Connection sits at (75,409) when open.
    if [ "${SHOT:-0}" = 1 ]; then
        docker exec autodrive_roboracer_api /bin/bash -c \
            "DISPLAY=${DISP} import -window root /tmp/click_$attempt.png 2>/dev/null; true"
        docker cp autodrive_roboracer_api:/tmp/click_$attempt.png \
            "${REPO_DIR}/data/logs/display_click_$(date +%H%M%S)_$attempt.png" >/dev/null 2>&1 || true
    fi
    click_at $CONN_X $CONN_Y
    sleep 3
    if connected; then echo "CONNECTED (IPS data flowing)"; exit 0; fi
    click_at $HAMB_X $HAMB_Y
    sleep 1.5
    click_at $CONN_X $CONN_Y
    sleep 3
    if connected; then echo "CONNECTED (IPS data flowing)"; exit 0; fi
done
echo "ERROR: could not connect after 3 attempts."
echo "Inspect what the sim is showing with:"
echo "  docker exec autodrive_roboracer_api /bin/bash -c 'DISPLAY=:99 import -window root /tmp/s.png'"
echo "  docker cp autodrive_roboracer_api:/tmp/s.png /tmp/s.png"
exit 1
