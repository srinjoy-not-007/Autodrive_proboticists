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
TAG="${TAG:-2026-iros-practice}"
GEOM="1920x1080"
if [ "${VISIBLE:-0}" = "1" ]; then
    DISP="${DISPLAY:-:0}"
    echo "VISIBLE=1: simulator will open on your display ${DISP}."
    echo "It will move the mouse pointer briefly to click Connection."
    xhost local:root >/dev/null 2>&1 || true
else
    DISP=":99"
fi

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
    WID=$(docker exec autodrive_roboracer_api /bin/bash -c \
        "DISPLAY=${DISP} xdotool search --name '^AutoDRIVE Simulator\$' 2>/dev/null | head -1" || true)
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
click_at() {
    docker exec autodrive_roboracer_api /bin/bash -c \
        "export DISPLAY=${DISP}
         xdotool mousemove 900 600; sleep 0.4
         xdotool mousemove $1 $2; sleep 1.2
         xdotool mousedown 1; sleep 0.2; xdotool mouseup 1"
}

# On :99 there is no window manager: the canvas is a bare 1920x1080 at 0,0.
# On a real display the window is decorated and smaller, so the UI is scaled
# and offset -- take the geometry from the window itself rather than assuming.
if [ "${DISP}" = ":99" ]; then
    HAMB_X=75;  HAMB_Y=43;  CONN_X=75;  CONN_Y=409
else
    # Make the window the 1920x1080 canvas the coordinates were measured on:
    # on a 5120x2880 desktop Unity opens small and the UI is scaled, and
    # fixed offsets (the old +51/+64/+302) missed every click. Resize, then
    # read the geometry back (Position is the client-area origin, i.e. it
    # already includes the window-manager decoration) and fall back to
    # proportional scaling if the window manager refused the size.
    docker exec autodrive_roboracer_api /bin/bash -c \
        "DISPLAY=${DISP} xdotool windowmove $WID 0 0; sleep 0.5
         DISPLAY=${DISP} xdotool windowsize $WID 1920 1080; sleep 1.5
         DISPLAY=${DISP} xdotool windowactivate $WID 2>/dev/null; true"
    GEO=$(docker exec autodrive_roboracer_api /bin/bash -c \
        "DISPLAY=${DISP} xdotool getwindowgeometry $WID")
    OX=$(echo "$GEO" | sed -n 's/.*Position: \([0-9]*\),\([0-9]*\).*/\1/p')
    OY=$(echo "$GEO" | sed -n 's/.*Position: \([0-9]*\),\([0-9]*\).*/\2/p')
    WW=$(echo "$GEO" | sed -n 's/.*Geometry: \([0-9]*\)x\([0-9]*\).*/\1/p')
    WH=$(echo "$GEO" | sed -n 's/.*Geometry: \([0-9]*\)x\([0-9]*\).*/\2/p')
    echo "window $WID at ${OX},${OY} size ${WW}x${WH} (frame)"
    # The frame geometry is not the canvas: find the sim's header bar in a
    # screenshot (tools/find_sim_ui.py) and use its origin. Falls back to the
    # frame with a proportional scale if the bar is not found.
    UI=""
    for shot in 1 2 3 4; do
        sleep 2
        docker exec autodrive_roboracer_api /bin/bash -c \
            "DISPLAY=${DISP} import -window root /tmp/ui.png 2>/dev/null; true"
        docker cp autodrive_roboracer_api:/tmp/ui.png /tmp/sim_ui.png >/dev/null 2>&1 || true
        UI=$("${REPO_DIR}/.venv/bin/python" "${REPO_DIR}/tools/find_sim_ui.py" /tmp/sim_ui.png 2>/dev/null || true)
        # accept only a settled, full-width canvas (the resize animates)
        if [ -n "$UI" ] && [ "$(echo "$UI" | cut -d" " -f3)" -ge 1900 ]; then break; fi
        UI=""
    done
    if [ -n "$UI" ]; then
        read -r CX CY CW <<<"$UI"
        echo "canvas at ${CX},${CY} width ${CW}"
        echo "$CX $CY $CW" > /tmp/sim_ui_origin
        HAMB_X=$((CX + 75 * CW / 1920)); HAMB_Y=$((CY + 43 * CW / 1920))
        CONN_X=$((CX + 75 * CW / 1920)); CONN_Y=$((CY + 409 * CW / 1920))
    else
        echo "WARN: header bar not found in screenshot; scaling from the frame"
        rm -f /tmp/sim_ui_origin
        HAMB_X=$((OX + 75 * WW / 1920)); HAMB_Y=$((OY + 43 * WH / 1080))
        CONN_X=$((OX + 75 * WW / 1920)); CONN_Y=$((OY + 409 * WH / 1080))
    fi
    # Keep a screenshot per attempt for diagnosis (data/logs/display_click_*.png).
    SHOT=1
fi
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
