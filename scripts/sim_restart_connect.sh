#!/bin/bash
# Restart the simulator (windowed, on the host display) and auto-click Connection.
# LiDAR/camera need real rendering: do NOT use -batchmode -nographics (frozen LiDAR).
set -e
TAG="${TAG:-2026-iros-practice}"

xhost local:root >/dev/null
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# Restart the devkit bridge first: a sim restart leaves it holding a dead socket
"${REPO_DIR}/scripts/run_devkit.sh" headless
echo "Reinstalling X tools in devkit container (for window clicking)..."
docker exec autodrive_roboracer_api /bin/bash -c \
    "apt-get update -qq >/dev/null 2>&1 && apt-get install -y -qq xdotool x11-apps imagemagick >/dev/null 2>&1" || true

docker rm -f autodrive_roboracer_sim 2>/dev/null || true
docker run --name autodrive_roboracer_sim --rm -d \
    --network=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env DISPLAY --privileged --gpus all \
    "autodriveecosystem/autodrive_roboracer_sim:${TAG}" \
    -c "cd /home/autodrive_simulator && ./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567 -logFile /home/unity.log; sleep 3600"

echo "Waiting for simulator window..."
for i in $(seq 1 30); do
    WID=$(docker exec autodrive_roboracer_api /bin/bash -c \
        "xdotool search --name '^AutoDRIVE Simulator\$' 2>/dev/null | head -1" || true)
    [ -n "$WID" ] && break
    sleep 2
done
[ -z "$WID" ] && { echo "ERROR: simulator window not found"; exit 1; }
sleep 8    # let the scene finish loading

connected() {
    docker exec autodrive_roboracer_api /bin/bash -c \
        "source /opt/ros/humble/setup.bash && ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; timeout 6 ros2 topic echo --once /autodrive/roboracer_1/ips --field x >/dev/null 2>&1"
}

echo "Clicking Connection button (window $WID)..."
docker exec autodrive_roboracer_api /bin/bash -c "xdotool windowactivate $WID" || true
sleep 1
for attempt in 1 2 3; do
    # The sidebar may be open or collapsed; the hamburger (51,64) toggles it.
    # Try Connection directly first, then toggle the sidebar and try again.
    docker exec autodrive_roboracer_api /bin/bash -c \
        "xdotool mousemove --window $WID 51 302 click 1"
    sleep 3
    if connected; then echo "CONNECTED (IPS data flowing)"; exit 0; fi
    docker exec autodrive_roboracer_api /bin/bash -c \
        "xdotool mousemove --window $WID 51 64 click 1"
    sleep 2
done
echo "WARNING: could not verify connection — click the Connection button in the sim window manually"
exit 1
