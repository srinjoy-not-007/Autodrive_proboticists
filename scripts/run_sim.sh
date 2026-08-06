#!/bin/bash
# Launch the AutoDRIVE Simulator container.
# Usage:
#   ./scripts/run_sim.sh            # GUI mode (needed for camera + visual debugging)
#   ./scripts/run_sim.sh headless   # batchmode, no graphics (LiDAR-only workflows, CI)
#   TAG=2026-iros-practice ./scripts/run_sim.sh
set -e
TAG="${TAG:-2026-iros-practice}"
MODE="${1:-gui}"
IMAGE="autodriveecosystem/autodrive_roboracer_sim:${TAG}"

docker rm -f autodrive_roboracer_sim 2>/dev/null || true

if [ "$MODE" = "headless" ]; then
    docker run --name autodrive_roboracer_sim --rm -d \
        --network=host --ipc=host \
        "$IMAGE" \
        -c "cd /home/autodrive_simulator && ./AutoDRIVE\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567"
    echo "Simulator running headless (container: autodrive_roboracer_sim)."
else
    xhost local:root >/dev/null
    docker run --name autodrive_roboracer_sim --rm -it \
        --network=host --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --env DISPLAY --privileged --gpus all \
        "$IMAGE" \
        -c "cd /home/autodrive_simulator && ./AutoDRIVE\ Simulator.x86_64"
fi
