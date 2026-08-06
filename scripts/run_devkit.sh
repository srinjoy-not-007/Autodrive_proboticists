#!/bin/bash
# Launch the AutoDRIVE Devkit container with our workspace mounted for development.
# Usage:
#   ./scripts/run_devkit.sh           # interactive bash shell (build & launch manually)
#   ./scripts/run_devkit.sh headless  # auto-launch bridge headless in background
#   ./scripts/run_devkit.sh graphics  # auto-launch bridge + RViz
set -e
TAG="${TAG:-2026-iros-practice}"
MODE="${1:-shell}"
IMAGE="autodriveecosystem/autodrive_roboracer_api:${TAG}"
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

docker rm -f autodrive_roboracer_api 2>/dev/null || true

# Extra deps our stack needs. Installed at container start because the base
# image lacks them and a restart discards anything pip-installed by hand.
# (These must also go into the submission Dockerfile — task #12.)
install_deps() {
    docker exec autodrive_roboracer_api /bin/bash -c \
        "python3 -c 'import scipy' 2>/dev/null || pip3 install -q scipy >/dev/null 2>&1; \
         which xdotool >/dev/null 2>&1 || (apt-get update -qq && apt-get install -y -qq xdotool imagemagick) >/dev/null 2>&1; true"
}

COMMON_ARGS=(
    --network=host --ipc=host
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "${REPO_DIR}/src:/home/autodrive_devkit/src/pit_crew"
    -v "${REPO_DIR}/data:/home/data"
    --env DISPLAY --privileged --gpus all
)

case "$MODE" in
    headless)
        docker run --name autodrive_roboracer_api --rm -d "${COMMON_ARGS[@]}" "$IMAGE" \
            -c "source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && ros2 launch autodrive_roboracer bringup_headless.launch.py"
        install_deps
        echo "Devkit bridge running headless (container: autodrive_roboracer_api)."
        ;;
    graphics)
        xhost local:root >/dev/null
        docker run --name autodrive_roboracer_api --rm -d "${COMMON_ARGS[@]}" "$IMAGE" \
            -c "source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && ros2 launch autodrive_roboracer bringup_graphics.launch.py"
        echo "Devkit bridge + RViz running (container: autodrive_roboracer_api)."
        ;;
    *)
        xhost local:root >/dev/null
        docker run --name autodrive_roboracer_api --rm -it "${COMMON_ARGS[@]}" "$IMAGE"
        ;;
esac
