#!/bin/bash
# Start once per container; additional interactive shells never start a race.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
cd /home/autodrive_devkit

# Exact map from the simulator mesh (tools/map_from_mesh.py) with the LiDAR's
# 0.1933 m forward offset applied in the filter: measured, this removes the
# 0.12-0.17 m inside bias the recon map had in the hairpins. A recon map built
# with PIT_CREW_LIDAR_OFFSET=0 must be run with PIT_CREW_LIDAR_OFFSET=0.
MAP="${PIT_CREW_MAP:-/home/data/maps/porto_mesh}"
LIDAR_OFFSET="${PIT_CREW_LIDAR_OFFSET:-0.1933}"
LINE="${PIT_CREW_LINE:-/home/data/racelines/track_v2_safe.csv}"
CENTER="${PIT_CREW_CENTERLINE:-/home/data/racelines/track_v2_centerline.csv}"
SCALE="${PIT_CREW_SPEED_SCALE:-1.00}"
MODE="${PIT_CREW_MODE:-raceline}"
if [[ "$MODE" != raceline && "$MODE" != reactive ]]; then
    echo "Invalid PIT_CREW_MODE: $MODE" >&2; exit 2
fi
if [[ "${PIT_CREW_CONTROLLER:-mpc}" != mpc ]]; then
    echo "Submission supports MPC only; use PIT_CREW_MODE=reactive for FTG" >&2
    exit 2
fi
if [[ "$MODE" == raceline ]]; then
    python3 -m pit_crew.preflight "$MAP" "$LINE" "$CENTER" "$SCALE"
fi

BRIDGE=""
WORKERS=()
stop_group() { kill -TERM -- "-$1" 2>/dev/null || true; }
shutdown() {
    code=$?
    trap - EXIT INT TERM
    # Stop producers first, including ros2-run descendants. Keep the bridge
    # alive while final commands drain so latched controls are cleared.
    for pid in "${WORKERS[@]}"; do stop_group "$pid"; done
    sleep 0.3
    for pid in "${WORKERS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
    timeout 4 ros2 topic pub -r 20 --times 20 \
        /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1 &
    ZERO_T=$!
    timeout 4 ros2 topic pub -r 20 --times 20 \
        /autodrive/roboracer_1/steering_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1 &
    ZERO_S=$!
    wait "$ZERO_T" "$ZERO_S" 2>/dev/null || true
    if [[ -n "$BRIDGE" ]]; then stop_group "$BRIDGE"; fi
    exit "$code"
}
trap shutdown EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid ros2 launch autodrive_roboracer bringup_headless.launch.py &
BRIDGE=$!
if [[ "$MODE" == reactive ]]; then
    setsid ros2 run pit_crew follow_the_gap --ros-args \
        --params-file /home/autodrive_devkit/src/pit_crew/config/ftg_mapping.yaml &
    WORKERS+=("$!")
else
    setsid ros2 run pit_crew localization --ros-args \
        -p map_prefix:="$MAP" -p centerline_csv:="$CENTER" \
        -p lidar_offset:="$LIDAR_OFFSET" &
    WORKERS+=("$!")
    setsid ros2 run pit_crew mpc --ros-args \
        -p raceline_csv:="$LINE" -p speed_scale:="$SCALE" &
    WORKERS+=("$!")
fi
# A child exiting, even successfully, is a failed race stack.
result=0
wait -n "$BRIDGE" "${WORKERS[@]}" || result=$?
echo "Required race process exited (status $result); shutting down" >&2
exit 1
