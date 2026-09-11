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
SLUG="$(basename "$MAP")"
LIDAR_OFFSET="${PIT_CREW_LIDAR_OFFSET:-0.1933}"
# Wide min-curvature line on the mesh map, driven by mpc3 (see STATUS, Sept 12
# night). PIT_CREW_CONTROLLER=mpc selects the legacy tracker, which needs the
# centerline (track_v2_safe.csv); do not pair mpc with the wide line.
CONTROLLER="${PIT_CREW_CONTROLLER:-mpc3}"
if [[ "$CONTROLLER" == mpc3 ]]; then
    LINE="${PIT_CREW_LINE:-/home/data/racelines/porto_mesh_mc0.60_ay6.0.csv}"
else
    LINE="${PIT_CREW_LINE:-/home/data/racelines/track_v2_safe.csv}"
fi
# The centerline is derived from the MAP and never from a fixed Porto path.
# It is not a diagnostic: localization seeds its entire particle cloud at
# centerline[0] and resamples along it for EVERY collision recovery, so a
# centerline from another track is a race with no laps in it.
CENTER="${PIT_CREW_CENTERLINE:-/home/data/racelines/${SLUG}_centerline.csv}"
SCALE="${PIT_CREW_SPEED_SCALE:-1.00}"
MODE="${PIT_CREW_MODE:-raceline}"
if [[ "$MODE" != raceline && "$MODE" != reactive ]]; then
    echo "Invalid PIT_CREW_MODE: $MODE" >&2; exit 2
fi
if [[ "$CONTROLLER" != mpc && "$CONTROLLER" != mpc3 ]]; then
    echo "Submission supports mpc or mpc3; use PIT_CREW_MODE=reactive for FTG" >&2
    exit 2
fi
if [[ "$MODE" == raceline ]]; then
    if [[ ! -f "$CENTER" ]]; then
        echo "no centerline for $SLUG: $CENTER" >&2; exit 2
    fi
    # Preflight now also cross-checks that the line and the centerline lie on
    # THIS map and that the map's own lidar_offset matches the one we localize
    # with -- the two finals mistakes that no per-file check can see.
    python3 -m pit_crew.preflight "$MAP" "$LINE" "$CENTER" "$SCALE" "$LIDAR_OFFSET"
fi

BRIDGE=""
WORKERS=()
# mpc3 and mpc2 take the corridor map explicitly; the legacy mpc does not
# declare the parameter and would refuse to start if it were passed. Deriving
# it from the raceline NAME (the old behaviour) silently disabled the corridor
# barrier whenever the guess did not resolve.
CTRL_ARGS=()
if [[ "$CONTROLLER" == mpc3 || "$CONTROLLER" == mpc2 ]]; then
    CTRL_ARGS=(-p map_prefix:="$MAP")
fi

LOC_PID=""; CTRL_PID=""; FTG_PID=""
start_loc() {
    setsid ros2 run pit_crew localization --ros-args \
        -p map_prefix:="$MAP" -p centerline_csv:="$CENTER" \
        -p lidar_offset:="$LIDAR_OFFSET" &
    LOC_PID=$!
}
start_ctrl() {
    setsid ros2 run pit_crew "$CONTROLLER" --ros-args \
        -p raceline_csv:="$LINE" -p speed_scale:="$SCALE" "${CTRL_ARGS[@]}" &
    CTRL_PID=$!
}
start_ftg() {
    setsid ros2 run pit_crew follow_the_gap --ros-args \
        --params-file /home/autodrive_devkit/src/pit_crew/config/ftg_mapping.yaml &
    FTG_PID=$!
}
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
    start_ftg
    WORKERS=("$FTG_PID")
else
    start_loc
    start_ctrl
    WORKERS=("$LOC_PID" "$CTRL_PID")
fi

# A child exiting, even successfully, is a failed race stack -- but a dead
# node must not be a dead RACE. The organisers run all 12 laps in one go, so a
# single unhandled exception used to cost every remaining lap and the whole
# score. Restart ONLY the node that died (restarting localization needlessly
# would throw away a converged particle filter mid-lap), a bounded number of
# times, then fall back to the reactive stack, which needs no map, no line and
# no localization at all. A dead ROS bridge is the one thing not recoverable
# from here.
#
# `wait -n` reaps exactly one child, so `kill -0` afterwards identifies it.
RESTARTS=0
MAX_RESTARTS=4
while :; do
    result=0
    wait -n "$BRIDGE" "${WORKERS[@]}" || result=$?
    if ! kill -0 "$BRIDGE" 2>/dev/null; then
        echo "ROS bridge exited (status $result); shutting down" >&2
        exit 1
    fi
    RESTARTS=$((RESTARTS + 1))
    sleep 0.5                       # never spin on a node that dies instantly
    if [[ "$MODE" == reactive ]]; then
        if (( RESTARTS > MAX_RESTARTS )); then break; fi
        echo "follow_the_gap exited ($result); restart $RESTARTS" >&2
        start_ftg
        WORKERS=("$FTG_PID")
        continue
    fi
    if (( RESTARTS > MAX_RESTARTS )); then
        echo "race stack failed $MAX_RESTARTS times; falling back to \
follow_the_gap" >&2
        for pid in "${WORKERS[@]}"; do stop_group "$pid"; done
        sleep 0.3
        MODE=reactive
        start_ftg
        WORKERS=("$FTG_PID")
        RESTARTS=0
        continue
    fi
    kill -0 "$LOC_PID" 2>/dev/null || { echo "localization exited ($result); \
restart $RESTARTS" >&2; start_loc; }
    kill -0 "$CTRL_PID" 2>/dev/null || { echo "$CONTROLLER exited ($result); \
restart $RESTARTS" >&2; start_ctrl; }
    WORKERS=("$LOC_PID" "$CTRL_PID")
done
echo "Reactive fallback exited too; shutting down" >&2
exit 1
