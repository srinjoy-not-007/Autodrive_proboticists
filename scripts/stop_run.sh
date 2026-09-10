#!/bin/bash
# Development-only shutdown. Submission images supervise their own children.
set -e
docker exec -i autodrive_roboracer_api /bin/bash <<'STOP'
source /opt/ros/humble/setup.bash
# Source in this shell, not in a background `source && publisher &` subshell.
timeout 6 ros2 topic pub -r 20 /autodrive/roboracer_1/throttle_command \
    std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1 &
ZERO_T=$!
timeout 6 ros2 topic pub -r 20 /autodrive/roboracer_1/steering_command \
    std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1 &
ZERO_S=$!
sleep 1.5
pkill -f '[i]nstall/pit_crew/lib' || true
pkill -INT -f '[r]ace_logger' || true
pkill -INT -f '[r]os2 bag record' || true
wait "$ZERO_T" "$ZERO_S" || true
STOP
echo "Run stopped; zero commands published on both actuator topics."
