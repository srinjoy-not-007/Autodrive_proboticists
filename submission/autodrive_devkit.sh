#!/bin/bash
# Competition entrypoint. Everything the car needs must start from here (the
# rules forbid using ~/.bashrc), and it must run only on container startup so
# stewards can open extra shells without relaunching the stack.
set -e

source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
cd /home/autodrive_devkit

# Commands LATCH in this simulator: whatever was last published keeps being
# applied. If the stack is stopped without zeroing, the car drives on by itself.
# Best effort only — nothing survives SIGKILL — but it covers an orderly stop.
zero_actuators() {
    timeout 2 ros2 topic pub -r 20 --times 10 \
        /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 '{data: 0.0}' \
        > /dev/null 2>&1 || true
    timeout 2 ros2 topic pub -r 20 --times 10 \
        /autodrive/roboracer_1/steering_command std_msgs/msg/Float32 '{data: 0.0}' \
        > /dev/null 2>&1 || true
}
shutdown() { zero_actuators; kill 0 2>/dev/null; exit 0; }
trap shutdown SIGINT SIGTERM

MAP="${PIT_CREW_MAP:-/home/data/maps/track_v2}"
LINE="${PIT_CREW_LINE:-/home/data/racelines/track_v2_safe.csv}"
CENTER="${PIT_CREW_CENTERLINE:-/home/data/racelines/track_v2_centerline.csv}"
SCALE="${PIT_CREW_SPEED_SCALE:-1.00}"   # full profile speed; clean on both tracks
MODE="${PIT_CREW_MODE:-raceline}"   # raceline | reactive

# The bridge to the simulator
ros2 launch autodrive_roboracer bringup_headless.launch.py &
sleep 5

if [ "$MODE" = "reactive" ]; then
    # Reactive-only: no map needed, drives any track. Used when the raceline
    # stack has not been validated on the track in question.
    ros2 run pit_crew follow_the_gap --ros-args \
        --params-file /home/autodrive_devkit/src/pit_crew/config/ftg_mapping.yaml &
else
    # Localization + Stanley tracker, driving the car directly.
    #
    # The arbiter is deliberately NOT started. Measured on this track it caused
    # the collisions it was meant to prevent: with it in the loop the same line,
    # speed and localization produced 18-29 collisions and no completed laps,
    # while the tracker publishing directly runs clean (11.21 s/lap).
    # Swapping control between two controllers with independent steering state
    # destabilises the car more than the hazards it reacts to.
    ros2 run pit_crew localization --ros-args \
        -p map_prefix:="${MAP}" -p centerline_csv:="${CENTER}" &
    sleep 4
    # MPC over Stanley: it is the only controller measured clean on a track it
    # was not tuned on (ICRA, 13 consecutive laps, 0 collisions, full speed),
    # it is smoother (1.42 reversals/s at 2.80 rad/s, inside the 3.2 rad/s
    # actuator limit, against Stanley's 1.68 at 4.32 - above it), and it is now
    # also marginally faster on Porto (11.32 s vs 11.24 s Stanley, 22 laps
    # clean). Stanley remains runnable via PIT_CREW_CONTROLLER=stanley.
    ros2 run pit_crew "${PIT_CREW_CONTROLLER:-mpc}" --ros-args \
        -p raceline_csv:="${LINE}" -p speed_scale:="${SCALE}" &
fi

wait
