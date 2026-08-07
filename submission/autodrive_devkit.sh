#!/bin/bash
# Competition entrypoint. Everything the car needs must start from here (the
# rules forbid using ~/.bashrc), and it must run only on container startup so
# stewards can open extra shells without relaunching the stack.
set -e

source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
cd /home/autodrive_devkit

MAP="${PIT_CREW_MAP:-/home/data/maps/track_v2}"
LINE="${PIT_CREW_LINE:-/home/data/racelines/track_v2_safe.csv}"
CENTER="${PIT_CREW_CENTERLINE:-/home/data/racelines/track_v2_centerline.csv}"
SCALE="${PIT_CREW_SPEED_SCALE:-1.00}"   # full profile speed; clean with Stanley
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
    ros2 run pit_crew stanley --ros-args \
        -p raceline_csv:="${LINE}" -p speed_scale:="${SCALE}" &
fi

wait
