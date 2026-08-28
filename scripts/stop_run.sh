#!/bin/bash
# Stop FTG, logger and rosbag inside the devkit container; zero the actuators.
#
# Order matters. Commands LATCH in this simulator: whatever steering and
# throttle were last published keep being applied until something overwrites
# them. The previous version killed the controller, slept 2 s, and only then
# zeroed -- so the car drove its last command, unsteered, for those 2 s, with
# the logger already dead and whatever it hit unrecorded.
#
# Zeroing before the kill does not work either: the controller is still running
# and simply overwrites it on its next cycle. So start the zero publishers
# FIRST and leave them running, then kill the nodes underneath them. The
# controller wins the topic while it is alive; the moment it dies, the zeros
# already in flight take over with no gap.
docker exec autodrive_roboracer_api /bin/bash -c "
    source /opt/ros/humble/setup.bash &&
    timeout 6 ros2 topic pub -r 20 /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1 &
    timeout 6 ros2 topic pub -r 20 /autodrive/roboracer_1/steering_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1 &
    sleep 1.5;
    pkill -f '[i]nstall/pit_crew/lib'; pkill -f '[r]ace_logger'; pkill -INT -f '[r]os2 bag record';
    wait;
    true"
echo "Run stopped, actuators zeroed."
