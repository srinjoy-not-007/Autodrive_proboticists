#!/bin/bash
# Stop FTG, logger and rosbag inside the devkit container; zero the actuators.
docker exec autodrive_roboracer_api /bin/bash -c "
    pkill -f '[i]nstall/pit_crew/lib'; pkill -f '[r]ace_logger'; pkill -INT -f '[r]os2 bag record';
    sleep 2;
    source /opt/ros/humble/setup.bash &&
    timeout 3 ros2 topic pub -r 20 /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1;
    timeout 2 ros2 topic pub -r 20 /autodrive/roboracer_1/steering_command std_msgs/msg/Float32 '{data: 0.0}' >/dev/null 2>&1;
    true"
echo "Run stopped, actuators zeroed."
