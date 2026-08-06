#!/bin/bash
# Open an extra shell inside the running devkit container (ROS 2 env pre-sourced).
exec docker exec -it autodrive_roboracer_api /bin/bash -c \
    "source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && cd /home/autodrive_devkit && exec bash"
