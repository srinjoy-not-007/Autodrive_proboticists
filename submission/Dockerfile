# Submission image for the RoboRacer Sim Racing League @ IROS 2026.
#
# Rules: must build FROM the official devkit container, must not modify the
# provided autodrive_roboracer package, and everything must start from
# autodrive_devkit.sh (never ~/.bashrc).
ARG TAG=2026-iros-practice
FROM autodriveecosystem/autodrive_roboracer_api:${TAG}

# Runtime deps our stack needs that the base image lacks. The base image is
# Python 3.10 / ROS 2 Humble.
RUN pip3 install --no-cache-dir scipy pyyaml

# Our racing package (never touches autodrive_roboracer)
COPY src /home/autodrive_devkit/src/pit_crew

# Track artefacts: map, centerline and both trajectories. Baked in so the
# container is self-sustaining with no mounted volumes.
COPY data/maps /home/data/maps
COPY data/racelines /home/data/racelines

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /home/autodrive_devkit && \
    colcon build --packages-select pit_crew --symlink-install"

COPY submission/autodrive_devkit.sh /home/autodrive_devkit.sh
RUN chmod +x /home/autodrive_devkit.sh

ENTRYPOINT ["/bin/bash"]
