# Submission image for the RoboRacer Sim Racing League @ IROS 2026.
#
# Rules: must build FROM the official devkit container, must not modify the
# provided autodrive_roboracer package, and everything must start from
# autodrive_devkit.sh (never ~/.bashrc).
ARG TAG=2026-iros-practice
FROM autodriveecosystem/autodrive_roboracer_api:${TAG}

# Runtime deps our stack needs that the base image lacks. The base image is
# Python 3.10 / ROS 2 Humble.
#
# numpy is PINNED BELOW 2 on purpose. The base image ships numpy 1.22.2, and the
# ROS 2 Humble C-extensions in it (cv_bridge among them) are compiled against
# the 1.x ABI. An unpinned `pip3 install scipy` resolves numpy to 2.x, after
# which the devkit's own bridge prints an ABI warning at startup and cv_bridge
# fails outright with "AttributeError: _ARRAY_API not found". We are forbidden
# from modifying autodrive_roboracer to fix that, so we must not break it.
# scipy 1.10.1 accepts numpy>=1.19.5, so nothing gets upgraded.
RUN pip3 install --no-cache-dir "numpy<2" "scipy==1.10.1" pyyaml

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

# The base image is ENTRYPOINT ["/bin/bash"] with no CMD, so `docker run <image>`
# lands in a shell and starts nothing. Keep the entrypoint (the rules and every
# helper script pass `-c "..."`), but add a default command so an argument-less
# run boots the race stack instead of idling. Passing `-c "..."` still overrides
# this, so nothing that works today changes.
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "/home/autodrive_devkit.sh"]
