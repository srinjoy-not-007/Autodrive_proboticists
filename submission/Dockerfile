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

# Track artefacts: maps, centerlines and the lines we are prepared to race.
# Baked in so the container is self-sustaining with no mounted volumes.
#
# MAPS are globbed over `*_mesh` (plus the legacy recon track_v2) precisely so
# that a finals map produced by `scripts/finals_drill_mesh.sh` ships without an
# edit here -- the commonest way to lose a race is an asset that did not make
# it into the image. A map with no line is unreachable: nothing loads a map
# that PIT_CREW_MAP does not name. ~3.6 MB total.
COPY data/maps/track_v2.pgm data/maps/track_v2.yaml /home/data/maps/
COPY data/maps/*_mesh.pgm data/maps/*_mesh.yaml /home/data/maps/

# RACELINES are listed one by one, never globbed, and that is deliberate. This
# used to copy the whole directory, which meant every rejected candidate and
# every intermediate variant shipped too -- including an ICRA raceline that
# reversed direction twice and asked for a 0.327 m turn radius against a
# 0.561 m steering lock. Gated out of race.csv, but one PIT_CREW_LINE typo away
# from being driven in the final. A line nobody has cleared should not be
# reachable inside the submission at all.
#
# Every line here has been driven clean; the entrypoint picks the shipping one (cr_dk0.8 at the time of writing: mc0.63_ay6.75 with its hairpin-entry curvature ramp halved, 100 laps / 0 collisions / 70.5 s at the fast sensor rate with rate_adaptive). mc0.60_ay6.0 (the earlier
# line, 100 laps / 0 collisions / 73.46 s with mpc3), track_v2_safe (100 laps
# clean with the legacy mpc), icra2026_mesh_race (60 laps / 0 collisions).
# ADD THE FINALS LINES HERE after the drill has produced and driven them.
COPY data/racelines/track_v2_safe.csv data/racelines/track_v2_race.csv \
     data/racelines/track_v2_centerline.csv \
     data/racelines/porto_mesh_mc0.60_ay6.0.csv data/racelines/porto_mesh_mc0.60_ay6.5.csv \
     data/racelines/porto_mesh_mc0.63_ay6.75.csv data/racelines/porto_mesh_cr_dk0.8.csv \
     data/racelines/icra2026_mesh_centerline.csv \
     data/racelines/icra2026_mesh_safe.csv \
     data/racelines/icra2026_mesh_race.csv \
     /home/data/racelines/
# porto_mesh_{centerline,safe} are symlinks to the track_v2 files in the repo;
# copy the targets under the names the entrypoint derives from PIT_CREW_MAP, so
# nothing depends on how Docker treats a symlink in the build context.
COPY data/racelines/track_v2_centerline.csv /home/data/racelines/porto_mesh_centerline.csv
COPY data/racelines/track_v2_safe.csv /home/data/racelines/porto_mesh_safe.csv

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
CMD ["-c", "exec /home/autodrive_devkit.sh"]
