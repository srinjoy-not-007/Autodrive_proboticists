# Qualification image — IROS 2026

The qualification deadline is **September 12, 2026, anywhere on Earth**
(September 13 at 17:30 IST). Final submission is a separate deadline on
September 19. Checked September 10 against the
[event page](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/),
[rules](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-rules-2026/),
and [technical guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/).

## Build and publish

Replace `YOUR_DOCKERHUB_USER` with the actual repository owner. The tag below is
for qualification; do not build against the unreleased `compete` image.

```bash
docker build --platform linux/amd64 --build-arg TAG=2026-iros-practice \
  -f submission/Dockerfile \
  -t YOUR_DOCKERHUB_USER/roboracer:iros2026-qualification .
docker push YOUR_DOCKERHUB_USER/roboracer:iros2026-qualification
docker image inspect YOUR_DOCKERHUB_USER/roboracer:iros2026-qualification \
  --format '{{json .RepoDigests}}'
```

Record the pushed digest in the submission notes. If the repository is private,
give DockerHub user `autodriveecosystem` collaborator access. Submit the exact
repository/tag using the Phase 1 form linked from the event page. GitHub pushes
alone do not submit an entry. Publication and the form must be completed before
the deadline.

## DockerHub overview / evaluator instructions

This image contains Team Proboticists' ROS 2 Humble racing stack, based on the
official `2026-iros-practice` AutoDRIVE devkit. It includes Porto map/raceline
assets and needs no host repository, data volumes, downloads, or interactive
shell setup at runtime.

```bash
docker pull YOUR_DOCKERHUB_USER/roboracer:iros2026-qualification
docker run --name autodrive_roboracer_api --rm \
  --network=host --ipc=host --stop-timeout=10 \
  YOUR_DOCKERHUB_USER/roboracer:iros2026-qualification
```

Start the official Porto simulator separately, use autonomous mode and Ultra
graphics, then press Connection. The bridge, localization and MPC start
automatically and wait for sensor data. Do not override the image command with
an interactive shell. Opening an additional shell is safe:

```bash
docker exec -it autodrive_roboracer_api bash
# Inside the additional shell, source ROS for inspection/recording:
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
# From the host, orderly shutdown:
docker stop --time 10 autodrive_roboracer_api
```

Startup belongs entirely to `/home/autodrive_devkit.sh`; `.bashrc` is untouched.
The CPU-only team nodes need no GPU. The official simulator retains its own
GPU/display/container requirements. A required node exiting terminates the
stack with a nonzero status and attempts to clear both actuators before stopping
the bridge. Abrupt host loss/SIGKILL cannot run that cleanup.

## Qualification acceptance

- Run the image without source or data mounts; verify bridge and both team nodes.
- Wait before connecting; confirm the car begins when Connection is pressed.
- Open another shell; confirm it creates no second controller.
- Record one warm-up and ten simulator-counted race laps. Score elapsed crossing
  times and collisions in that window, never `10 * median` or geometric laps.
- Test stale sensor inputs, child-process failure, and orderly shutdown.
- Preserve the validated image digest before any performance experiments.

Only LiDAR, IMU, encoders and throttle feedback feed the runtime team nodes.
IPS, odometry, TF, resets and scoring topics are excluded from their inputs.
Development loggers may use them for evaluation but are never launched by this
image. The upstream `autodrive_roboracer` package is unchanged.

The baked optional ICRA assets are regression fixtures. Finals require separately
generated and driven assets for the released track plus a new image; this Porto
image must not be presented as finals-ready.
