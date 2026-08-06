# AutoDrive — RoboRacer Sim Racing League @ IROS 2026

Team workspace for the [6th RoboRacer Sim Racing League @ IROS 2026](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/).

**Objective:** drive fast, don't crash. Qualification = 10 clean laps on the Porto practice track. Finals = 10-lap time attack on a track released 3 days before submission. Collisions cost +10 s each (cumulative); >10 collisions = DQ.

## Key dates (2026)

| Date | Event |
|---|---|
| Sep 6 | **Registration closes** ([register here](https://forms.gle/MiuoSMYQ4z6QACLS6)) |
| Sep 12 | Phase 1 (qualification) submission deadline |
| Sep 16 | Competition track released |
| Sep 19 | Phase 2 (final race) submission deadline |

## Layout

```
scripts/     helper scripts (pull images, run sim/devkit, smoke test)
src/         our ROS 2 packages (mounted into the devkit container at src/pit_crew)
data/        recorded runs, maps, racelines (git-ignored)
docs/        PLAN.md — strategy and roadmap
upstream/    clone of the official competition repo (git-ignored)
```

## Quick start

```bash
./scripts/pull_images.sh              # pull 2026-iros-practice sim + devkit images
./scripts/run_sim.sh                  # simulator with GUI (or: run_sim.sh headless)
./scripts/run_devkit.sh headless      # devkit bridge (or: graphics | no-arg for shell)
./scripts/shell_devkit.sh             # extra shell inside running devkit container
./scripts/smoke_test.sh               # end-to-end check: LiDAR flow + actuation
```

Inside the sim GUI: click **Connection** to link to the bridge (WebSocket :4567).

## Vehicle / interface cheat sheet

- LiDAR: 1080 pts, 270° (±135°), 0.06–10 m, 40 Hz — `/autodrive/roboracer_1/lidar`
- IMU + 2 rear wheel encoders (16 PPR); camera 48.8° FOV
- Commands: `.../throttle_command`, `.../steering_command` (`std_msgs/Float32`, both [-1, 1])
- Steering: ±0.5236 rad, rate-limited 3.2 rad/s; top speed 22.88 m/s; wheelbase 0.324 m
- **Runtime-restricted topics (do not use while racing): IPS pose, odom, collision count, lap info, TF from sim**

## Rules digest

- Build our stack as separate ROS 2 package(s); never modify `autodrive_roboracer` devkit package or the sim.
- Submission = devkit-derived Docker container on DockerHub; everything auto-starts from `autodrive_devkit.sh` (not `.bashrc`).
- Full rules: [rules page](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-rules-2026/) · [technical guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/)
