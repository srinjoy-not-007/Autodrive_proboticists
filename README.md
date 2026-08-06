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

## Running the stack

```bash
./scripts/run_race_stack.sh 0.30 track_v2 safe      # localization + raceline + FTG + arbiter
./scripts/run_race_stack.sh 0.45 track_v2 raceline  # the fast line
./scripts/record_mapping_run.sh                     # slow FTG lap, recorded for mapping
./scripts/stop_run.sh                               # stop everything, zero the actuators
```

New track, end to end (this is the finals drill):

```bash
./scripts/record_mapping_run.sh                       # drive + record a clean lap
./scripts/stop_run.sh
docker exec autodrive_roboracer_api /bin/bash -c \
  "python3 /home/autodrive_devkit/src/pit_crew/tools/build_map_from_bag.py \
   /home/data/bags/<BAG> /home/data/maps/<NAME>"
.venv/bin/python tools/extract_centerline.py data/maps/<NAME> data/racelines/<NAME>_centerline
.venv/bin/python tools/optimize_raceline.py data/racelines/<NAME>_centerline.csv          # fast line
.venv/bin/python tools/optimize_raceline.py data/racelines/<NAME>_centerline.csv \
    --centerline-only --out data/racelines/<NAME>_safe                                    # safe line
.venv/bin/python tools/footprint_analysis.py data/maps/<NAME>   # check corner clearance!
```

## Submission

```bash
docker build --build-arg TAG=2026-iros-compete -t <user>/roboracer:iros2026 \
    -f submission/Dockerfile .
docker push <user>/roboracer:iros2026
```

The image builds FROM the official devkit, leaves `autodrive_roboracer`
untouched, and starts everything from `autodrive_devkit.sh` (never `.bashrc`),
as the rules require. `PIT_CREW_MODE=reactive` selects the map-free
Follow-the-Gap stack — the safe entry for a track we have not validated on.

## Platform gotchas (learned the hard way — read before debugging)

- **Headless sim freezes the LiDAR.** `-batchmode -nographics` keeps publishing a
  *stale* scan; the LiDAR is GPU-raytraced and needs real rendering. Run the sim
  windowed via `scripts/sim_restart_connect.sh` (it also clicks *Connection*).
- **LiDAR arrives at ~10 Hz**, not the 40 Hz in the spec sheet.
- **Wheel-encoder speed is unusable.** The encoder angle aliases at the publish
  rate — correlation with true speed is |r| < 0.2 however you unwrap it. Throttle
  runs open loop (**~0.040 throttle per m/s**, measured); real speed feedback must
  come from differentiating the localization pose.
- **Commands latch.** Publish zeros to stop; a stale command keeps being applied.
- **`/autodrive/reset_command`** returns the car to the last checkpoint and does
  not clear a wedged/beached vehicle — restart the sim container for that.
- **`pkill -f` inside `docker exec bash -c`** matches the wrapper's own command
  line and kills the script. Use bracket patterns (`[i]nstall/pit_crew`) or PIDs.
- **`ros2 bag play` signals its process group** on completion, killing sibling
  nodes launched from the same shell — start those with `docker exec -d`.
- Humble's `ros2 bag play` has **no duration flag**; use `src/tools/trim_bag.py`.

## Rules digest

- Build our stack as separate ROS 2 package(s); never modify `autodrive_roboracer` devkit package or the sim.
- Submission = devkit-derived Docker container on DockerHub; everything auto-starts from `autodrive_devkit.sh` (not `.bashrc`).
- Full rules: [rules page](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-rules-2026/) · [technical guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/)
