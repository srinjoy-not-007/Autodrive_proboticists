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
scripts/     helper scripts (pull images, run sim/devkit, race, smoke test)
src/         our ROS 2 package (mounted into the devkit container at src/pit_crew)
tools/       offline pipeline run on the host via .venv (maps → lines → analysis)
             incl. grip_curve.py, sweep_lines.py, open_corners.py,
             compare_sim_builds.py
data/        maps and racelines (tracked); bags, logs and screenshots (git-ignored)
docs/        PLAN.md — strategy · STATUS.md — every measured result, and what's next
submission/  Dockerfile + entrypoint for the competition image
CLAUDE.md    operating manual for AI coding agents working in this repo
upstream/    clone of the official competition repo (git-ignored)
```

## Quick start

```bash
./scripts/check_pipeline.sh           # regenerate every track, fail on any drift
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
- Steering: ±0.5236 rad, rate-limited 3.2 rad/s; top speed 22.88 m/s; wheelbase
  0.324 m per the datasheet (the simulator scene measures 0.3312 m — see STATUS)
- **Runtime-restricted topics (do not use while racing): IPS pose, odom, collision count, lap info, TF from sim**

## Running the stack

```bash
CONTROLLER=mpc ./scripts/run_stanley.sh 1.0 track_v2 safe   # the shipping config
./scripts/record_mapping_run.sh                     # slow FTG lap, recorded for mapping
./scripts/stop_run.sh                               # stop everything, zero the actuators
```

Shipping config = safe centerline + MPC, ay 3.5 / ax 6.5 / brake 4.0, with the
controller capping its own steering by the lateral grip that exists at the
current speed. Measured:

| Track | Result |
|---|---|
| Porto (practice) | **27 laps, 0 collisions**, best **9.81 s**, 10-lap race **101.2 s** |
| ICRA — never tuned on | **16 laps, 0 collisions**, median **17.95 s/lap** |
| Follow-the-Gap alone, any track | 11 laps, 0 collisions, 33.43 s — clears qualification |

The published winning lap on this same Porto track is 6.53 s (65.41 s race).
The remaining gap is not the planner or the controller: the tyre holds ~0.9 of
the curvature asked of it up to 9.9 m/s², then breaks away within one 0.5 m/s
step, and we know our position to ~0.17 m. Plan closer to that cliff and the
speed is clipped — by a wall, or by backing off. `docs/STATUS.md` has the four
independent demonstrations.

`run_race_stack.sh` also exists but starts the arbiter, which measurably makes
things *worse* — see `docs/STATUS.md`.

New track, end to end (this is the finals drill) — one command, ~271 s measured:

```bash
./scripts/new_track_drill.sh <name> <sim_tag> <record_seconds>
CONTROLLER=mpc ./scripts/run_stanley.sh 1.0 <name> safe        # then race it
```

It drives a blind recon lap with Follow-the-Gap, builds the map **both ways**
(with and without the LiDAR mount correction), and runs `make_track_lines.py` on
each — which picks the centerline method by measured clearance, rejects any line
that reverses direction or turns tighter than the steering lock, applies the
validated speed profile, and keeps whichever map produced the faster drivable
line. If neither does, it aborts loudly rather than shipping something ungated.

Do not call `extract_centerline.py` / `optimize_raceline.py` directly for a race
line: their defaults skip every one of those checks. That bypass was in this
drill until Aug 23 and would have produced a slower, ungated line at the finals.

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
- **Wheel-encoder speed works** — `position` is cumulative wheel angle in
  radians, so speed = `wheel_radius * d(position)/dt` (scale 0.993/0.999 on two
  tracks). An earlier note here said otherwise; that data was recorded while the
  car was stuck and reversing. Throttle actuation stays open loop
  (**~0.040 throttle per m/s**, measured).
- **`lap_count` and `collision_count` are cumulative across runs**, resetting
  only when the sim *container* restarts. Difference them against the run's
  first row.
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
