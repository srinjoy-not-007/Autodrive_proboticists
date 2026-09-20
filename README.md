# AutoDrive — RoboRacer Sim Racing League @ IROS 2026

Team workspace for the [6th RoboRacer Sim Racing League @ IROS 2026](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/).

**Objective:** drive fast, don't crash. Qualification = 10 clean laps on the Porto practice track. Finals = 10-lap time attack on a track released 3 days before submission. Collisions cost +10 s each (cumulative); >10 collisions = DQ.

## Key dates (2026)

| Date | Event |
|---|---|
| Sep 6 | **Registration closes** ([register here](https://forms.gle/MiuoSMYQ4z6QACLS6)) |
| Sep 12 | Phase 1 (qualification) submission deadline |
| Sep 16 | Competition track released — **SRL 2026 IROS Track**, sim/api tag `2026-iros-compete` |
| Sep 19 | Phase 2 (final race) submission deadline — AoE, i.e. **Sep 20, 17:30 IST** |

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

**Sep 20, 13:00 IST — the finals entry is image v10:** mpc3 on
`finals_mesh_apex_f42k_z36q_race.csv` (the apexing min-curvature line with the
s 36-40 speed zone released a quarter of the way to the ay-9 profile),
`barrier_margin` 0.34 with a 0.44 window at the hairpin-2 entry, the publish
gate, the period-gated exit servo and straight-line pure-pursuit blend, and the
0.10 s steering command filter modelled inside the solver's rollout above a
measured period of 0.080 s. **100 laps / 0 collisions at both sensor periods:
108.25 s on a real display (0.058 s) and 111.24 s on Xvfb (0.096 s, ~ the
organisers' period)**; image tests 111.18 / 111.32 / 108.39 s, 12 / 0 each; 64
laps at the slow end (0.103 s) with one collision at the s 25 exit. The
display-optimised alternative (v11: 0.10 m more inside margin at hairpin 2 so
the lag model can run at every period) is 107.62 s on the display but 112.74 s
in band, both 100 / 0, and is not shipped. `docs/FINALS_TUNING.md` has every
run of the last 20 hours: open hairpins, more throttle (wheelspin), later
braking, apex caps, joint MPC, all driven and rejected with their mechanisms.
The paragraph and table below are the Sep 18 state (114.04 / 119.51 s), kept
for the record.

Shipping config (Sep 18, finals) = **mpc3** on `finals_mesh_race.csv`, the SRL
2026 IROS finals track (`2026-iros-compete`), with `rate_adaptive` (timing terms
scaled by the measured sensor period), `w_dcmd` 50 + `cmd_tau` 0.10 (steering
limit-cycle damping), `steer_tau` 0.04 (rack lag in the rollout), a corridor
barrier at `barrier_margin` 0.40 with `thr_brake_lead_s` 0.05, and — the change
that made the real display clean — **`brake_mode:=zero`** (`brake_a_min` 1.5,
`brake_dv` 0.3, `brake_a_full` 4.6). The simulator applies its brake torque only
when the throttle command is exactly 0.0, and this stack had never published it;
without that, every finals configuration took 10 collisions in 11-14 display
laps, which is a DQ. Localization runs `max_step_mps` 9.0 with
`publish_on_reloc` off. Everything ships as an uncommented default in
`submission/autodrive_devkit.sh`; `PIT_CREW_MAP=/home/data/maps/porto_mesh`
still races the qualifier. The sensor period the organisers' box delivers is
unknown, so every number below is quoted with the period it was measured at
(OFFICIAL race = simulator lap timer + 10 s/collision):

| Track | Real display, 0.058 s | Xvfb, 0.09-0.10 s |
|---|---|---|
| **SRL 2026 IROS (finals)** | **32 laps, 0 collisions, 114.04 s** (113.98 s on a second sample) | **32 laps, 0 collisions, 119.51 s**; the built image raced the organisers' way, 12 laps, 0 collisions, 119.57 s |
| Porto (qualifier) | **32 laps, 0 collisions, 70.13 s** (69.36 s at margin 0.34) | **100 laps, 0 collisions, 71.91 s** (75.08 s on the finals args) |
| SRL 2025 CDC-TF — never tuned on | — | 32 laps, **2** collisions, 101.9 s (fast line; the roomier `fast3` line is collision-free but missed a checkpoint once) |
| SRL 2025 ICRA — never tuned on | — | 32 laps, 0 collisions, **140.4 s** (fast line) |
| ICRA 2026 — never tuned on | — | 32 laps, 0 collisions, 148.3 s |
| Legacy `mpc` on the safe line (fallback) | never driven | 32 laps, 0 collisions, 78.4 s |

`docs/FINALS_TUNING.md` is the finals state page — frozen config, the mechanism,
every arm driven and the submission commands; `docs/FINALS_RUNBOOK.md` is the
procedure.

Baseline on `main` is 89.6 s (Porto). The display, not Xvfb, is the gate: on
Porto four real-display runs of earlier configurations failed identically
(10-11 collisions, always the same hairpin entry) before the corridor barrier,
and on the finals track three more failed the same way before the brake. A run
is evidence only together with its `dt_p50`.

The fastest published lap on this same Porto track is **6.46 s** (VAUL, ICRA
2026 qualification), for a ~65 s race. Our median display lap is 6.94-7.00 s and
our best 6.87 s, so the gap is now ~7 %. It is not the line — 76 swept
candidates span 0.10 s/lap — and not the tracker: of the plan-side gap, ~90 % is
the lateral-acceleration cap (planning at the winners' ay 9.0 alone takes the
ideal lap 7.04 → 6.35 s) and ~10 % is longitudinal, where the simulator's own
speed servo saturates at 4.2-4.7 m/s² against the profile's 6.5. See
`docs/STATUS.md`, "NEXT SESSION".

Until 2026-08-28 this section said the remaining gap was a grip cliff, and that
the ceiling had been "demonstrated four independent ways". That was wrong: all
four were measured while the controller had **no braking authority** — the
braking branch was unreachable, so the car shed speed only by coasting while the
velocity profile assumed 4 m/s² of deceleration. Fixing it took the usable
lateral-acceleration cap from 3.5 to 5.0 (later re-tuned to 4.5 alongside the
jerk-limiter fix) and the race from 97.2 to 91.4 s. `docs/reviews/deepak-g.md`
has the ladder.

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
# finals (default assets are the SRL 2026 IROS track)
docker build --platform linux/amd64 --build-arg TAG=2026-iros-compete \
    -t bongracers/roboracer:iros2026-finals -f submission/Dockerfile .
docker push bongracers/roboracer:iros2026-finals

# qualification image, for reference
docker build --platform linux/amd64 --build-arg TAG=2026-iros-practice \
    -t bongracers/roboracer:iros2026-qualification -f submission/Dockerfile .
```

See [submission/README.md](submission/README.md) for the deadline, exact run
command, DockerHub overview, and acceptance procedure; the finals sequence is in
[docs/FINALS_TUNING.md](docs/FINALS_TUNING.md) section 6.

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
