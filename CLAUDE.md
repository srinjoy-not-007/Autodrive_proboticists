# Working in this repo

Racing stack for the [6th RoboRacer Sim Racing League @ IROS 2026](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/)
— AutoDRIVE simulator, 1:10 scale car, ROS 2 Humble, submitted as a Docker image.
Solo entry. `README.md` is the human-facing overview; this file is the operating
manual for an agent, and `docs/STATUS.md` is the live log of what has actually
been measured.

## House rules

1. **Every number in the docs came from a run.** Keep it that way — if you claim
   a lap time, a clearance or an error, it must trace to a run you did. Mark
   estimates as estimates.
2. **Drive it before you believe it.** Offline metrics have been wrong here in
   both directions: a line with healthy measured clearance still crashed every
   lap (it doubled back on itself), and a "junk" sensor turned out to be fine.
3. **When a change makes things worse in a way the physics can't explain, stop
   tuning and look at the geometry.** Halving the speed once *increased*
   collisions; that was the clue that found the real bug.
4. **Do not modify the `autodrive_roboracer` devkit package or the simulator.**
   Competition rules. Our code is a separate ROS 2 package, `pit_crew`.
5. `docs/STATUS.md` ends with a **NEXT SESSION — START HERE** section. Read it
   first; update it before you stop.

## Environment

- Two Docker images per track: `autodriveecosystem/autodrive_roboracer_{sim,api}:<tag>`.
  `2026-iros-practice` is the Porto track (28.6 m); the `*-compete` tags carry
  real past-competition circuits (`2026-icra-compete` is a 56.7 m loop) and are
  how we test on tracks we have not tuned for. Pass with `TAG=... ./scripts/...`.
- Images' ENTRYPOINT is `/bin/bash` — pass only `-c "..."`.
- Host `.venv/` holds the offline toolchain (numpy, scipy, scikit-image,
  matplotlib, `trajectory_planning_helpers`). Offline tools in `tools/` run on
  the **host** via `.venv/bin/python`; anything under `src/` runs **inside** the
  devkit container.
- The sim needs a real X display and a GPU. See the LiDAR gotcha below.

## Running things

```bash
./scripts/sim_restart_connect.sh                    # restart sim + click Connection, verify data flows
CONTROLLER=mpc ./scripts/run_stanley.sh 1.0 track_v2 safe   # the shipping config
./scripts/stop_run.sh                               # stop nodes and zero the actuators
./scripts/new_track_drill.sh <name> <sim_tag> <s>   # the whole unseen-track pipeline, timed
.venv/bin/python tools/validate_laps.py             # lap times and collisions from a run log
.venv/bin/python tools/footprint_analysis.py data/maps/<name> data/racelines/<line>.csv
.venv/bin/python tools/compare_sim_builds.py <tag_a> <tag_b>   # diff two sim images' physics/track
```

`sim_restart_connect.sh` **restarts the devkit container, which wipes the colcon
build** — every script that uses it rebuilds `pit_crew` afterwards. If you bring
nodes up by hand, rebuild first or you get "Package 'pit_crew' not found":

```bash
colcon build --packages-select pit_crew --symlink-install
```

## Simulator gotchas (each of these cost hours)

- **Headless freezes the LiDAR.** `-batchmode -nographics` keeps publishing a
  *stale* scan forever — the LiDAR is GPU-raytraced and needs real rendering.
  Run the sim windowed.
- **LiDAR arrives at ~10 Hz**, not the 40 Hz on the spec sheet. Every sensor
  shares one WebSocket at ~10 Hz, so controllers dead-reckon the pose to "now".
- **`lap_count` and `collision_count` are cumulative across runs**, resetting
  only when the sim *container* restarts. Difference them against the run's
  first row or you will misread every result. This has caused wrong conclusions
  twice.
- **Commands latch.** Publish zeros to stop.
- **`/autodrive/reset_command`** returns the car to the last checkpoint but does
  not clear a wedged vehicle — restart the sim container for that.
- Positive `steering_command` = **left**. LiDAR uses standard ROS ordering
  (index 0 = −135°, i.e. right).
- **Runtime-restricted topics** (fine for evaluation, never for control): IPS
  pose, odom, collision/lap info, sim TF.
- `pkill -f <pat>` inside `docker exec bash -c` matches the wrapper's own
  command line and kills the script — use bracket patterns (`[i]nstall/pit_crew`).
- `ros2 bag play` signals its whole process group on exit, killing sibling
  nodes started from the same shell — start those with `docker exec -d`.
- After a container restart, `ros2 daemon stop && ros2 daemon start`.

## The stack

All nodes are in `src/pit_crew/`:

| Node | Role |
|---|---|
| `follow_the_gap` | reactive, map-free. Clears qualification on any track. |
| `localization` | Monte Carlo localization against a pre-built map |
| `mpc` | **shipping controller** — kinematic-bicycle MPC in the path-relative frame |
| `stanley`, `pure_pursuit` | earlier trackers, kept for comparison |
| `odometry`, `scan_odometry` | wheel odometry; LiDAR odometry (implemented, not adopted) |
| `arbiter` | safety layer — **do not ship, see below** |
| `skidpad` | grip measurement; needs open space neither track has |

Offline pipeline (this *is* the finals drill, and `new_track_drill.sh` runs it
end to end in ~271 s measured):

```
record a recon lap → build_map_from_bag.py → extract_centerline.py
                   → make_track_lines.py   → footprint_analysis.py → race it
```

Vehicle constants live in one place, `src/pit_crew/vehicle.py`: wheelbase
0.324 m, car 0.500 × 0.270 m, max steer 0.5236 rad rate-limited to 3.2 rad/s
→ **minimum turn radius 0.561 m**. Throttle is open loop at **~0.040 per m/s**;
speed feedback comes from the wheel encoders (`position` is cumulative wheel
angle in radians).

## Where things stand

Shipping: safe centerline + MPC + full profile speed (ay cap 3.0 m/s²).

| Track | Result |
|---|---|
| Porto (practice) | 22 laps, 0 collisions, best **11.32 s** |
| ICRA (never tuned on) | 13 laps, 0 collisions, **20.9 s/lap** |
| Follow-the-Gap alone, any track | 11 laps, 0 collisions, 33.43 s |

Published winning lap on the same Porto track is **6.53 s**. We track our own
plan to within 2.4%, so the tracker is no longer the bottleneck — the gap is in
the planner. `docs/STATUS.md` has the full lever sweep.

That 6.53 s was **verified comparable** (Aug 22): the build it was set on
(`2025-cdc-tf-practice`) and ours have a byte-identical Porto mesh, identical
physics settings and an identical vehicle, checked asset-by-asset with
`tools/compare_sim_builds.py`. The gap is real, not an artefact.

## Do not re-derive these

- **The arbiter causes the collisions it was meant to prevent.** Same line,
  speed and localization: with it, 18–29 collisions and no completed laps;
  the tracker publishing directly runs clean. It flips state almost every scan.
  Needs a real state machine with dwell time before it is trustworthy.
- **The optimized min-curvature raceline loses to the plain "safe" centerline**
  on Porto — it is faster in theory (10.41 s vs 11.05 s ideal) but only leaves
  0.146 m corner clearance and collects ~57 collisions. Clearance beats path
  length when a collision costs +10 s. Better tracking did not rescue it.
- **Do not keep raising the ay cap** *by itself*. ay 4.0 gives 10.11 s but 23
  collisions, and it fails at curvature *transitions*, not the tightest corners.
  The reason matters: the tyre model peaks at **mu 1.0 (ay 9.81 m/s2) at 0.01
  slip** and halves to 0.5 (4.91 m/s2) by 0.10 slip, so our ~5 m/s2 ceiling is
  the *asymptote*, not the limit — the ay-4.0 run sat at full steering lock 20%
  of samples. More grip is there, but only for a line and controller that hold
  slip inside that window. Raising the cap without fixing that just crashes.
- **Do not re-tune the centering strength** of the centerline extractor —
  harder centering makes the line jagged and clearance *worse*.
- `--kappa-bound` in `optimize_raceline.py` does nothing on Porto; the corridor
  binds first.
- `skidpad.py`'s Porto numbers are contaminated (the car grinds the walls once
  it slides) — do not cite them.
