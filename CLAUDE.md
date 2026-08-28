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
5. `docs/STATUS.md` **begins** with a **NEXT SESSION — START HERE** section
   (the rest of the file is append-only history, oldest first). Read it first;
   update it before you stop.
6. **Run `./scripts/check_pipeline.sh` before any commit that touches `tools/`
   or `src/tools/`.** It regenerates every track and fails if one stops
   producing a drivable line, or if a shipping line moves a byte. A change
   validated on Porto once shipped having broken ICRA completely.

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
VISIBLE=1 ./scripts/sim_restart_connect.sh          # ...on the real display, to watch a run (steals the pointer)
CONTROLLER=mpc ./scripts/run_stanley.sh 1.0 track_v2 safe   # the shipping config
./scripts/stop_run.sh                               # stop nodes and zero the actuators
./scripts/new_track_drill.sh <name> <sim_tag> <s>   # the whole unseen-track pipeline, timed
.venv/bin/python tools/validate_laps.py             # lap times and collisions from a run log
.venv/bin/python tools/footprint_analysis.py data/maps/<name> data/racelines/<line>.csv
.venv/bin/python tools/compare_sim_builds.py <tag_a> <tag_b>   # diff two sim images' physics/track
./scripts/check_pipeline.sh                         # regenerate all tracks, fail on any drift
.venv/bin/python tools/grip_curve.py <race log>     # achieved vs asked curvature from any run
.venv/bin/python tools/sweep_lines.py <track>       # corridor x grip grid, ranked
.venv/bin/python tools/open_corners.py <centerline> --out X   # maximise the smallest radius
```

**Measuring grip properly needs the `-explore` image** — it has no track active
at all, only `Floor`, which is the open space `skidpad.py` was written for:

```bash
TAG=2026-iros-explore ./scripts/sim_restart_connect.sh
ros2 run pit_crew skidpad --ros-args -p steer:=0.20 -p v_max:=4.5
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
- **The sim runs on Xvfb `:99` inside its own container, and that is faster.**
  Not the frozen-LiDAR trap: Xvfb is a real X server, Unity really renders, and
  the raytracer goes through CUDA — measured 10.97 Hz, ten consecutive scans all
  distinct. It also stops the sim competing with the desktop compositor for the
  GPU, worth **~0.4 s/lap**: the identical shipping config measured 101.2 s on
  Aug 23 and **97.2 s** on Aug 28. **Never compare a lap time against a
  pre-Aug-28 number** — only against one measured in the same session.
- **There is no API to connect the sim; the button must be clicked.** Verified,
  do not re-derive: `-ip`/`-port` only pre-fill the fields (zero
  `WebSocket.Connect` attempts in the Unity log without a click), PlayerPrefs
  hold only window geometry, and Unity ignores `xdotool --window` (XSendEvent).
  Unity's UI also needs the pointer to *enter* the control and needs the press
  and release in separate frames, so `mousemove away; mousemove on; mousedown;
  sleep; mouseup` lands where `xdotool click` silently misses.
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

**Aug 23 correction — this overrides the older ceiling claims below.** Official
winner onboard telemetry demonstrates roughly **8.6-9.5 m/s2** lateral
acceleration on the same Porto vehicle/physics, and its sampled path is close to
the optimized TUM line. Our inability to use that envelope is a stack limitation,
not proof that the envelope does not exist.

A controlled replay also disproved the claim that lateral localization error is
structurally stuck near 0.17 m. Keeping the LiDAR's physical 0.1933 m forward
offset consistent in both map construction and localization reduced offline
lateral p95 to **0.041 m** at the original process noise, or **0.039 m** with
0.035 m noise. Low noise on the old zero-offset map regressed total p95 to
0.370 m. The combined candidate ran 17 live laps without collision (lateral p95
0.047 m, best 9.89 s), but is **not shipping yet** because it has not passed
ICRA or beaten 9.81 s. See the first section of `docs/STATUS.md` for the exact
TODO and validation gate.

Shipping: safe centerline + MPC, **ay 5.0** / **ax 6.5** / brake 4.0 / jerk 20,
with the controller's **`ay_limit`** capping steering by the grip that exists
and **`brake_gain` 0.3** giving it actual braking. Ten-lap race time
115.0 -> 101.2 -> **91.4 s** (Porto) and **165.3 s** on ICRA. Of that, ~4 s came
from the Xvfb measurement change and ~5.8 s from the brakes.

| Track | Result (Aug 28, ay 5.0 + braking) |
|---|---|
| Porto (practice) | 40 laps, 0 collisions, best **9.01 s**, median 9.14 s, race **91.4 s** |
| ICRA (never tuned on) | 29 laps, 0 collisions, median **16.53 s**, race 165.3 s |
| Follow-the-Gap alone, any track | 11 laps, 0 collisions, 33.43 s |

Published winning lap on the same Porto track is **6.53 s** (65.41 s race).
We track our own plan to within ~5%, so the tracker is not the bottleneck —
**and neither is the planner, the tyre, or the grip cap.**

**SUPERSEDED (Aug 28) — the ceiling was the missing brakes.** Every one of the
four "independent" demonstrations was run while the controller had **no braking
authority**: `ThrottleController`'s braking branch was gated behind `kp > 0`,
which nothing sets, so every throttle command was positive and the car shed
speed only by coasting. It could plan a corner-entry speed it had no way to
reach. ay 4.0 (23 collisions) and ay 5.0 (31) are exactly what that looks like.
With braking enabled the same profiles run **perfectly clean**, and the shipping
cap moved 3.5 -> **5.0**. See `docs/reviews/deepak-g.md`.

There IS still a ceiling, but it is real and further out: ay 6.0 turns back
(9.44 s against ay 5.0's 9.14 s) because past ~5 the car cannot deliver the
planned curvature and runs wide. The grip-cliff and 10 Hz-sensing facts below
remain true; they were just not what was binding.

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
- **SUPERSEDED: "do not keep raising the ay cap".** True only without brakes;
  ay 5.0 now runs clean and ships. The original note, kept because the tyre
  measurements in it are still correct: ay 4.0 gives 10.11 s but 23 collisions;
  capping lateral jerk at 20 m/s3 as well cuts that to 10 (best lap 9.96 s) but
  still is not clean. **Measured grip, from the race trace and not the asset:**
  the car turns exactly as the bicycle model predicts below 0.10 rad of
  steering, saturates past ~0.15 rad, and achieved lateral acceleration
  plateaus at **4.5 m/s2** — the tyre's asymptote, not its 9.81 peak (that peak
  sits at 0.01 slip, too narrow a window to hold). Real headroom over ay 3.0 is
  ~1.5x. `tools/grip_curve.py <race log>` re-measures this from any run.
- **Staying inside the linear tyre region means planned curvature under
  0.46 1/m** (0.15 rad of steer). Porto's tightest corner is 1.019, so it cannot
  be taken inside that region without using track width to open it — which
  costs the clearance that keeps us collision-free. That trade, not the ay cap,
  is the real constraint on lap time.
- **Steering authority is not constant — it is `atan(L * ay_max / v^2)`.** The
  controller caps its own command by what the tyres can pay for at the current
  speed (`ay_limit`, 4.5 m/s²). Without it, it commanded 0.202 rad at 5.45 m/s —
  a demand for ~15 m/s² — slid wide and hit a wall while 0.054 m from its own
  line. With it, steering never reaches full lock at all (15.6% of samples -> 0%)
  and collisions went 4 -> 0. Do not remove it to "get more steering".
- **Longitudinal grip is 7.1 m/s², not 3.0.** Longitudinal mu is 0.72 and
  acceleration happens where there is no lateral demand, so `--ax 6.5` is nearly
  free (ideal lap 11.0 -> 10.1 s). Braking stays at 4.0: it overlaps corner
  entry. `PROFILE` in `make_track_lines.py` holds the validated set.
- **Do not put the understeer curve in the controller's rollout.** Measured: it
  raises collisions per lap from 0.50 to 0.88 and time at full lock from 16% to
  36%. The controller cannot slow down — the profile is fixed offline — so
  telling it the tyres deliver less only makes it command more lock, deeper into
  saturation. `vehicle.achieved_curvature()` is correct and kept; it belongs in
  the **planner**, capping planned curvature at what the car can achieve.
- **SUPERSEDED: "on Porto the geometry is settled" was wrong, but the
  conclusion survives for a different reason.** That claim came from
  `tools/sweep_lines.py`, whose grid stopped at `w_veh 0.32` and which ties
  `--ax` to `--ay` instead of using the shipping profile. Swept properly, the
  optimized line beats the centerline by **0.31 s** of ideal lap for **13%** of
  its clearance (w_veh 0.44: 0.283 m, 9.28 s vs 0.325 m, 9.59 s), not 0.05 s for
  44%. `make_track_lines.py` now sweeps `w_veh` per track.
  **It still loses when driven**: 9.43 s median but 4 collisions in 38 laps, all
  at its own tightest point, against 9.72 s clean — race 104.8 s vs 97.2 s. The
  corrected-LiDAR map did not rescue it. Tighter lines also *track* worse:
  driven/ideal 4.2% against the safe line's 1.4%.
- **Offline ideal lap time mispredicts the driven ranking.** `race.csv` is
  chosen by fastest ideal lap above `MIN_CLEARANCE`, and on Porto that rule
  picked the line that drives *slower*. `MIN_CLEARANCE` is **0.30 m**, set from
  what collides (0.283) and what does not (0.301, 0.325) — not from an error
  budget. Do not lower it without driving what the lower floor admits.
- **Do not re-tune the centering strength** of the centerline extractor —
  harder centering makes the line jagged and clearance *worse*.
- `--kappa-bound` in `optimize_raceline.py` does nothing on Porto; the corridor
  binds first.
- `skidpad.py`'s *Porto* numbers are contaminated — but the tool works fine in
  the **`-explore`** world, which has no track at all. Measured there: the tyre
  holds ~0.9 of asked curvature to **9.9 m/s2**, then collapses to 0.24-0.35
  within one 0.5 m/s step. **That grip is real and still unusable on track** —
  planning ay 5.0 gives 31 collisions, because breakaway is abrupt and we know
  our position to only 0.17 m.
- **The grip watchdog works and loses.** Closed-loop breakaway detection
  (`grip_watchdog`) cuts collisions 31 -> 3 at ay 5.0 and 2 -> 0 at ay 4.0, and
  costs more race time than it saves in *every* configuration — at the shipping
  cap it fires on a clean run and even caused a collision. Default off.
- **SUPERSEDED: lateral localization error is not fixed at 0.17 m.** Particle
  count and likelihood sharpening did not help, but those sweeps retained an
  inconsistent zero-offset map. Correcting the LiDAR origin in both the map and
  runtime model cuts lateral p95 to 0.039-0.047 m. Do not lower motion noise on
  the old map; that combination regresses badly.
- **`offline_pf.py` does not reproduce live behaviour** — 192 relocalizations
  where the live filter has zero. Do not tune against it; drive instead.
- **Recon trajectories contain teleports** (3 in the ICRA recon, 58 in
  `maplap_short`) because a recon run collides. Two attempts to make centerline
  tracing robust to them were reverted for breaking the shipping ICRA line.
