# Defects

Verified problems in the repo. Everything here was either **reproduced by
running it** or read directly out of the source — nothing is inferred.

Raised 2026-08-25 during a full read of the codebase plus a live run of the
stack on macOS. Line references are against commit `8074658`.

| # | Severity | Where | One line |
|---|---|---|---|
| [1](#1-a-physically-undrivable-raceline-is-committed-and-shipped) | **critical** | `data/racelines/icra2026_raceline.csv` | Undrivable line committed and baked into the submission image |
| [2](#2-make_track_linespy-never-validates-the-raceline-it-writes) | **critical** | [`tools/make_track_lines.py:104`](../tools/make_track_lines.py#L104) | The optimized raceline is written with no validation at all |
| [3](#3-the-finals-drill-bypasses-the-line-selection-tool) | **high** | [`scripts/new_track_drill.sh:48`](../scripts/new_track_drill.sh#L48) | Drill produces the worse centerline and skips the drivability check |
| [4](#4-stop_runsh-corrupts-the-collision-counter) | **high** | [`scripts/stop_run.sh:4`](../scripts/stop_run.sh#L4) | Actuators zeroed *after* nodes are killed; commands latch |
| [5](#5---brake-is-dead-code) | medium | [`tools/optimize_raceline.py:69`](../tools/optimize_raceline.py#L69) | `--brake` declared, never used |
| [6](#6-vehicle-mass-contradicts-the-measured-value) | medium | [`tools/optimize_raceline.py:158`](../tools/optimize_raceline.py#L158) | `m_veh=3.906` vs a measured 3.47 kg |
| [7](#7-the-drill-clearance-check-verifies-the-wrong-file) | medium | [`scripts/new_track_drill.sh:59`](../scripts/new_track_drill.sh#L59) | Checks the recon drive, not the generated line |
| [8](#8-unpinned-scipy-pulls-numpy-2x-into-the-submission-image) | medium | [`submission/Dockerfile:11`](../submission/Dockerfile#L11) | Non-deterministic submission build |
| [9](#9-smoke_testsh-cannot-detect-what-it-claims-to-test) | low | [`scripts/smoke_test.sh:7`](../scripts/smoke_test.sh#L7) | Headless sim + wrong expected rate |
| [10](#10-retracted-docstrings-still-state-the-retracted-claim) | low | [`src/pit_crew/vehicle.py:70`](../src/pit_crew/vehicle.py#L70) | Docstrings contradict the code directly above them |

---

## 1. A physically undrivable raceline is committed and shipped

**Where:** `data/racelines/icra2026_raceline.csv`

**Problem.** The line demands a turn radius the car does not have, and reverses
direction twice.

The worst curvature, rows 304–306 of the file
(`s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2`):

```csv
30.1963,-2.3897,3.4894,2.4032,2.6983,0.9906,0.0000
30.2963,-2.4668,3.4331,1.8498,3.0571,0.9906,0.0000    <-- |kappa| 3.057 -> R = 0.327 m
30.3963,-2.5384,3.4390,-0.4793,3.0571,0.9906,0.0000
```

`R = 0.327 m` against a steering lock that allows **0.561 m** — the car cannot
turn this tightly at any speed.

And the two direction reversals, rows 355–356, where consecutive points flip
heading by more than 90°:

```
row 355: heading flips 128.6 deg between consecutive points
row 356: heading flips 133.7 deg between consecutive points
```

| Measured | Value | Limit |
|---|---|---|
| peak \|κ\| | **3.057 rad/m** → R = **0.327 m** | lock allows **0.561 m** |
| direction reversals | **2** | must be 0 |
| worst-corner clearance | **0.079 m** | safe line achieves 0.564 m |

A direction reversal is the exact failure that produced 83 collisions in one
ICRA run (`STATUS.md`, "Unseen track (ICRA): SOLVED") — the car is commanded to
drive somewhere, stop, and reverse, and it wedges there every lap.

**Why it matters.** The file is committed, and
[`submission/Dockerfile:20`](../submission/Dockerfile#L20) copies the whole
directory into the competition image:

```dockerfile
COPY data/maps /home/data/maps
COPY data/racelines /home/data/racelines
```

Any run with `PIT_CREW_LINE` pointed at it fails immediately.

**Reproduce.**
```bash
.venv/bin/python -c "
import sys; sys.path.insert(0,'tools')
from make_track_lines import drivability
print(drivability('data/racelines/icra2026_raceline.csv'))"
# ['2 direction reversal(s)', 'min radius 0.327 m < 0.561 m lock']
```

**Fix.** Delete the file, and fix defect #2 so it cannot be regenerated.

---

## 2. `make_track_lines.py` never validates the raceline it writes

**Where:** [`tools/make_track_lines.py:104-108`](../tools/make_track_lines.py#L104)

**Problem.** The tool's entire purpose is choosing a line **by measurement**. It
does exactly that for the safe line — note the clearance check, the drivability
check, and the `REJECTED` path:

```python
        try:
            wc = worst_corner(mp, safe + '.csv')          # <-- clearance checked
        except Exception as e:
            print(f'  {tag}: clearance check failed ({e})')
            continue
        bad = drivability(safe + '.csv')                  # <-- drivability checked
        if bad:
            print(f'  {tag:9s} REJECTED: {"; ".join(bad)}')
            continue
```

Then, at the end of `main()`, the optimized raceline is generated and written to
disk with **neither check applied**:

```python
    # optimized raceline from the chosen centerline
    subprocess.run([VENV, 'tools/optimize_raceline.py',
                    f'data/racelines/{track}_centerline.csv',
                    '--out', f'data/racelines/{track}_raceline'],
                   capture_output=True, text=True)     # <-- no check=True, no validation
    print(f'wrote data/racelines/{track}_{{centerline,safe,raceline}}.csv')
```

Two problems in those four lines:

- **No `worst_corner()` or `drivability()` call** — the raceline is trusted
  unconditionally, which is what produced defect #1.
- **`capture_output=True` with no `check=True`** — a non-zero exit is silently
  discarded, so a failed generation prints the same success message as a good
  one.

**Why it matters.** This will recur on the finals track. The safety gate already
exists in this file and is simply not applied to one of its two outputs.

**Fix.** Run both checks on the raceline; refuse to write it (or write it under a
`_REJECTED` name) when it fails, and print why. Check the return code.

---

## 3. The finals drill bypasses the line-selection tool

**Where:** [`scripts/new_track_drill.sh:47-56`](../scripts/new_track_drill.sh#L47)

**Problem.** Both `CLAUDE.md` and `docs/STATUS.md` describe the drill as
`extract_centerline → make_track_lines → footprint_analysis`. The script never
calls `make_track_lines.py`:

```bash
stage "3/5 centerline"
.venv/bin/python tools/extract_centerline.py \
    "data/maps/${TRACK}" "data/racelines/${TRACK}_centerline" || exit 1
#                                    ^^^ no --skeleton flag

stage "4/5 racing lines"
.venv/bin/python tools/optimize_raceline.py \
    "data/racelines/${TRACK}_centerline.csv" --out "data/racelines/${TRACK}_raceline"
.venv/bin/python tools/optimize_raceline.py \
    "data/racelines/${TRACK}_centerline.csv" --centerline-only \
    --out "data/racelines/${TRACK}_safe" || exit 1
```

Without `--skeleton`, `extract_centerline.py` takes the trajectory-anchored
branch — skeleton is opt-in
([`extract_centerline.py:263`](../tools/extract_centerline.py#L263)):

```python
    if '--skeleton' in sys.argv:
        ...
    else:
        pts = centerline_from_trajectory(tx, ty, corridor, res, ox, oy)
```

**Why it matters.** Two consequences, both bad on an unseen track:

- **The worse line.** On Porto the anchored method gives 0.079 m worst-corner
  clearance against the skeleton's 0.325 m. Both shipping lines in `data/` are
  byte-identical to their `_skeleton` variants — `make_track_lines` chose
  skeleton on both tracks, by measurement. The drill would silently pick the
  other one.
- **No drivability gate.** `drivability()` lives inside `make_track_lines.py`,
  so the drill never runs the spur/steering-lock check — the check written
  specifically because a spur cost every lap of an ICRA run.

This is the procedure that runs under a three-day clock on a circuit nobody has
seen, which makes it the highest-consequence defect after #1 and #2.

**Fix.** Replace stages 3–4 with a single `make_track_lines.py <track>` call.

---

## 4. `stop_run.sh` corrupts the collision counter

**Where:** [`scripts/stop_run.sh:4-8`](../scripts/stop_run.sh#L4)

**Problem.** The nodes are killed first, and the actuators are zeroed **two
seconds later**:

```bash
docker exec autodrive_roboracer_api /bin/bash -c "
    pkill -f '[i]nstall/pit_crew/lib'; pkill -f '[r]ace_logger'; ...   # 1. kill the controller
    sleep 2;                                                          # 2. car still driving
    source /opt/ros/humble/setup.bash &&
    timeout 3 ros2 topic pub -r 20 .../throttle_command ... '{data: 0.0}'   # 3. finally stop
    timeout 2 ros2 topic pub -r 20 .../steering_command ... '{data: 0.0}'
```

Commands **latch** in this simulator, so through step 2 the car keeps driving on
its last steering and throttle command with nothing steering it. Note the logger
is killed in step 1 too, so whatever happens next is unrecorded.

**Observed 2026-08-25.** During a Follow-the-Gap run the collision counter read
1 with the logger active, and 12 immediately after `stop_run.sh` completed. The
11 additional collisions are unattributable — they may be shutdown artifacts,
real, or a mixture.

**Why it matters.** Every run's headline number passes through this script.
Collisions are the primary success metric for the whole project, and >10 is a
disqualification. A shutdown that can silently add them makes results
untrustworthy in both directions.

**Fix.** Zero the actuators **first**, then kill the nodes, then zero again.

**Workaround until fixed.**
```bash
docker exec autodrive_roboracer_api /bin/bash -c \
  "source /opt/ros/humble/setup.bash && \
   timeout 2 ros2 topic pub -r 20 /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 '{data: 0.0}'"
./scripts/stop_run.sh
```

---

## 5. `--brake` is dead code

**Where:** [`tools/optimize_raceline.py:69`](../tools/optimize_raceline.py#L69)

**Problem.** The flag is declared:

```python
    ap.add_argument('--vmax', type=float, default=8.0)
    ap.add_argument('--ay', type=float, default=3.0)
    ap.add_argument('--ax', type=float, default=3.0)
    ap.add_argument('--brake', type=float, default=4.0)     # <-- never read again
```

`args.brake` appears **nowhere else in the file** — line 69 is the only
occurrence of the string `brake`. The `ggv` that actually sets the limits uses
`args.ax` for the longitudinal column:

```python
    # Velocity profile from a simple friction-circle ggv
    v_steps = np.arange(0.0, args.vmax + 0.5, 0.5)
    ggv = np.column_stack([v_steps,
                           np.full_like(v_steps, args.ax),   # <-- accel AND braking
                           np.full_like(v_steps, args.ay)])
```

So **braking deceleration is silently equal to the acceleration limit** (3.0 by
default), not the 4.0 the flag implies.

**Why it matters.** Anyone tuning braking gets no effect and no warning, and
would reasonably conclude braking has no influence on lap time. Braking distance
is most of what a velocity profile is made of, so this is a misleading dead end
in the middle of the planner.

**Fix.** Wire it into the ggv as a separate deceleration limit, or delete the
flag. Wiring it in is the more useful option — the tyre model gives longitudinal
μ 0.72, i.e. **ax up to 7.1 m/s²**, well above the 3.0 currently used for both.

---

## 6. Vehicle mass contradicts the measured value

**Where:** [`tools/optimize_raceline.py:155-158`](../tools/optimize_raceline.py#L155)

**Problem.**

```python
    vx = tph.calc_vel_profile.calc_vel_profile(
        ggv=ggv, ax_max_machines=ax_machines, v_max=args.vmax,
        kappa=kappa, el_lengths=el_lengths, closed=True,
        drag_coeff=0.0, m_veh=3.906)                       # <-- where does 3.906 come from?
```

`tools/compare_sim_builds.py` read the actual Unity rigidbody and found
**mass 3.47 kg** (`docs/STATUS.md` line 404, in the verified build-comparison
table). 3.906 has no stated source anywhere in the repo.

**Why it matters.** A 12.6% error against a number that has been measured
directly out of the simulator asset, and a violation of the repo's own house
rule that every number must trace to a run. With `drag_coeff=0.0` the effect on
a friction-limited profile is small — but "small" is an untested assumption, and
the correct value is known.

**Fix.** Use 3.47, ideally as a constant in `src/pit_crew/vehicle.py` so there is
one source of truth.

---

## 7. The drill clearance check verifies the wrong file

**Where:** [`scripts/new_track_drill.sh:58-60`](../scripts/new_track_drill.sh#L58)

**Problem.**

```bash
stage "5/5 footprint clearance check"
.venv/bin/python tools/footprint_analysis.py "data/maps/${TRACK}" \
    "data/maps/${TRACK}_traj.csv"          # <-- the RECON DRIVE, not the racing line
```

`<track>_traj.csv` is the reconnaissance drive — the blind Follow-the-Gap lap
used to build the map. The racing line the car is about to follow is never
passed in.

Compare the documented usage in `CLAUDE.md`:

```
.venv/bin/python tools/footprint_analysis.py data/maps/<name> data/racelines/<line>.csv
```

**Why it matters.** The stage exists to catch an undrivable line *before* driving
it. As written it reports how much clearance the recon lap had, which is already
known and cannot fail the way a generated line can.

**Fix.** Pass the generated line as well:
`... data/maps/<track> data/racelines/<track>_safe.csv`

---

## 8. Unpinned scipy pulls numpy 2.x into the submission image

**Where:** [`submission/Dockerfile:11`](../submission/Dockerfile#L11)

**Problem.**

```dockerfile
# Runtime deps our stack needs that the base image lacks. The base image is
# Python 3.10 / ROS 2 Humble.
RUN pip3 install --no-cache-dir scipy pyyaml     # <-- no version pins
```

The devkit base image ships **numpy 1.22.2** (apt, matching ROS 2 Humble, which
is built against numpy 1.x). Installing scipy today resolves to scipy 1.15.3 and
**upgrades numpy to 2.2.6** — a major version boundary.

**Observed 2026-08-25.** Reproduced in the devkit container:

```
$ pip3 list | grep -iE 'numpy|scipy'
numpy      2.2.6      # was 1.22.2 before installing scipy
scipy      1.15.3
```

`rclpy`, `numpy` and `scipy` still import together afterwards and no failure was
seen, so this is a risk rather than a confirmed break.

**Why it matters.** The submission is an image built once and uploaded; what
lands in it depends on what PyPI resolves to on build day. A dependency silently
crossing a major version on a stack not built for it is not something to discover
after the deadline.

**Fix.**
```dockerfile
RUN pip3 install --no-cache-dir "scipy<1.16" "numpy<2" pyyaml
```
and build the image once as a cold-start test before submitting.

---

## 9. `smoke_test.sh` cannot detect what it claims to test

**Where:** [`scripts/smoke_test.sh:7`](../scripts/smoke_test.sh#L7)

**Problem.**

```bash
./scripts/run_sim.sh headless          # <-- LiDAR is frozen in headless
./scripts/run_devkit.sh headless
echo "Waiting 15 s for sim + bridge to connect..."
sleep 15
...
echo "--- LiDAR rate (expect ~40 Hz) ---"    # <-- real rate is ~10 Hz
```

Two documented facts contradict this:

- Headless (`-batchmode -nographics`) makes the sim publish a **stale scan
  forever** — the LiDAR is GPU-raytraced and needs real rendering. This is
  gotcha #1 in `CLAUDE.md`.
- The real rate is **~10 Hz**, not 40. Measured 8.2–9.5 Hz on 2026-08-25.

**Why it matters.** The end-to-end sanity check reports a healthy-looking rate
from a frozen sensor. That is worse than no test, because it grants confidence.

**Fix.** Run the sim windowed, expect ~10 Hz, and assert that consecutive scans
actually *differ* rather than only that messages arrive.

---

## 10. Retracted docstrings still state the retracted claim

**Where:** [`src/pit_crew/vehicle.py:67-75`](../src/pit_crew/vehicle.py#L67),
[`src/pit_crew/pure_pursuit.py:5`](../src/pit_crew/pure_pursuit.py#L5)

**Problem.** `SpeedEstimator` and `ThrottleController` are adjacent in the same
file and say opposite things. First, `SpeedEstimator`:

```python
    """Vehicle speed from wheel encoder angles.

    `position` is the cumulative wheel angle in radians, so speed is simply
    wheel_radius * d(position)/dt. Verified against IPS ground truth on two
    different tracks: ... the scale factor comes out at 0.993 and 0.999.
```

Then, immediately below it:

```python
class ThrottleController:
    """Open-loop throttle from a target speed.

    Open loop on purpose: there is no trustworthy speed feedback on this
    platform (see SpeedEstimator), and closing the loop on the encoder signal
    made the car surge back and forth. `ff_gain` is a placeholder until the
    throttle->steady-state-speed curve is measured; ...
    """
```

Both claims are false. The encoders were verified; `ff_gain` defaults to the
**measured** 0.040. `pure_pursuit.py` repeats the same retracted claim in its
module docstring.

**Why it matters.** `STATUS.md` finding #2 is that this exact wrong belief cost
significant time. Leaving it in the source is how it gets re-learned.

**Fix.** Rewrite both docstrings to match what is now known.

---

## Retracted — claims that did not survive checking

Recorded so nobody re-raises them.

| Claim | Why it was wrong |
|---|---|
| "Dockerfile is missing Pillow" | Pillow 12.3.0 ships in the devkit base image. The Dockerfile installs exactly the two packages that are absent. |
| "LiDAR runs at 21.6 Hz, not ~10 Hz" | Measurement artifact — two simultaneous sim connections were open, publishing every message twice. Clean single connection measures 8.2–9.5 Hz. |
| "Clearance is overstated because unknown cells count as free" | 91% of unknown cells are the single exterior region beyond the walls; interior unknowns are 2,413 blobs of **median size 1 cell**. The apparent ICRA drop (0.564 → 0.202 m) was speckle. `footprint_analysis.py` is correct as written. |
