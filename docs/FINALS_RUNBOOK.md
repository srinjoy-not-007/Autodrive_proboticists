# Finals runbook — the day the finals image drops

**Rewritten September 12, 2026.** Ordered, copy-pasteable procedure for the
finals track. The compete image is released **September 16**; the submission is
due **September 19, AoE**. Every command below already exists in this repo — no
invented flags. Measured numbers live in `docs/STATUS.md`; this file gives the
sequence and the gates.

Facts about how the run is scored (verified from the official repo, recorded in
`docs/STATUS.md`): organisers run **Ultra graphics, GUI, windowed on a real
display**, start the team image with `xhost local:root` then
`docker run ... --entrypoint /bin/bash -it` and run `/home/autodrive_devkit.sh`
inside it, click **Connection** in the simulator by hand to start the race,
drive **12 laps** (warm-up, 10 scored, cool-down), sum the **simulator's own lap
timer**, add **+10 s per collision**, and **more than 10 collisions is a DQ**.

## The three things that decide this weekend

1. **The sensor period, not the lap time.** The whole stack sees the world at
   whatever period the bridge delivers, and `mpc3` is rate-sensitive. Measured
   on this host: **Xvfb 0.088-0.106 s** (a per-start lottery; Ultra on Xvfb
   0.102-0.105) and **a real display at Ultra 0.058 s** — graphics quality does
   not move the display number, the GPU/display does. The organisers measured
   ~0.097 s on an RTX 4090, **and we do not know what their box will actually
   deliver.** Four real-display runs of earlier shipping configs failed
   identically (10-11 collisions, always the same hairpin entry). As of Sept 12
   the stack is validated clean at **both** ends of the band — 0.058 s on the
   display and 0.09-0.10 s on Xvfb — **only with `barrier_margin 0.34`**
   (Xvfb regression on all four tracks running at the time of writing). Every
   run is evidence only together with its `dt_p50` (step 5A), and a run at one
   period says nothing about the other.
2. **The line, on a circuit tighter than Porto.** The drill's absolute kappa
   gate (`MESH_RACE_KAPPA_MAX` 1.0) rejects every width-using candidate on a
   track whose own centerline is tighter than 1.0 1/m — both 2025 circuits —
   and silently ships the centerline. The relative gate (`--kappa-gate safe`)
   plus the curvature-rate and half-width flags is what produces a fast line
   there, and the fast line is worth 0.7-1.2 s/lap. It also spends clearance,
   which is what `barrier_margin 0.34` needs. Step 3a has the rule.
3. **Assets that never made it into the image.** Three files must describe one
   circuit and all three must be `COPY`ed. `tests/test_submission_assets.py`
   checks this on the host, in under a second, from the entrypoint's own shell.

Everything that ships is in `submission/autodrive_devkit.sh` as a default; a few
validated-but-unshipped options are listed in step 5D and are turned on for one
run with `PIT_CREW_CONTROLLER_ARGS` / `PIT_CREW_LOCALIZATION_ARGS`, which are
appended **after** the script's own arrays and therefore win (ROS takes the last
`-p name:=value`; `tests/test_entrypoint_args.py` asserts that for both nodes).

Total wall clock for a clean pass: **~5-7 h**, dominated by the 100-lap Xvfb
validation and the display runs. Budget one full day.

---

## THE SEPTEMBER 16-19 CHECKLIST

Work top to bottom. Each line is either a command or a gate; do not pass a gate
on a run whose `dt_p50` you have not read.

### What is already in the image (change nothing to get these)

| Where | Parameters | Why |
|---|---|---|
| `CTRL_ARGS`, mpc3 branch | `map_prefix`, `rate_adaptive:=true` | Scales `apply_ahead_s` with the measured period. 100 Xvfb laps clean at 0.091-0.098 s; 7 collisions/32 → 2 at a 0.090 s instance. |
| `CTRL_ARGS`, mpc3 branch | `w_dcmd:=50.0 cmd_tau:=0.10` | Damps the solver's steering limit cycle. Display 0.058 s: flips 5.5 → 2.4/s, command step p90 0.14 → 0.041 rad. Costs ~0.9 s of Xvfb race time and ~0.5 s/lap on a tight circuit — robustness at an unknown period outranks both. |
| `LOC_ARGS` | `max_step_mps:=9.0 publish_on_reloc:=false` | The motion model's 0.6 m/update was a 5.77 m/s cap: SRL 2025 ICRA went 4 laps / 14 collisions (a DQ) with cross-track reading 0.03 m. With the fix, 32 laps / 0 collisions / 148.8 s; Porto and ICRA-2026 unchanged. |
| Everywhere | `OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=1` | mpc3's numpy otherwise runs at 1030 % CPU and starves the simulator, which changes the period. Do not remove. |

**Expected to ship, decided on Sep 16-17:** `barrier_margin:=0.34` (step 5D).
It is the only configuration that has ever produced a clean real-display run
(32 laps, 0 collisions, OFFICIAL **69.44 s** at 0.059 s). It is prepared as a
commented line in `CTRL_ARGS` and must be enabled deliberately, after its Xvfb
four-track regression is read.

### Sep 16 — track release (~1.5 h)

```bash
cd /home/ankit/code/data/simracing
git status && git checkout -b finals        # never on the promoted commit
./scripts/check_pipeline.sh                 # required for any tools/ change
uptime                                      # quiet host; no other container
TAG=<announced tag>                         # pattern: 2026-<venue>-compete
docker pull autodriveecosystem/autodrive_roboracer_sim:$TAG
docker pull autodriveecosystem/autodrive_roboracer_api:$TAG
.venv/bin/python tools/map_from_mesh.py --tag $TAG --list
./scripts/finals_drill_mesh.sh finals $TAG "<track name>"
```

**That is the whole line-generation step — one command, no flags to decide.**
The tight-circuit set that used to be a paragraph here is the drill's default
now (`--dkappa-max 1.5 --dkappa-refine 3 --kappa-bound 1.0 --kappa-gate safe`
plus `--min-halfwidth-ladder 0.53,0.50,0.45`), and stage 5/5 runs
`tools/pick_config.py`, which measures the chosen line and prints the
`PIT_CREW_CONTROLLER_ARGS` to race with. `LINE_ARGS=` (explicitly empty)
restores the old flagless behaviour if the new one produces something wrong.

Two things the drill does NOT do, on purpose: it never passes `--out-name` (the
shipping line must be `finals_mesh_race.csv`, the name the entrypoint derives
from the map slug), and it never decides to RACE the fast line. That decision
is Gate C and Gate D, not the ideal lap time.

**Gate A — the drill's own output** (step 3a explains each): free fraction
15-70 %, `source: mesh`, `lidar_offset 0.1933`, and a `race` and a `safe` line
both reported by `footprint_analysis.py`.

**Gate A2 — the drill's pick (stage 5/5).** It writes
`data/racelines/finals_mesh_pick.json`; read these five numbers out of it
before anything is driven.

| Number in the sidecar | What it is | Pass |
|---|---|---|
| `min_halfwidth` | closest the LINE comes to a wall, marched along the path normal — the way the barrier measures it | >= 0.45 m; below that no barrier margin worth having is feasible |
| `feasible_margin` | `min_halfwidth - 0.135` (car half-width). Above it the barrier pushes the car ACROSS its own line | >= 0.30 m |
| `barrier_margin` | 0.40 where `feasible_margin` clears 0.40 + 0.02 slack, else 0.34 | 0.34 or 0.40, never anything else |
| `thr_brake_lead_s` | 0.05 when the line's tightest kappa > 1.2 **or** the corridor min width < 1.3 m, else 0 | never 0.08 — bit-identical to 0.05 at 0.097 s |
| `clearance` / `ideal_lap` | worst body-corner clearance and the profile's own lap | clearance >= 0.28 m (the mesh floor); ideal lap is context, not a gate |

Measured on the circuits we have, for calibration:

| Circuit / line | min half-width | feasible | barrier | lead | clearance | ideal lap |
|---|---|---|---|---|---|---|
| Porto `cr_dk0.8` (qualifier) | 0.420 | 0.285 | 0.34 | 0.05 | 0.309 | 7.01 s |
| CDC-TF 2025, drill pick (= `fast3_m53`) | 0.520 | 0.385 | 0.34 | 0.05 | 0.395 | 9.96 s |
| SRL 2025 ICRA, drill pick | 0.500 | 0.365 | 0.34 | 0.05 | 0.389 | 14.24 s |
| ICRA 2026 (`safe`) | 0.720 | 0.585 | 0.40 | 0.05 | 0.608 | 14.40 s |

Porto is the **qualifier exception**: its shipped config runs `barrier_margin`
0.40 on `cr_dk0.8` (display 32/0 70.13 s, Porto 100/0 Xvfb) even though the
line's feasible cap is 0.285. Nothing else gets that.

Two known consequences of the rule, so they are not surprises on the day:

* On the two 2025 **safe** lines the measured cap is 0.385, so the rule lands
  on 0.34 where the entrypoint's `case "$LINE" in *_safe.csv|*cr_dk0.8.csv`
  line currently gives them 0.40. If the pick is enabled, it wins (last `-p`).
* On SRL 2025 ICRA the ladder stops at the **0.53** rung, which is a roomier
  and 0.40 s/lap slower line than the hand-picked `fast2_m50` (0.480 / 13.84 s)
  that was driven 32/0 there. Rung 0.53 buys 0.041 m of clearance and 0.020 m
  of feasible margin for that time. If Gate C/D are clean on `fast2_m50`,
  `LINE_ARGS='--dkappa-max 1.5 --dkappa-refine 3 --kappa-bound 1.0 --kappa-gate
  safe --min-halfwidth-ladder 0.50'` reproduces it exactly.

**Carrying the pick into the image.** Either paste the printed
`PIT_CREW_CONTROLLER_ARGS` into the `docker run` on the day, or add
`data/racelines/finals_mesh_pick.json` to the Dockerfile COPY list and
uncomment the `PICK_BLOCK` in `submission/autodrive_devkit.sh` (one block, at
the end of the mpc3 `CTRL_ARGS`). `tests/test_pick_config.py` runs that exact
block against a fixture sidecar, so enabling it is an uncomment, not a rewrite.

**Gate B — does the finals track outrun the localization speed cap?** The
shipped cap is `max_step_mps` 9.0 (it was 5.77, and that was a DQ). Check the
profile the drill just wrote:

```bash
.venv/bin/python - <<'PY'
import numpy as np
a = np.loadtxt('data/racelines/finals_mesh_safe.csv', delimiter=',')
s, v = a[:, 0], a[:, 5]
print('profile v_max %.2f m/s' % v.max())
for cap in (5.8, 8.5):
    fast, runs, n, i = v > cap, [], len(s), 0
    while i < n:
        if fast[i]:
            j = i
            while j < n and fast[j]:
                j += 1
            runs.append(s[min(j, n - 1)] - s[i]); i = j
        else:
            i += 1
    print('longest stretch above %.1f m/s: %.1f m' % (cap, max(runs) if runs else 0.0))
print('(above 5.8 m/s, the OLD cap, on the shipped safe lines: Porto 4.7,')
print(' ICRA-2026 10.0, CDC-TF 11.6 -- clean; SRL 2025 ICRA 13.8 -- 14')
print(' collisions in 4 laps, a DQ, before max_step_mps 9.0 shipped.)')
PY
```

Anything above ~12 m over **8.5** m/s means raise the cap for this track:
`-e PIT_CREW_LOCALIZATION_ARGS="-p max_step_mps:=12.0"`, and validate with it
on. Under that, the shipped default covers it and there is nothing to do — and
in practice it will, because `optimize_raceline.py`'s `--vmax` is 8.0 m/s and
the drill never overrides it, so no generated profile asks for more than 8. The
check is here for the case where someone raises `--vmax`, or where a very fast
circuit makes the planner sit at the ceiling for a long stretch.

### Sep 17 — validation runs (~4-5 h, quiet host, nothing else running)

Probes first, on Xvfb, to find out which line is even a candidate:

```bash
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_race_probe race
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_safe_probe safe
```

Read every probe with its period **before** reading its collisions:

```bash
.venv/bin/python tools/lap_report.py data/logs/race_<ts>.csv \
    data/racelines/finals_mesh_<line>.csv data/maps/finals_mesh
.venv/bin/python tools/score_race.py data/logs/race_<ts>.csv
```

Then the runs that decide it. **Both periods, in this order** — a config is not
shipped until it is clean at both:

```bash
# --- Gate C: 100 laps on Xvfb at the ORGANISERS' period, not at whatever the
#     per-start lottery gave you.
./scripts/sim_rate_calibrate.sh 0.097
SIM_CPUSET=<chosen> RATE_MIN_DT=0.094 RATE_MAX_DT=0.100 SIM_QUALITY=ultra \
  CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=100 \
  ./scripts/run_exp.sh finals_100 <race|safe> -p barrier_margin:=0.34

# --- Gate D: the real display, self-clicked, ~0.058 s. VISIBLE=1 puts the
#     simulator on this host's own display and steals the pointer; the click
#     geometry is located from a screenshot (tools/find_sim_ui.py), so no human
#     is needed. Do not touch the machine while it runs.
VISIBLE=1 SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh \
  LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_display <race|safe> -p barrier_margin:=0.34

# --- the rate-proven fallback, run it too
SIM_QUALITY=ultra CONTROLLER=mpc MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_legacy_probe safe
```

**Gate C — pass = 0 collisions over 100 laps with `dt_p50` in 0.094-0.100 s.**
32 laps is a probe, not a pass: it cannot resolve a 1-3 %/lap collision rate.

**Gate D — pass = 0 collisions over 32 laps with `dt_p50` ~0.058 s.** This is
the organisers' own setup and it is the gate that four earlier configurations
failed. If Gate D fails and Gate C passes, the problem is the corner the car
drifts inside of: read `tools/crosstrack.py` and `tools/signed_offset.py` on the
worst corner (step 4) and try `barrier_margin` before anything else — it is the
one lever that moved that offset outward at both rates. Do not spend Sep 17
inventing a new one.

**Gate E (optional, ~30 min) — the stall test.** The organisers record the
window with OBS on the same box. A three-frame telemetry gap puts the car on the
coast watchdog, and in the harness a repeated 1.0 s gap is 35-40 collisions in
36 laps. Inject the real thing into a live run from the host — nothing is killed
or restarted, and the resume is in an EXIT trap:

```bash
# in a second terminal, while a Gate C style run is going:
./scripts/stall_inject.sh --mode sim --dur 0.6 --every 15 --count 4 \
    --log data/logs/stall_$(date +%H%M%S).csv
```

Run it once with the shipped defaults. If the car collides in the stall windows,
the opt-in ride-out exists (step 5D) and can be turned on for one A/B without
rebuilding anything. This gate is optional because nothing has driven it live
yet; skip it rather than let it eat Gate C/D time.

### Sep 18 — bake, build, race the image (~2 h)

```bash
# 1. edit the two places the drill printed: the Dockerfile COPY list (the
#    centerline AND the one line you validated) and the entrypoint MAP default.
#    If the validated line is `safe`, either copy it to the derived name or set
#    PIT_CREW_LINE -- see step 6.
# 2. enable barrier_margin (and anything else Gate C/D needed) by uncommenting
#    the ONE prepared line in CTRL_ARGS -- see step 5D.
# 3. prove it on the host, in one second, BEFORE a 5-minute build:
.venv/bin/python -m pytest -q tests
# 4. build FROM the finals api image
docker build --platform linux/amd64 --build-arg TAG=$TAG \
  -f submission/Dockerfile -t YOUR_DOCKERHUB_USER/roboracer:iros2026-finals .
# 5. race the image as shipped, both documented run paths
./scripts/test_submission_image.sh YOUR_DOCKERHUB_USER/roboracer:iros2026-finals 12 $TAG
ENTRY_MODE=cmd ./scripts/test_submission_image.sh YOUR_DOCKERHUB_USER/roboracer:iros2026-finals 12 $TAG
.venv/bin/python tools/lap_report.py data/logs/submission_<ts>.csv \
    data/racelines/finals_mesh_<line>.csv data/maps/finals_mesh
```

**Gate F — 12 laps, 0 collisions, OFFICIAL race time at or below Gate C's, and
`dt_p50` in band, on both entry modes.** (`test_submission_image.sh` races on
Xvfb, so its band is Xvfb's.) The one image regression we have ever seen showed
up as `dt_p50` 0.088 s against 0.104 s and nothing else. A 14-lap sample
scatters ~1.5 s, so a small time difference is not a regression; a rate
difference is.

### Sep 19 — publish and submit (deadline AoE)

```bash
docker push YOUR_DOCKERHUB_USER/roboracer:iros2026-finals
docker image inspect YOUR_DOCKERHUB_USER/roboracer:iros2026-finals \
  --format '{{json .RepoDigests}}'
```

- Record the digest in `docs/STATUS.md` and the submission notes.
- If the DockerHub repository is private, add DockerHub user
  `autodriveecosystem` as a collaborator. Verify by pulling the exact
  `repository:tag` from a clean `docker logout` shell if you can.
- Paste the overview text (step 10) into the repository description, with the
  variable defaults updated to the finals values.
- **Submit the exact `repository:tag` on the form linked from the event page.**
  A GitHub push is not a submission; publication *and* the form must both be
  complete before **September 19, AoE**.
- Do not touch the branch again after the digest is recorded. Any further
  experiment goes on a separate branch.

**Slipping the schedule:** if Sep 17 is not clean, spend Sep 18 on the decision
table (5C) rather than on new tuning, and keep Sep 19 for publication only.
Something imperfect that finishes 12 laps beats a DQ and beats no submission.
---

## Step 0 — Preconditions (5 min)

```bash
cd /home/ankit/code/data/simracing
git status                 # clean tree; work on a branch, never on the
git checkout -b finals     # qualification/promoted commit
uptime                     # host must be quiet: run_exp.sh refuses load > 12
docker ps                  # no leftover autodrive_* containers
```

**Nothing else may run on this host while a scored run is in progress** — no
offline line search, no second container, no build. This is not politeness: an
offline search measurably drops the simulator's frame rate, which changes the
sensor period, which changes what mpc3 does. `run_exp.sh` refuses above load 12,
but load 12 is a backstop, not the gate; the real gate is `dt_p50` in step 5.
The BLAS thread pin that keeps mpc3's own numpy from eating the host
(`OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=1`, measured 1030 % CPU
without it) is already in `submission/autodrive_devkit.sh` and in
`scripts/run_stanley.sh` — nothing to set by hand, but do not remove it.

Preserve the currently shipping image digest before touching anything. A
validated image you can still push is the floor under the whole weekend.

---

## Step 1 — Pull the finals images (~10-20 min, network bound)

Replace `<TAG>` with whatever the organisers announce (the pattern of the
existing tags is `2026-<event>-compete`, e.g. `2026-icra-compete`).

```bash
TAG=2026-iros-finals        # <-- the announced tag
docker pull autodriveecosystem/autodrive_roboracer_sim:$TAG
docker pull autodriveecosystem/autodrive_roboracer_api:$TAG
```

Both are needed: the `sim` image for driving and for the mesh, the `api` image
as the `FROM` base of the submission image.

---

## Step 2 — List the tracks and confirm a mesh exists (2 min)

```bash
.venv/bin/python tools/map_from_mesh.py --tag $TAG --list
```

This does `docker create` + `docker cp` against an image that is never started,
so it cannot disturb anything running. Note the **exact track name** — you pass
a substring of it to the drill. Sanity-check the geometry before trusting it:

```bash
.venv/bin/python tools/map_from_mesh.py --tag $TAG "<track name>" /tmp/probe --report
```

`--report` prints the mesh's height extent and near-horizontal face fraction.
Our tracks are two closed tubes with **no floor faces**; if a future track does
carry a floor, slice a height band with `--band LO HI`.

**No usable mesh** means: `--list` shows nothing matching, or there is no
`"<stem> Checkpoints"` / `"<stem> Spawn Points"` root, or the tool exits before
writing a `.pgm`. Go to the fallback in step 3b.

---

## Step 3a — Build the map and lines with the mesh drill (~45 s)

```bash
./scripts/finals_drill_mesh.sh finals $TAG "<track name>"
```

Writes `data/maps/finals_mesh.{pgm,yaml}`, `data/maps/finals_mesh_checkpoints.csv`,
and `data/racelines/finals_mesh_{centerline,safe,raceline,race}.csv`. Only the
map pair, `_centerline.csv` and the one line you validate are needed at runtime.
The checkpoints CSV is a host-side artefact and is **not** in the image (the map
glob takes `.pgm` and `.yaml` only), which is why the prepared teleport line in
5D sets `teleport_reseed:=false` — reseeding is the one branch that would need
it, and it measured no better than holding.

It then prints a footprint report for **both** `race` and `safe`, and the exact
two edits (Dockerfile `COPY` list, entrypoint `MAP` default) for this slug.

Stage 5/5 then runs `tools/pick_config.py <slug>_mesh`, which measures the
chosen line's min half-width and tightest curvature and prints the
`PIT_CREW_CONTROLLER_ARGS` for it (Gate A2 in the Sep 16 checklist).

The drill passes the tight-circuit set to `make_track_lines.py` **by default**:

```
--dkappa-max 1.5 --dkappa-refine 3 --kappa-bound 1.0 --kappa-gate safe
--min-halfwidth-ladder 0.53,0.50,0.45
```

`LINE_ARGS=<anything>` replaces that set wholesale, and `LINE_ARGS=`
(explicitly empty) restores the old flagless behaviour, whose output is
byte-identical to what the drill produced before. All of these flags are still
off by default inside `make_track_lines.py` itself, so `check_pipeline.sh` and
the recon tracks are untouched. Three things that matter more than they look:

* **`--kappa-gate safe` is the relative gate**: max(steering lock 1.78 1/m, the
  safe line's own curvature peak). The absolute 1.0 bar rejects width-using
  candidates on any circuit whose centerline is itself tighter — cdctf2025
  peaks at 2.10, icra2025 at 1.63 — i.e. it rejects the *easier* line while
  shipping the harder one.
* **`--min-halfwidth-ladder 0.53,0.50,0.45`** floors the distance from the line
  to either wall at every stage of the QP — not just the QP, which is where
  the fast lines' single pinch came from (0.427 m and 0.432 m measured). The
  ladder takes the FIRST rung that still yields an accepted candidate and only
  descends when the corridor is exhausted; if no rung solves, `make_track_lines`
  ships `safe` on its own. 0.53 is what absorbs the tracker's hairpin cut on
  CDC-TF (hairpin clearance p05 0.30 vs 0.25 at 0.50), and the drill's 0.53
  output there is byte-identical to the hand-built `fast3_m53`. The printed
  `race line:` line names the rung that won.
* **Never pass `--out-name` for the shipping line.** The entrypoint derives
  `<slug>_race.csv` from `PIT_CREW_MAP`; a line generated as
  `finals_fast_race.csv` is a line the image cannot find by default.

### What each printed gate means

| Stage | Gate | Meaning if it fails |
|---|---|---|
| 2/5 free fraction | must be **15-70 %** (every known circuit gives 30-55 %) | The mesh or its parent transform is wrong — e.g. the `Infrastructure` root at Unity (-10, 0, -14) was missed, so the spawn lands outside the corridor and the flood fill leaves everything unknown; or the track mesh carries a floor and rasterised solid. Re-run `map_from_mesh.py` with `--report` and `--list`. |
| 2/5 `source: mesh` | asserted | The YAML was not stamped by `map_from_mesh.py`. Nothing downstream will take the mesh code path. |
| 2/5 `lidar_offset` | must print **0.1933** | This is what the entrypoint and the preflight will be checked against. |
| 3/5 `clearance X.XXX m ideal lap Y.YY s` per candidate | per-candidate report | Informational unless followed by `REJECTED:`. |
| 3/5 `REJECTED: clearance ... < 0.28 m budget` | `MESH_MIN_CLEARANCE` = 0.28 (0.230 collided on ICRA, 0.292 clean on Porto) | That candidate is too close to a wall for our lateral error. Normal — the sweep rejects several. |
| 3/5 `REJECTED: kappa max K above the 1.0` | `MESH_RACE_KAPPA_MAX` = 1.0 | The width-using race line is tighter than anything mpc3 has held on Porto (0.83 clean over 100 laps; ICRA 1.36 → 9 collisions in 30 laps). If this fires on **every** candidate, the circuit is tighter than Porto and the absolute bar is wrong for it: re-run with `--kappa-gate safe` (above). If it still fires, the centerline ships — that is an outcome, not a failure. |
| 3/5 `no solution (corridor exhausted)` | the QP ran out of corridor at that `w_veh` | A fact about the track, not an error. |
| 3/5 `safe line is not drivable; shipping the opened-corner line instead` | graceful degrade | Accept it, but drive it before trusting it. |
| 4/5 `footprint_analysis.py` on `finals_mesh_race.csv` | worst-corner body clearance | This is the number that decides step 5's first run. |
| 5/5 `pick_config.py` | `min_halfwidth` >= 0.45 m, `feasible_margin` >= 0.30 m, `barrier_margin` in {0.34, 0.40}, `thr_brake_lead_s` in {0, 0.05} | Gate A2. A line whose feasible margin is under 0.30 m cannot carry a barrier worth having — ship `safe` and re-read its sidecar. |

If every raceline candidate is rejected but `safe` survives, `race.csv` will be
the rejected best; **race `safe`, not `race`** — see step 5.

Useful reruns:

```bash
.venv/bin/python tools/make_track_lines.py finals_mesh --min-clearance 0.30
.venv/bin/python tools/make_track_lines.py finals_mesh --out-name try1   # no overwrite
```

### Step 3b — Fallback: no mesh, or the line pipeline rejects everything (~271 s + a live sim)

Recon drill (drives the unknown track blind with Follow-the-Gap, then builds a
map from the recording):

```bash
FTG_ARGS='-p bubble_radius:=0.22 -p extend_margin:=0.18' \
  ./scripts/new_track_drill.sh finals $TAG 150
```

**A recon map is built at LiDAR offset 0 and must be raced at 0** —
`PIT_CREW_LIDAR_OFFSET=0`, `LIDAR_OFFSET=0` in `run_exp.sh`. Mixing them costs a
systematic 0.19 m bias against ~0.29 m of clearance; the preflight now refuses
the mismatch outright.

If the recon drill also produces nothing raceable, ship the **safe line**
(`finals_*_safe.csv`; `_centerline.csv` is not a raceline — see above) — it is
the corridor middle, has
0.5-0.6 m of room on every circuit we have measured, and is what ICRA-2026
ships.

---

## Step 4 — Localization health from the log (5 min, after the first run)

`race_logger.py` writes both the simulator IPS truth (`x`,`y`) and the particle
filter's estimate (`px`,`py`) into every `data/logs/race_*.csv`. Compare them:

```bash
.venv/bin/python tools/crosstrack.py data/logs/race_<ts>.csv \
    data/racelines/finals_mesh_<line>.csv
.venv/bin/python tools/signed_offset.py data/racelines/finals_mesh_<line>.csv \
    data/logs/race_<ts>.csv <s_lo> <s_hi>
```

`crosstrack.py` reports lap-wide `|e_lat|` p50/p95/max plus the same inside an
`s` window (default 14.0-17.5, Porto's problem corner — pass the finals track's
own worst corner from the drill output). `signed_offset.py` bins signed offset
(+ = left of travel) by `s` with the local curvature, which is how you tell
"cutting the apex" from "running wide".

Healthy on a mesh map: `e_lat` p50 ~0.07 m, p95 ~0.18 m or better. A p95 near
0.3 m, or a systematic one-sided bias in `signed_offset.py`, means the map and
the filter disagree — check `lidar_offset` first.

---

## Step 5 — Validation runs with `run_exp.sh`

`run_exp.sh` refuses to start when 1-min load average exceeds 12 (`LOAD_OK=1`
overrides — **do not** for a scored run). **A scored run needs a quiet host.**
A 150-lap re-run of a 0/100 config while three build agents computed took 9
collisions. Any run taken under load is contaminated evidence; re-run it.

**Every command in this step carries `SIM_QUALITY=ultra`.** It is a hook in
`run_stanley.sh`: after the sim is connected it calls
`scripts/sim_set_quality.sh ultra`, which clicks the simulator's quality button
(the cycle Low -> High -> Ultra is verified; on a real-display run `VISIBLE=1`
points it at `$DISPLAY` instead of `:99`). The organisers run Ultra, and quality
changes the Xvfb period by ~4 %. Omitting it validates at the wrong setting.

Order of combos (the checklist's Gates C-F are these runs with their pass
conditions attached):

**5.1 — mpc3 on the race line, only if step 3a promoted one** (~15 min for 32
laps):

```bash
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_race_probe race
```

**5.2 — otherwise (or if 5.1 collides), mpc3 on the centerline/safe line:**

```bash
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_safe_probe safe
```

**5.3 — 100-lap validation of whichever of 5.1/5.2 was clean** (~1 h):

```bash
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=100 \
  ./scripts/run_exp.sh finals_100 <race|safe>
```

**5.4 — the real display** (~15 min for 32 laps), which is where every
configuration before Sept 12 failed. `VISIBLE=1` puts the simulator on this
host's own display; the Connection click is located from a screenshot
(`tools/find_sim_ui.py`) so no human is needed, but it steals the pointer —
leave the machine alone. `MANUAL_CONNECT=1` is the fallback if the self-click
ever misses (it waits `MANUAL_WAIT_S`, default 300 s, for a human).

```bash
VISIBLE=1 SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh \
  LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_display <race|safe> -p barrier_margin:=0.34
```

**5.5 — the rate-proven fallback, run it too** (~15 min). The legacy tracker on
the safe line is slower but has been clean at both Xvfb rates, so it is what you
ship if mpc3 cannot be made clean:

```bash
SIM_QUALITY=ultra CONTROLLER=mpc MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_legacy_probe safe
```

**Pass = 0 collisions over 100 Xvfb laps at ~0.097 s (Gate C) AND 0 collisions
over 32 real-display laps at ~0.058 s (Gate D), on a quiet host.** 32 laps is a
probe on Xvfb, not a pass: it cannot resolve a 1-3 %/lap collision rate
(measured — 0.04 `throttle_a_gain` was 0/32 and 8+/100). On the display, where
the failures ran at 30-100 % of laps, 32 laps is a real test.

### 5A — The logger-period gate (mandatory on every scored run)

Before a run counts as evidence, read the sensor period it was taken at:

```bash
.venv/bin/python tools/lap_report.py data/logs/race_<ts>.csv \
    data/racelines/finals_mesh_<line>.csv data/maps/finals_mesh
```

The last column of the per-lap table is **`dt_p50`** — the median interval
between distinct sensor poses in that lap (rows where the IPS pose did not
change are dropped, so it measures the bridge, not the CSV writer). Read it per
lap, not just the summary row: a run that starts at 0.104 and drifts to 0.090
is a run something else started competing for the GPU part-way through.

| `dt_p50` | Verdict |
|---|---|
| ~0.058 s | **The organisers' own setup** (simulator windowed on a REAL display at Ultra) and the hardest case. Four configurations failed here with 10-11 collisions; the only clean run is the shipping smoothing plus `barrier_margin 0.34` — 32 laps, 0 collisions, 69.44 s. This is Gate D and it is not optional. |
| 0.088-0.106 s | Xvfb on this host, a per-start lottery; Ultra on Xvfb sits at 0.102-0.105. Valid evidence, and the band every 100-lap century was measured in — but evidence about **this** end of the range only. |
| ~0.097 s | The organisers' measured rate on an RTX 4090. Aim Gate C here with `sim_rate_calibrate.sh` + `RATE_MIN_DT`/`RATE_MAX_DT`. |
| < 0.052 s or > 0.156 s | Outside `rate_lo`/`rate_hi`: the rate adaptation is clamped and the 0.104 s tune applies unscaled. The clamp is logged. Nothing is validated there. |
| Varies by more than ~0.005 s across the laps of one run | Something else ran on the host. Contaminated; re-run. |

If a run lands off Ultra's expected 0.102-0.105 s, the first suspects are
`SIM_QUALITY=ultra` not having been passed (or its click having missed — the
script prints a WARN and saves a label crop under `/tmp/sim_quality`) and
another process on the host.

### Reproducing the organisers' sensor period

The period is **not a property of the host** — it is a per-start lottery. One
CPU-saturated Unity render thread sets the telemetry rate (the GPU is idle and
there is no vsync on Xvfb), so which cores that thread lands on decides what the
whole stack sees: 0.088-0.091 s on some starts, 0.103-0.106 s on others, stable
within a run. That is the difference between the rate mpc3 collides at and the
rate every clean century was measured at, and the organisers' ~0.097 s sits
between them — so "run it again until it looks right" is not a method.

Pinning the sim container turns the lottery into a dial. `SIM_CPUSET` passes
`--cpuset-cpus` straight through to the sim container in
`scripts/sim_restart_connect.sh`. On this machine cpus **0-7,16-23** are CCD0
(fast) and **8-15,24-31** are CCD1 (~5 % slower); a **narrower** set slows the
render thread further, which is how a period is lengthened toward 0.097 s.

```bash
./scripts/sim_rate_calibrate.sh 0.097
```

It starts the shipping stack once per candidate cpuset
(`CONTROLLER=mpc3 LIDAR_OFFSET=0.1933`, `run_stanley.sh 1.0 porto_mesh
mc0.63_ay6.75`, Ultra), waits 30 s, measures the logger period exactly as
`run_exp.sh` does — the median of the last 150 row intervals of the newest
`data/logs/race_*.csv` — stops the run, and prints a `cpuset -> period` table
plus whichever set lands closest to the target. A cpuset whose stack fails to
start (a missed Connection click, a wedged sim) is reported as `skipped`, not
guessed at. Candidates default to
`0-7,16-23  8-15,24-31  0-3  8-11  0,16  8,24`; override with `SIM_CPUSETS`.
Budget ~8 min for the default six.

`SIM_GPU_LOCK=1` runs `sudo nvidia-smi -pm 1 -lgc <GPU_MHZ>` first (default
1800). It **needs sudo and persists after the script exits** — run
`sudo nvidia-smi -rgc` when you are done. It is opt-in because the render thread
is CPU-bound: locking clocks removes a source of drift, it does not move the
period much.

**Then use the chosen set on every validation run, with the period gate as the
check, not as the mechanism:**

```bash
SIM_CPUSET=<chosen> RATE_MIN_DT=0.094 RATE_MAX_DT=0.100 \
    SIM_QUALITY=ultra CONTROLLER=mpc3 LIDAR_OFFSET=0.1933 \
    ./scripts/run_exp.sh <name> mc0.63_ay6.75
```

`SIM_CPUSET` aims the start at the right period; `RATE_MIN_DT`/`RATE_MAX_DT`
restart the run if it still lands outside the band, and `dt_p50` in 5A confirms
after the fact what it actually ran at. All three, every scored run.

### 5B — Scoring: use the OFFICIAL numbers

Every run appends a row to `data/experiments.tsv` and prints
`laps= coll= median= best= mean=` plus the `score_race.py` line. Score by hand
with:

```bash
.venv/bin/python tools/score_race.py data/logs/race_<ts>.csv
```

It reports one warm-up plus ten scored laps, +10 s per scored collision, from
**three** clocks. Quote the first one; the others exist to cross-check it:

| Line | What it is | Use |
|---|---|---|
| `OFFICIAL (simulator last_lap_time) laps: ...` / `OFFICIAL race ... s + N s penalties` | The values the simulator itself latches at each line crossing — exactly what the organisers' Foxglove `race_time` / `adjusted_race_time` accumulate. **No sampling error at all.** | **This is the score.** Report it, record it in `docs/STATUS.md`, compare configs on it. |
| `Sim-timer estimate:` | The running `lap_time` read on the last sample *before* the transition — same clock, up to one sensor period early. | A sanity check on the official line, and the only sim-side number for a log from a logger too old to record `last_lap_time`. |
| `Sample-clock laps:` / `Sample-clock elapsed` | Differences of our own CSV timestamps; jitter on both ends. | Diagnostics only. Never quote it as a race time. |

`OFFICIAL race time unavailable` means the log predates the `last_lap_time`
column — re-run rather than scoring a finals decision off the sample clock.
`UNVERIFIED: ...` means the log cannot be certified at all (started after
warm-up, a counter reset mid-log, a missing lap transition): fix the run, do not
argue with the tool.

### 5C — Controller decision table for the finals

Take the **first** row whose condition holds, from the top:

| # | Ship | Condition to be allowed to ship it |
|---|---|---|
| 1 | **mpc3 on the race line** (`finals_mesh_race.csv`) | Step 3a produced a race line that passed its kappa gate — the absolute 1.0 one, or the relative `--kappa-gate safe` one on a tight circuit — **and** Gate C (100 Xvfb laps at ~0.097 s, 0 collisions) **and** Gate D (32 real-display laps at ~0.058 s, 0 collisions) both passed with the config you intend to ship. All three, not any of them. |
| 2 | **mpc3 on the safe line** (`finals_mesh_safe.csv`, never `_centerline.csv`) | The kappa gate rejected the race line, or the race line failed Gate C or D, and mpc3 on the safe line passes both gates. This is what ICRA-2026 ships (0/30 where the width-using line took 9 collisions in 30), and it is also the line `barrier_margin 0.34` is geometrically feasible on. |
| 3 | **Legacy `mpc` on the safe line** (`PIT_CREW_CONTROLLER=mpc`) | Neither mpc3 configuration is clean. Slower — 78.2-78.4 s on Porto against mpc3's ~70-73 s — but it is the **rate-proven** option: clean at 11 Hz (32 laps, 0 collisions) *and* at Ultra on Xvfb (32 laps, 0 collisions). **It has never been driven on the real display**, so row 3 is a fallback with a known lap time and an unknown Gate D. The entrypoint pairs `mpc` with the safe line automatically; **never pair `mpc` with the wide min-curvature line.** |

Decide on the official numbers at **both** periods, with the collision rate and
the +10 s penalty in front of you, and remember >10 collisions is a DQ: one
collision per ten laps is +10 s on a ~70 s race, which no line choice can pay
for. A configuration that is clean on Xvfb and collides on the display is not a
candidate — the organisers race on a display.

### 5D — What ships, what is opt-in, and what stays off

`submission/autodrive_devkit.sh` holds two arrays: `CTRL_ARGS` (the controller)
and `LOC_ARGS` (localization). Both are appended at **every** launch site —
initial and supervisor restart — and both come **before**
`PIT_CREW_CONTROLLER_ARGS` / `PIT_CREW_LOCALIZATION_ARGS`, so a finals-day
`-e PIT_CREW_CONTROLLER_ARGS="-p barrier_margin:=0.30"` overrides the built-in
value (ROS takes the last `-p name:=value` for a repeated name).
`tests/test_entrypoint_args.py` asserts both the ordering and the restart path.

**Shipping by default (mpc3):**

```
-p map_prefix:=$MAP -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10
```

* `rate_adaptive` scales `apply_ahead_s` — a constant 0.090 s, i.e. 0.87 of the
  0.104 s period mpc3 was tuned at — with the measured period. 100 Xvfb laps
  clean at 0.091-0.098 s; at a rate-selected 0.090 s instance it cut 7
  collisions in 32 laps to 2. Identity at 0.104 s. The clamp is
  [0.052, 0.156] s (`rate_lo` / `rate_hi`, both parameters, and the clamp is
  logged) — below 0.052 s it silently reverts to the 0.104 tune.
* `w_dcmd` + `cmd_tau` damp the solver's steering limit cycle: the first free
  plan step after the committed steps has no jerk cost, so a one-step flick at
  the rack's slew limit is the cheapest answer to any small error and it is
  never executed before the next solve flicks back. Display (0.058 s): feedback
  sign flips 5.5 → 2.4/s, command step p90 0.14 → 0.041 rad. Xvfb Porto 71.17 s
  against 70.31 s without — the price of robustness at an unknown period. The
  cost is larger on a tight circuit (CDC-TF 116.7 s vs 111.1 s, +0.5 s/lap) and
  it is entirely `cmd_tau`: `w_dcmd` alone is the fastest variant anywhere
  (Xvfb Porto 69.39 s) **and takes 10 collisions in 12 display laps**. The
  filter is what survives 0.058 s; do not drop it to buy the lap back.

**Shipping by default (localization):**

```
-p max_step_mps:=9.0 -p publish_on_reloc:=false
```

`max_step_m` 0.6 m per update was a 5.77 m/s cap on the motion model at
`rate_nominal` 0.104. On a circuit with a long fast stretch the estimate falls
metres behind and the car drives into a wall reading cross-track 0.03 m: SRL
2025 ICRA, 4 laps and 14 collisions, a DQ. With the fix: 32 laps, 0 collisions,
148.8 s, and Porto (70.31 s) and ICRA-2026 (148.9 s) unchanged.
`publish_on_reloc:=false` suppresses the one pose published after a global
reinit — the centroid of a track-wide particle cloud is not a pose, and it was
measured 13 m from the car twice.

**The decision of the weekend — `barrier_margin:=0.34` (prepared, commented).**

```
CTRL_ARGS+=(-p barrier_margin:=0.34)
```

Default is 0.24. This is the only configuration that has produced a clean real
display run: **32 laps, 0 collisions, OFFICIAL 69.44 s at 0.059 s**, the best
official race time measured. It works by moving the hairpin offset outward at
both rates (+0.056 m at 0.058 s) for about +0.07 s/lap. Margin is thin — worst
lap -0.313 m against a wall at ~-0.35 m — and it is **geometrically feasible
only where the line leaves 0.135 + 0.34 = 0.475 m of half-width**: measured
feasible on every `safe` line (worst available 0.520 m at the CDC-TF S-kink,
lap sum -0.02 s, 1 collision in 432 harness laps) and **infeasible on the fast
lines and on Porto's own line** (0.40-0.42 m), where it degenerates into a bias
toward the centre. See the `--min-halfwidth` rule in step 3a.
**Enable when:** its four-track Xvfb regression is clean and Gate D is clean
with it. Uncomment the one prepared line — nothing else in the script changes.

**Opt-in, committed, not shipped.** Each is one `-e PIT_CREW_*_ARGS` away; none
has a live run behind it, so each needs its own A/B before it goes in the image:

| Knob | Flags | What it is for | Evidence |
|---|---|---|---|
| Tight-track lap cost | `-p steer_tau:=0.04` | Models the rack's first-order lag in the rollout, buying back the tracking `cmd_tau` costs | CDC-TF 112.0 s with the smoothing against 116.7 s without it (4.7 of the 5.5 s back), 32 laps 0 collisions. **Alone** on the display it is no better than the smoothing alone (10 collisions / 30 laps); its display test *with* the barrier is the open question. Must stay at 0.04 — 0.06+ is a mismatch that collides on a pure-slew plant. |
| Telemetry gaps (controller) | `-p gap_predict_s:=0.6` | Keeps solving on the dead-reckoned pose with a decel ramp instead of dropping to the coast watchdog | Harness with a stall every 5 s, 72 laps per cell: 3-frame gaps 6 → 1 / 19 → 8 collisions; 5-frame 18 → 3 / 46 → 18. One dropped frame is fine either way; no-gap reports byte-identical. Not driven live — see Gate E. |
| Telemetry gaps (filter) | `-p gap_dt:=0.15 -p gap_noise_mult:=3 -p lost_patience_min:=3 -p scan_dt_max:=1.2 -p enc_gap_max:=1.5` | Two 0.5 s constants coincide by default, so above a 0.5 s gap the filter dead-reckons at the stale pre-gap speed: 0.9 m of phantom travel for a stalled car, 2.25 m error and 9 relocalizations for a 1 s gap | Replayed on the real collision bag, 25 injected stalls x 6 seeds: worst post-gap error 0.46 m. Widening the cloud works; capping the step throws away real motion. |
| Teleport detector | `-p teleport_scan_jump:=0.35 -p teleport_imu_g:=0 -p teleport_hold_n:=7 -p teleport_reseed:=false` | A collision reset moves the car ~0.88 m and the filter never notices; the scan branch does | On the real collision bag: **9/9 resets detected within one frame, 0 false positives** for any threshold 0.20-0.525 (clean max 0.194). The IMU branch is unusable (1/9, one frame early once) — keep `teleport_imu_g` 0. Reseed helped on 0/9 and costs 0.06 m; hold-only is better, and the image carries no `checkpoints_csv` anyway. Peak post-reset error 0.65 → 0.30-0.38 m. |
| Long straight beyond 9 m/s | `-p max_step_mps:=12.0` | Only if Gate B says the finals profile asks for more than the shipped cap | Zero lap cost at any cap ≥ 6.4 in the harness. Pair with the caveat that a higher cap also lets the cloud dead-reckon further after a reset (up to 0.89 m). |

**Rejected — do not re-test on the day.**

| Knob | Why not |
|---|---|
| `apply_floor_steps:=0.0` + `rate_terms:=...` (the old "fix 1") | It did exactly what the harness said at 0.058 s (command slew saturation 0.078 → 0.003) and changed **nothing** at the hairpin: 15 laps, 11 collisions. The slew was a symptom. With the shipping smoothing the staircase it targeted is gone (0.0 % over-slew commands) and the fix now *costs*. |
| `nd_mode:=period` | +0.5..+2.2 s/lap on the tight circuits and 61/432 harness collisions on ICRA 2025 with the identified actuator, at four different corners. Porto's +0.05 m does not generalise. |
| `lead_mode:=spatial` (spatial publish lead) | Display: 11 collisions in 10 laps. ForzaETH's L1 numbers are a pure-pursuit lookahead, not a plan knot — 198/210 collisions as a publish lead. |
| `w_dcmd` alone (no `cmd_tau`) | Fastest everywhere and worst clearance; 10 collisions in 12 display laps. |
| `rate_slew` | Added while auditing the period range; the data does not support widening the clamp. |
| `control_mode:=timer` | A fixed 20 Hz solve is 0.25 s/lap slower and collides at every period (4-6 in 33 laps vs 0). Per-pose solving is information-optimal on this bridge. |
| ILC (`ilc_enable` and the `ilc_*` family) | Converges within two laps and ran 32 laps clean at 0.092 s, but a table learned at one period goes stale when the period moves — 100 laps with the prior took 7 collisions, 5 of them in the 20 laps after the host drifted 0.092 → 0.108 s. The organisers' period is unknown and OBS may drift it. It removes only 30-37 % of the hairpin offset and the shipping config is clean without it. If ever re-tested, always start from a fresh copy of the pristine prior — the node overwrites the table each lap. |
| `grip_watchdog` | Costs more race time than it saves in every configuration, and at the shipping cap it fires on a clean run. |

**Testing any of them without editing anything.** `run_exp.sh` passes its
positional extras to the controller and `LOC_ARGS` to localization:

```bash
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_bm34_tau race -p barrier_margin:=0.34 -p steer_tau:=0.04
LOC_ARGS='-p gap_dt:=0.15 -p gap_noise_mult:=3 -p lost_patience_min:=3 \
          -p scan_dt_max:=1.2 -p enc_gap_max:=1.5' \
  SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_gaploc race -p gap_predict_s:=0.6
```

and in the built image with `-e PIT_CREW_CONTROLLER_ARGS="..."` /
`-e PIT_CREW_LOCALIZATION_ARGS="..."`, which is how you change the shipped
config on the organisers' box without a rebuild — if they will let you set an
environment variable at all. They run the script with no environment, so
**anything that must ship, ships as an uncommented line in the script.**

If the race line collides, rebuild at a floor with evidence behind it
(`--min-clearance 0.30`) and re-probe; if it still collides, race `safe`.

---

## Step 6 — Bake the assets into the image (10 min)

Edit `submission/Dockerfile`:

- **Maps need no edit.** The COPY line globs `data/maps/*_mesh.pgm`
  `data/maps/*_mesh.yaml`, deliberately, so a finals mesh map ships without a
  Dockerfile change. A **recon** map (step 3b) is not `*_mesh` and **must be
  added by hand**, both `.pgm` and `.yaml`.
- **Racelines are listed one by one, never globbed** — on purpose, so no
  undriven candidate is one `PIT_CREW_LINE` typo away from being raced. Add to
  the existing `COPY data/racelines/... /home/data/racelines/` list, at the
  marked `ADD THE FINALS LINES HERE` comment:
  - `data/racelines/finals_mesh_centerline.csv` (**required** — localization
    seeds its whole particle cloud at `centerline[0]` and resamples along it on
    every collision recovery; a missing or wrong-track centerline is a race
    with no laps in it), and
  - the one line you validated in step 5 (`finals_mesh_race.csv` or
    `finals_mesh_safe.csv`). Nothing undriven.

Then point the entrypoint at them. Two equivalent options:

**Option A (no code change, chosen at run time):**

```
PIT_CREW_MAP=/home/data/maps/finals_mesh
PIT_CREW_LINE=/home/data/racelines/finals_mesh_race.csv
PIT_CREW_CENTERLINE=/home/data/racelines/finals_mesh_centerline.csv
PIT_CREW_LIDAR_OFFSET=0.1933
PIT_CREW_CONTROLLER=mpc3
```

**Option B (edit the defaults in `submission/autodrive_devkit.sh`) — preferred**,
because the organisers run the script with no environment at all. **It is now
one line:**

```
MAP="${PIT_CREW_MAP:-/home/data/maps/finals_mesh}"
```

`SLUG` comes from `basename "$MAP"`, and `LINE` and `CENTER` both follow it:
`<slug>_race.csv` for `mpc3`, `<slug>_safe.csv` for the legacy `mpc`, and
`<slug>_centerline.csv` for localization. That is exactly why the drill's output
names matter and why the centerline must be copied under that name. (`porto_mesh`
keeps an explicit exception for its validated `cr_dk0.8` line; every other slug
is derived.)

**If the validated finals line is `safe` rather than `race`** — which is the
likely outcome on a tight circuit, and is what ICRA-2026 ships — do **not**
leave the `mpc3` default pointing at a `race.csv` nobody drove. Pick one:

```bash
# preferred: make the derived name BE the validated line, one file, one COPY
cp data/racelines/finals_mesh_safe.csv data/racelines/finals_mesh_race.csv
# or ship both files and pin it in the script (a prepared comment marks the spot):
#   LINE="/home/data/racelines/${SLUG}_safe.csv"
```

The copy is the safer of the two: `tests/test_submission_assets.py` then checks
the exact file the entrypoint will load, and there is no second line in the
image for a typo to reach.

The line default used to be two fixed Porto paths, so changing `MAP` alone left
the legacy branch on `track_v2_safe.csv` — Porto's line on the finals map.
Preflight caught it, but it caught it by refusing to start: exit 2, no race.

Do both if you like; the env vars override the defaults.

**Then prove it on the host before the five-minute build:**

```bash
.venv/bin/python -m pytest -q tests/test_submission_assets.py
```

It runs the entrypoint's own shell to derive `MAP`/`LINE`/`CENTER` for each
controller, asserts every one of them is produced by a Dockerfile `COPY`, and
runs preflight on the result. The "edited the default, forgot the `COPY`"
failure cannot survive it.

---

## Step 7 — Build the image (~5 min)

```bash
docker build --platform linux/amd64 --build-arg TAG=$TAG \
  -f submission/Dockerfile \
  -t YOUR_DOCKERHUB_USER/roboracer:iros2026-finals .
```

`--build-arg TAG` matters: the image must be `FROM` the **finals** api image.

---

## Step 8 — Race the image as shipped (~15 min)

```bash
./scripts/test_submission_image.sh YOUR_DOCKERHUB_USER/roboracer:iros2026-finals 12 $TAG
```

No source or data mounts, the image's own entrypoint, the official docker flags;
a helper container does what the organisers do by hand (clicks Connection) and
records the evaluation log. It prints the live `ros2 node list` (expect bridge +
localization + the controller, and nothing else) then the `score_race.py` result.

The script **sets Ultra by default** (`SIM_QUALITY` defaults to `ultra`; it runs
`sim_set_quality.sh ultra` through the helper container), so this test is at the
scoring quality without being asked. Do not override it.

**Pass = 12 laps, 0 collisions**, with an OFFICIAL race time at or below what
step 5.3 measured, **and** `dt_p50` in the Xvfb band you validated at (0.088-0.106 s; `test_submission_image.sh` races on Xvfb, not on the display) — run step 5A's
`lap_report.py` on the evaluation log this script leaves behind, not only
`score_race.py`. A 14-lap sample scatters ~1.5 s, so do not read a small time
difference as a regression; a rate difference, on the other hand, is real. The
one image regression we have seen — 11 collisions in 20 laps from an image whose
100-lap dev runs were clean — showed up as `dt_p50` 0.088 s against 0.104 s and
nothing else.

---

## Step 9 — Push and record the digest (~10-20 min)

```bash
docker push YOUR_DOCKERHUB_USER/roboracer:iros2026-finals
docker image inspect YOUR_DOCKERHUB_USER/roboracer:iros2026-finals \
  --format '{{json .RepoDigests}}'
```

Record the digest in `docs/STATUS.md` and in the submission notes. If the
repository is private, give DockerHub user `autodriveecosystem` collaborator
access.

---

## Step 10 — DockerHub overview text

Paste this into the repository description, with `YOUR_DOCKERHUB_USER` and the
tag filled in:

> Team Proboticists' ROS 2 Humble racing stack for the RoboRacer Sim Racing
> League @ IROS 2026, built FROM the official AutoDRIVE devkit image. The
> upstream `autodrive_roboracer` package is unmodified. All map, centerline and
> raceline assets for the finals track are baked in: no host repository, mounted
> volumes, downloads or interactive setup are needed at runtime. Only LiDAR,
> IMU, encoders and throttle feedback feed the runtime nodes — IPS, odometry, TF,
> reset and scoring topics are not subscribed.
>
> ```bash
> docker pull YOUR_DOCKERHUB_USER/roboracer:iros2026-finals
> # The documented competition-execution path:
> docker run --name autodrive_roboracer_api --rm -it --entrypoint /bin/bash \
>   --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY \
>   --privileged --gpus all YOUR_DOCKERHUB_USER/roboracer:iros2026-finals
> # inside the container:
> /home/autodrive_devkit.sh
> ```
>
> An argument-less `docker run <image>` runs the same script via the image's
> default CMD. Start the official simulator separately, autonomous mode, Ultra
> graphics, then press **Connection** — the bridge, localization and MPC are
> already up and waiting for sensor data. A second shell
> (`docker exec -it autodrive_roboracer_api bash`) starts no second controller.
> Orderly shutdown: `docker stop --time 10 autodrive_roboracer_api`; the stack
> zeroes both actuators before the bridge goes down.
>
> Optional environment overrides (defaults are the validated finals config):
>
> | Variable | Default |
> |---|---|
> | `PIT_CREW_MAP` | `/home/data/maps/finals_mesh` |
> | `PIT_CREW_LINE` | `/home/data/racelines/finals_mesh_race.csv` |
> | `PIT_CREW_CENTERLINE` | `${PIT_CREW_MAP basename}_centerline.csv` |
> | `PIT_CREW_LIDAR_OFFSET` | `0.1933` |
> | `PIT_CREW_CONTROLLER` | `mpc3` (`mpc` = legacy tracker + centerline) |
> | `PIT_CREW_SPEED_SCALE` | `1.00` |
> | `PIT_CREW_MODE` | `raceline` (`reactive` = map-free Follow-the-Gap) |

---

## Step 11 — Submit the form (10 min)

Submit the exact `repository:tag` using the submission form linked from the
[event page](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/).
**A GitHub push is not a submission.** Publication to DockerHub *and* the form
must both be complete before **September 19, AoE**.

---

## Pre-flight checklist — failure modes and fixes

| Symptom | Cause | Fix |
|---|---|---|
| `lidar_offset mismatch: map says 0.0000 m, PIT_CREW_LIDAR_OFFSET says 0.1933` | A recon map (built at offset 0) raced at the mesh offset | Set `PIT_CREW_LIDAR_OFFSET=0` for a recon map, `0.1933` for a mesh map. Never mix. |
| `<map>.yaml has no lidar_offset` | Map not produced by `map_from_mesh.py` | Re-run `map_from_mesh.py`, or stamp the key deliberately. |
| `no centerline for <slug>: <path>` at startup | The centerline was not COPYed, or is not named `<map basename>_centerline.csv` | Add it to the Dockerfile raceline list under exactly that name. The entrypoint derives it from `PIT_CREW_MAP`. |
| `raceline/centerline ... leaves the map` or `runs through walls on this map` | Asset from a different track, or a map built with a different offset | The classic finals mistake: three consistent-looking files describing two circuits. Rebuild all three from the same drill run. |
| `... is not a closed loop: X m between its last point and its first` | Open line; the trackers treat it as cyclic | Regenerate the line; do not hand-edit CSVs. |
| `raceline requires positive speeds and increasing arc length` | Corrupt/intermediate candidate | Ship only a drill output that passed the gates. |
| `REFUSING: host load N; a scored run needs a quiet host` | Something else is computing | Stop it and re-run. `LOAD_OK=1` exists but any run under load is contaminated. |
| `lap_report.py` shows `dt_p50` ~0.088-0.091 s | Validated at Low quality, something else is using the GPU, or simply the per-start lottery | Valid evidence (`rate_adaptive` closed the old fast-instance failure), but it is not the ~0.097 s Gate C band. Re-run with `SIM_QUALITY=ultra`, `SIM_CPUSET` and `RATE_MIN_DT`/`RATE_MAX_DT` on an idle host. |
| `dt_p50` drifts across the laps of one run | A process started mid-run | Contaminated. Re-run on a quiet host; do not average it out. |
| `WARN: could not set Ultra` | The quality click missed (same Unity press/release quirk as Connection) | Check the label crop in `/tmp/sim_quality/quality_after_label.png`; re-run `scripts/sim_set_quality.sh next --verify` until it reads Ultra. Never during a scored run — Reset and Quit are the next rows down. |
| `OFFICIAL race time unavailable ...` from `score_race.py` | Log written by a logger with no `last_lap_time` column | Re-run with the current logger. Do not decide the finals config on the sample clock. |
| `UNVERIFIED: ...` from `score_race.py` | Log starts after warm-up, or has a counter reset or a missing lap transition | Re-run. The scorer refuses to certify ambiguous telemetry on purpose. |
| `ERROR: could not connect` from `test_submission_image.sh` | The Connection click missed (Unity needs the pointer to *enter* the control and press/release in separate frames) | The script already retries 3x with a toggle-off click. Just re-run it; check the sim window actually appeared. |
| `docker: Conflict. The container name "autodrive_roboracer_api" is already in use` | Leftover container from a previous run | `docker rm -f autodrive_roboracer_api autodrive_roboracer_sim pit_helper` (`test_submission_image.sh` does this itself at the start). |
| `Package 'pit_crew' not found` | `sim_restart_connect.sh` restarts the devkit container and wipes the colcon build | `colcon build --packages-select pit_crew --symlink-install` inside the container; the scripts do it for you. |
| Car circulates but scores zero laps | Running the track backwards (Follow-the-Gap has no notion of travel direction) | Do not ship `PIT_CREW_MODE=reactive` unless everything else has failed. |
| Sensors look frozen | Headless `-batchmode -nographics` freezes the GPU-raytraced LiDAR | Always a real X server (Xvfb `:99` is fine and is what the scripts use). |
| Lap/collision counts look absurd | They are **cumulative** and reset only when the sim *container* restarts | Every tool here differences against the run's first row; do the same by hand. |

---

## If it is September 19 and nothing is clean

Ship in this order, taking the first thing that is not collecting collisions.
Something imperfect that finishes 12 laps beats a DQ (>10 collisions) and beats
no submission at all.

1. **Centerline with mpc3.** `PIT_CREW_LINE=/home/data/racelines/finals_mesh_safe.csv`,
   `PIT_CREW_CONTROLLER=mpc3`. **Not `_centerline.csv`:** that file is 4 columns
   (x, y, w_right, w_left) with no speed profile, it is what localization seeds
   from, and preflight rejects it as a line. `_safe.csv` is the corridor middle
   *with* a profile. The corridor middle,
   0.5-0.6 m of room. This is what ICRA-2026 ships and it was 0/30 there where
   the width-using line took 9 collisions.
2. **Legacy tracker at ay 4.5** — the rate-proven option, clean at both 11 Hz
   and Ultra. `PIT_CREW_CONTROLLER=mpc` with the centerline
   (the entrypoint pairs `mpc` with `<slug>_safe.csv` automatically; **never pair
   `mpc` with a wide min-curvature line**). If the line itself is the problem,
   rebuild it at the conservative profile:
   `.venv/bin/python tools/make_track_lines.py finals_mesh --ay 4.5 --min-clearance 0.30`
   — ay 4.5 / jerk 40 is the old shipping cap that ran 36 laps clean.
3. **Reactive.** `PIT_CREW_MODE=reactive` — map-free Follow-the-Gap, needs no
   map, no line and no localization, and clears qualification on any track. It
   is genuinely weak on a narrow circuit (24 collisions in 194 s on 2025 ICRA,
   which would be a DQ) and has been seen to circulate backwards, so this is the
   last resort, not a plan. Tune the bubble down for a tight track via
   `src/pit_crew/config/ftg_mapping.yaml`.

The entrypoint already falls back on its own: a dead node is restarted up to 4
times, then the whole stack drops to the reactive fallback rather than losing
the remaining laps. A dead ROS bridge is the one unrecoverable failure.

Whatever you ship, re-run step 8 against the exact image you push, and record
the digest before any further experiment.
