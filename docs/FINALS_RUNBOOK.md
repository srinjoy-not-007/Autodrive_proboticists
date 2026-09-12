# Finals runbook — the day the finals image drops

Ordered, copy-pasteable procedure for the finals track. The compete image is
released **September 16**; the submission is due **September 19, AoE**. Every
command below is one that already exists in this repo — no invented flags.

Facts about how the run is scored (verified from the official repo, recorded in
`docs/STATUS.md`): organisers run **Ultra graphics, GUI**, start the team image
with `docker run ... --entrypoint /bin/bash -it` and then run
`/home/autodrive_devkit.sh` inside it, click **Connection** in the simulator to
start the race, drive **12 laps** (warm-up, 10 scored, cool-down), sum the
**simulator's own lap timer**, add **+10 s per collision**, and **more than 10
collisions is a DQ**.

**Ultra graphics is not cosmetic — it sets the sensor rate, and mpc3 is
rate-sensitive.** The whole stack sees the world at whatever period the
simulator's bridge delivers, and that period depends on graphics quality and on
how busy the host is. Measured on this host (Xvfb): Low 11.05 Hz, Ultra
10.64 Hz; the logger's own per-lap period is 0.088-0.091 s at Low and
0.102-0.105 s at Ultra. mpc3 is **clean at ~0.104 s and collides at ~0.088 s**
(faster arrival, later braking) — every "clean century" it has ever run was a
9.6 Hz result. The organisers measured **~0.097 s** on their RTX 4090 at Ultra,
i.e. between the clean and the colliding rate. So:

* **Validate at Ultra.** Pass `SIM_QUALITY=ultra` to `run_exp.sh` /
  `run_stanley.sh`; `test_submission_image.sh` already sets Ultra by default.
* **Check the period on every scored run** (the `dt_p50` gate in step 5).
* A run at the wrong rate is not evidence about the finals, in either
  direction.

Total wall clock for a clean pass through this runbook: **~4-6 h**, dominated by
the 100-lap validation runs (step 5). Budget one full day.

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
and `data/racelines/finals_mesh_{centerline,safe,raceline,race}.csv`.

### What each printed gate means

| Stage | Gate | Meaning if it fails |
|---|---|---|
| 2/4 free fraction | must be **15-70 %** (every known circuit gives 30-55 %) | The mesh or its parent transform is wrong — e.g. the `Infrastructure` root at Unity (-10, 0, -14) was missed, so the spawn lands outside the corridor and the flood fill leaves everything unknown; or the track mesh carries a floor and rasterised solid. Re-run `map_from_mesh.py` with `--report` and `--list`. |
| 2/4 `source: mesh` | asserted | The YAML was not stamped by `map_from_mesh.py`. Nothing downstream will take the mesh code path. |
| 2/4 `lidar_offset` | must print **0.1933** | This is what the entrypoint and the preflight will be checked against. |
| 3/4 `clearance X.XXX m ideal lap Y.YY s` per candidate | per-candidate report | Informational unless followed by `REJECTED:`. |
| 3/4 `REJECTED: clearance ... < 0.28 m budget` | `MESH_MIN_CLEARANCE` = 0.28 (0.230 collided on ICRA, 0.292 clean on Porto) | That candidate is too close to a wall for our lateral error. Normal — the sweep rejects several. |
| 3/4 `REJECTED: kappa max K above the 1.0` | `MESH_RACE_KAPPA_MAX` = 1.0 | The width-using race line is tighter than anything mpc3 has held (Porto 0.83 clean over 100 laps; ICRA 1.36 → 9 collisions in 30 laps). **When this fires, the centerline ships** — that is the intended outcome, not a failure. |
| 3/4 `no solution (corridor exhausted)` | the QP ran out of corridor at that `w_veh` | A fact about the track, not an error. |
| 3/4 `safe line is not drivable; shipping the opened-corner line instead` | graceful degrade | Accept it, but drive it before trusting it. |
| 4/4 `footprint_analysis.py` on `finals_mesh_race.csv` | worst-corner body clearance | This is the number that decides step 5's first run. |

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

If the recon drill also produces nothing raceable, ship the **centerline**
(`finals_*_centerline.csv` / `_safe.csv`) — it is the corridor middle, has
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
(the cycle Low -> High -> Ultra is verified). Omitting it validates at Low —
about 11 Hz — which is the rate mpc3 collides at.

Order of combos:

**5.1 — mpc3 on the race line, only if the pipeline produced one that passed
the kappa 1.0 gate** (~15 min for 32 laps):

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

**5.4 — the rate-proven fallback, run it too** (~15 min). The legacy tracker on
the centerline is slower but has been clean at both rates, so it is the thing
you ship if mpc3 cannot be made clean at Ultra:

```bash
SIM_QUALITY=ultra CONTROLLER=mpc MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_legacy_probe safe
```

**Pass = 0 collisions over 100 laps on a quiet host.** 32 laps is a probe, not
a pass: it cannot resolve a 1-3 %/lap collision rate (measured — 0.04
`throttle_a_gain` was 0/32 and 8+/100).

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
| 0.102-0.106 s | The rate every clean mpc3 century was measured at. Valid evidence. |
| ~0.097 s | The organisers' measured rate. The most valuable number to have, and the one nothing is yet proven at. |
| 0.088-0.091 s | mpc3 collides here (3/10 laps, 11 collisions in 20 in the image run). **Discard the run as evidence about the finals** — but not as evidence about rate sensitivity. |
| Varies by more than ~0.005 s across the laps of one run | Something else ran on the host. Contaminated; re-run. |

If a run lands off Ultra's expected 0.102-0.105 s, the first suspects are
`SIM_QUALITY=ultra` not having been passed (or its click having missed — the
script prints a WARN and saves a label crop under `/tmp/sim_quality`) and
another process on the host.

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
| 1 | **mpc3 on the race line** (`finals_mesh_race.csv`) | The pipeline produced a race line that passed the `MESH_RACE_KAPPA_MAX` 1.0 gate in step 3a, **and** a 100-lap run at `SIM_QUALITY=ultra` on a quiet host was **0 collisions** with `dt_p50` in band. Both, not either. |
| 2 | **mpc3 on the centerline/safe line** (`finals_mesh_safe.csv`) | The kappa gate rejected the race line, or the race line collided, and mpc3 on the safe line passed the same 100-lap Ultra gate. This is what ICRA-2026 ships (0/30 where the width-using line took 9 collisions in 30). |
| 3 | **Legacy `mpc` on the centerline** (`PIT_CREW_CONTROLLER=mpc`) | Neither mpc3 configuration is clean at Ultra. Slower — 78.2-78.4 s on Porto against mpc3's ~73 s — but it is the **rate-proven** option: clean at 11 Hz (32 laps, 0 collisions) *and* at Ultra (32 laps, 0 collisions), where mpc3 at Ultra took 2 collisions in 100 laps. The entrypoint pairs `mpc` with the centerline automatically; **never pair `mpc` with the wide min-curvature line.** |

A 2 %/lap mpc3 still beats a clean legacy on expected race time (~75 s against
78.4 s), so row 3 is for "mpc3 is visibly unreliable at Ultra", not for "mpc3
took one collision in 100". Decide on the Ultra numbers, with the collision
rate and the +10 s penalty in front of you — and remember >10 collisions is a DQ.

### 5D — Experimental knobs: `rate_adaptive` and ILC (do not ship blind)

`src/pit_crew/mpc3.py` declares two opt-in mechanisms, both defaulting to
**off**, and **neither validated live**:

* **`rate_adaptive`** (with `rate_nominal`, `rate_window`, `rate_min_change`,
  `rate_terms`) — the identified cause of the rate sensitivity. `apply_ahead_s`
  was a constant 0.090 s, i.e. 0.87 of the 0.104 s period mpc3 was tuned at;
  scaling it with the measured period is identity at 0.104 s and, **offline**,
  reproduces and removes the 0.104 -> 0.088 s lateral-error rise (p50 0.075 ->
  0.093 offline, 0.068 -> 0.093 live). The period is always measured; the flag
  only decides whether the tracker acts on it.
* **`ilc_enable`** (plus the `ilc_*` family; `ilc_v_enable` and `ilc_store` are
  separately opt-in) — steering feedforward over 0.25 m bins, converging by lap
  8-10 to p50 0.03 offline but costing 0.6-0.8 s/lap there. The speed-ceiling
  table is a net loss in the harness and is off within the flag.

**There is no `PIT_CREW_*` environment variable for either of these.** Nothing
in `submission/autodrive_devkit.sh` maps env to mpc3 parameters; the entrypoint
passes only `raceline_csv`, `speed_scale` and `map_prefix` (via `CTRL_ARGS`).
Using one in the finals image therefore means **editing the entrypoint** to add
an explicit `-p` to `start_ctrl`, e.g. appending to `CTRL_ARGS`:

```bash
CTRL_ARGS=(-p map_prefix:="$MAP" -p rate_adaptive:=true)
```

Test them the same way, with an explicit `-p` after the line argument:

```bash
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_rateadapt race -p rate_adaptive:=true
```

**Do not put either flag in the shipped image without its own clean 100-lap
Ultra run**, scored the same way as every other row of the decision table. As of
today their evidence is offline only, and the offline harness does not reproduce
the live speed lag (0.5 m against the measured 1.45 m) that mpc3 was implicitly
tuned around.

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
because the organisers run the script with no environment at all. Change the
`MAP` default to `/home/data/maps/finals_mesh` and, in the `mpc3` branch, the
`LINE` default to the validated finals line. `CENTER` derives itself from
`basename "$MAP"` as `<slug>_centerline.csv`, so it follows automatically —
which is exactly why the centerline file must be named and copied that way.

Do both if you like; the env vars override the defaults.

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
step 5.3 measured, **and** `dt_p50` in the 0.102-0.106 s band — run step 5A's
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
| `lap_report.py` shows `dt_p50` ~0.088-0.091 s | Validated at Low quality, or something else is using the GPU | Re-run with `SIM_QUALITY=ultra` on an idle host. mpc3 collides at this rate; the run is not evidence about the finals. |
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

1. **Centerline with mpc3.** `PIT_CREW_LINE=/home/data/racelines/finals_mesh_safe.csv`
   (or `_centerline.csv`), `PIT_CREW_CONTROLLER=mpc3`. The corridor middle,
   0.5-0.6 m of room. This is what ICRA-2026 ships and it was 0/30 there where
   the width-using line took 9 collisions.
2. **Legacy tracker at ay 4.5** — the rate-proven option, clean at both 11 Hz
   and Ultra. `PIT_CREW_CONTROLLER=mpc` with the centerline
   (the entrypoint pairs `mpc` with a centerline automatically; **never pair
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
