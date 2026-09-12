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

Order of combos:

**5.1 — mpc3 on the race line, only if the pipeline produced one that passed
the kappa 1.0 gate** (~15 min for 32 laps):

```bash
CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_race_probe race
```

**5.2 — otherwise (or if 5.1 collides), mpc3 on the centerline/safe line:**

```bash
CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh finals_safe_probe safe
```

**5.3 — 100-lap validation of whichever of 5.1/5.2 was clean** (~1 h):

```bash
CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=100 \
  ./scripts/run_exp.sh finals_100 <race|safe>
```

**Pass = 0 collisions over 100 laps on a quiet host.** 32 laps is a probe, not
a pass: it cannot resolve a 1-3 %/lap collision rate (measured — 0.04
`throttle_a_gain` was 0/32 and 8+/100).

Every run appends a row to `data/experiments.tsv` and prints
`laps= coll= median= best= mean=` plus the `score_race.py` line. Score by hand
with:

```bash
.venv/bin/python tools/score_race.py data/logs/race_<ts>.csv
```

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

**Pass = 12 laps, 0 collisions**, with a sim-timer race time at or below what
step 5.3 measured. Also validate once at **Ultra graphics with `VISIBLE=1`**
before submitting, since that is how it will be scored.

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
2. **Legacy tracker at ay 4.5.** `PIT_CREW_CONTROLLER=mpc` with the centerline
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
