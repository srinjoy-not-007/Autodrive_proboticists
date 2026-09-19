# Submission image — IROS 2026

Qualification was due **September 12, 2026, AoE**; the final submission is a
separate deadline on **September 19, 2026, AoE = September 20, 17:30 IST**. The
finals image is `bongracers/roboracer:iros2026-finals`, built from the SRL 2026
IROS track assets (`2026-iros-compete`), which are the image defaults. Checked against the
[event page](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/),
[rules](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-rules-2026/),
and [technical guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/).

This file documents the image: how to build it, how an evaluator runs it, and
every knob it exposes. The step-by-step procedure for the day the finals track
drops — map/line generation, validation runs, scoring and the shipping decision —
lives in [`docs/FINALS_RUNBOOK.md`](../docs/FINALS_RUNBOOK.md). Measured results
live in [`docs/STATUS.md`](../docs/STATUS.md). No lap times are quoted here, so
that this file cannot go stale.

## Build and publish

`--build-arg TAG` selects the devkit base image and must name the track image
being raced: `2026-iros-compete` for the final submission,
`2026-iros-practice` for qualification. It **defaults to `2026-iros-compete`**,
so a build that forgets the flag still gets the finals base image; the
qualification image is now the one that needs the flag typed on purpose.
`--build-arg FINALS_SLUG` names the
raceline set to COPY and already defaults to `finals_mesh`, so the finals build
takes no edit.

```bash
docker build --platform linux/amd64 --build-arg TAG=2026-iros-compete \
  -f submission/Dockerfile \
  -t bongracers/roboracer:iros2026-finals .
docker push bongracers/roboracer:iros2026-finals
docker image inspect bongracers/roboracer:iros2026-finals \
  --format '{{json .RepoDigests}}'
```

Record the pushed digest in the submission notes. If the repository is private,
give DockerHub user `autodriveecosystem` collaborator access. Submit the exact
`repository:tag` using the form linked from the event page. A GitHub push is not
a submission: publication *and* the form must both be complete before the
deadline.

## What the image contains

Built `FROM autodriveecosystem/autodrive_roboracer_api:<TAG>`. The upstream
`autodrive_roboracer` package is **unmodified**; our code is a separate ROS 2
package, `pit_crew`.

- `pit_crew` at `/home/autodrive_devkit/src/pit_crew`, colcon-built at image
  build time (`--packages-select pit_crew --symlink-install`).
- `pip3 install "numpy<2" "scipy==1.10.1" pyyaml`. numpy is pinned below 2
  deliberately: the base image's ROS 2 Humble C-extensions (cv_bridge among
  them) are built against the 1.x ABI, and we may not patch the devkit to fix
  what an unpinned upgrade breaks.
- Maps in `/home/data/maps/`: `track_v2.*` (legacy recon) plus everything
  matching `*_mesh.pgm` / `*_mesh.yaml`. The glob is intentional so a finals
  mesh map ships with no Dockerfile edit; a non-mesh recon map must be added by
  hand.
- Racelines and centerlines in `/home/data/racelines/`, **listed one by one and
  never globbed**, so no undriven candidate is one `PIT_CREW_LINE` typo away
  from being raced. `porto_mesh_centerline.csv` / `porto_mesh_safe.csv` are
  copied from the `track_v2` files under the names the entrypoint derives from
  `PIT_CREW_MAP`.
- `/home/autodrive_devkit.sh` — the entrypoint script, the only thing that
  starts a race. **Nothing runs on shell entry.** The base image's
  `/root/.bashrc` sources `/opt/ros/humble/setup.bash` and
  `/home/autodrive_devkit/install/setup.bash` and starts no node; our
  Dockerfile never writes to it, and there is no `.bash_profile`, `.profile`,
  `/etc/bash.bashrc` or `/etc/profile` content of ours either (verified by
  extracting them from both images). `scripts/test_submission_image.sh` asserts
  the live `ros2 node list` is unchanged by a second `docker exec` shell.

`ENTRYPOINT ["/bin/bash"]` is kept (the rules and every helper pass `-c "..."`),
with `CMD ["-c", "exec /home/autodrive_devkit.sh"]` added so an argument-less
`docker run` boots the stack instead of idling. No mounted volumes, downloads or
interactive setup are needed at runtime.

## DockerHub overview / evaluator instructions

Paste the block below into the DockerHub repository description for
`bongracers/roboracer:iros2026-finals`.

> Team Proboticists' ROS 2 Humble racing stack for the **RoboRacer Sim Racing
> League @ IROS 2026 final round**, built FROM the official AutoDRIVE devkit
> image. The upstream `autodrive_roboracer` package is unmodified. All assets
> for the **finals track — the SRL 2026 IROS Track (`2026-iros-compete`)** are
> baked in and are the defaults: map, centerline and the validated raceline. No
> host repository, mounted volumes, downloads or interactive setup are needed at
> runtime. Only LiDAR, IMU, encoders and throttle feedback feed the runtime
> nodes — IPS, odometry, TF, reset and scoring topics are not subscribed. The
> Porto practice track is still available with
> `-e PIT_CREW_MAP=/home/data/maps/porto_mesh`.
>
> ```bash
> xhost local:root
> docker pull bongracers/roboracer:iros2026-finals
> # the documented competition-execution path:
> docker run --name autodrive_roboracer_api --rm -it --entrypoint /bin/bash \
>   --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY \
>   --privileged --gpus all bongracers/roboracer:iros2026-finals
> # inside the container:
> /home/autodrive_devkit.sh
> ```
>
> An argument-less `docker run bongracers/roboracer:iros2026-finals` runs the
> same script through the image's default `CMD`. Start the official simulator
> separately (autonomous mode, **Ultra** graphics), then press **Connection** —
> the bridge, localization and the MPC are already up and waiting for sensor
> data. A second shell (`docker exec -it autodrive_roboracer_api bash`) starts no
> second controller.
>
> If **Connection** was pressed before this container was started, press it
> again: the simulator is the WebSocket client and the bridge in this container
> is the server, so a press made before the bridge is listening has nothing to
> connect to. (Architecture, not a measured retry policy — pressing again costs
> nothing and is the safe habit.)
>
> Orderly shutdown: press **Ctrl-C** in the shell running the script, or --
> if it was started through the image's default `CMD` -- `docker stop --time 10
> autodrive_roboracer_api`; the stack zeroes both actuators before the bridge
> goes down. On the `-it --entrypoint /bin/bash` path `docker stop` reaches
> only the interactive shell, which ignores `SIGTERM`, so Ctrl-C is the
> orderly stop there. The team nodes are CPU-only.
>
> Optional environment overrides (the defaults are the validated finals config):
>
> | Variable | Default |
> |---|---|
> | `PIT_CREW_MAP` | `/home/data/maps/finals_mesh` |
> | `PIT_CREW_LINE` | `/home/data/racelines/finals_mesh_apex_f42k_race.csv` |
> | `PIT_CREW_CENTERLINE` | `/home/data/racelines/<map basename>_centerline.csv` |
> | `PIT_CREW_LIDAR_OFFSET` | `0.1933` |
> | `PIT_CREW_CONTROLLER` | `mpc3` (`mpc` = legacy tracker + safe line) |
> | `PIT_CREW_SPEED_SCALE` | `1.00` |
> | `PIT_CREW_MODE` | `raceline` (`reactive` = map-free Follow-the-Gap) |

Both run paths execute the same script. The organisers' documented procedure
([official repo, "Competition Execution"](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing#competition-execution))
uses `--entrypoint /bin/bash -it`, which lands in a shell and starts nothing, so
the stack is started from that shell:

```bash
xhost local:root
docker pull bongracers/roboracer:iros2026-finals
docker run --name autodrive_roboracer_api --rm -it --entrypoint /bin/bash \
  --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY \
  --privileged --gpus all bongracers/roboracer:iros2026-finals
# inside the container:
/home/autodrive_devkit.sh
```

An argument-less run does the same thing via the image's default `CMD`:

```bash
docker run --name autodrive_roboracer_api --rm \
  --network=host --ipc=host --stop-timeout=10 \
  bongracers/roboracer:iros2026-finals
```

Start the official simulator separately, use autonomous mode and **Ultra**
graphics, then press **Connection**. The bridge, localization and the controller
are already up and waiting for sensor data. Opening another shell starts no
second controller:

```bash
docker exec -it autodrive_roboracer_api bash
# inside the additional shell, for inspection/recording only:
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
# from the host, orderly shutdown:
docker stop --time 10 autodrive_roboracer_api
```

The team nodes are CPU-only and need no GPU; the official simulator keeps its own
GPU/display requirements. On shutdown the script stops the producer nodes first,
publishes zeros to `throttle_command` and `steering_command` while the bridge is
still alive (commands latch), then stops the bridge. Abrupt host loss or
SIGKILL cannot run that cleanup -- **and neither can `docker stop` on the `-it
--entrypoint /bin/bash` path**, where PID 1 is the interactive shell and an
interactive bash ignores `SIGTERM`: the script never sees it and is killed when
the grace period ends, leaving the last commands latched in the simulator. On
that path the orderly stop is **Ctrl-C** in the shell running the script
(`trap 'exit 130' INT` runs the same cleanup). The `CMD` path, where the script
is PID 1, takes `docker stop` correctly.

## Environment variables

All are read by `submission/autodrive_devkit.sh`; defaults are the validated
shipping config, so the organisers can run the script with no environment at
all. Nothing else is read.

| Variable | Default | Meaning |
|---|---|---|
| `PIT_CREW_MAP` | `/home/data/maps/finals_mesh` | Map prefix (`.pgm` + `.yaml`). Its basename is the slug used to derive the centerline. Exact mesh map from `tools/map_from_mesh.py`. |
| `PIT_CREW_LINE` | Derived from the map slug: `porto_mesh_cr_dk0.8.csv` for `mpc3` on `porto_mesh`, otherwise `<slug>_race.csv` for `mpc3` and `<slug>_safe.csv` for `mpc`, all under `/home/data/racelines/` | The raceline to track (6 columns: s, x, y, heading, curvature, speed). **Not `<slug>_centerline.csv`** — that file is 4 columns (x, y, w_right, w_left) with no speed profile, and preflight rejects it as a line. The `_safe.csv` line is the corridor middle *with* a profile. |
| `PIT_CREW_CENTERLINE` | `/home/data/racelines/<map basename>_centerline.csv` | Derived from `PIT_CREW_MAP`, never from a fixed Porto path. **Not a diagnostic:** localization seeds its whole particle cloud at `centerline[0]` and resamples along it on every collision recovery, so a centerline from another track is a race with no laps in it. |
| `PIT_CREW_LIDAR_OFFSET` | `0.1933` | LiDAR forward offset used by the filter, in metres. Must match the `lidar_offset` stamped in the map YAML — preflight rejects a mismatch. A recon map built at 0 must be raced at 0. |
| `PIT_CREW_CONTROLLER` | `mpc3` | `mpc3` or `mpc` only; anything else exits 2 (use `PIT_CREW_MODE=reactive` for Follow-the-Gap). `mpc3`/`mpc2` are passed `-p map_prefix:=` for the corridor barrier; the legacy `mpc` does not declare it. `mpc` defaults to the centerline — **never pair `mpc` with the wide min-curvature line.** |
| `PIT_CREW_SPEED_SCALE` | `1.00` | Multiplier on the line's speed profile; preflight requires finite and in (0, 1.5]. |
| `PIT_CREW_MODE` | `raceline` | `raceline` = localization + controller. `reactive` = map-free Follow-the-Gap only (no map, line, centerline or preflight). Any other value exits 2. |
| `PIT_CREW_CONTROLLER_ARGS` | *(empty)* | Extra `-p name:=value` arguments appended verbatim to the controller at **every** launch, initial and supervisor restart. Word-split on purpose, e.g. `"-p rate_adaptive:=true -p ilc_enable:=true"`. See the runbook's [5D](../docs/FINALS_RUNBOOK.md#5d--what-ships-what-is-opt-in-and-what-stays-off) for what is already on (`rate_adaptive`, `w_dcmd`, `cmd_tau`), what is prepared but not enabled, what is opt-in (`steer_tau`, `gap_predict_s`, the teleport and telemetry-gap sets) and what stays off (ILC, `nd_mode`, the spatial lead, `apply_floor_steps`). Appended **after** the script's own `CTRL_ARGS`, so an override here wins. |
| `PIT_CREW_LOCALIZATION_ARGS` | *(empty)* | Same, for the localization node. Appended **after** the script's own `LOC_ARGS` (which ships `max_step_mps:=9.0 publish_on_reloc:=false`), so an override here wins (ROS takes the last `-p name:=value` for a repeated name). |
| `PIT_CREW_FTG_PARAMS` | `…/src/pit_crew/config/ftg_mapping.yaml` | Parameter file for the reactive stack (`PIT_CREW_MODE=reactive`, and the supervisor's last-resort fallback). The default is the **mapping** profile, `v_max` 1.0 m/s: deliberately slow, and the only profile carrying the stuck/escape keys that unwedge an unattended car. `…/config/ftg.yaml` is the faster racing profile (`v_max` 1.5) with no escape behaviour. Which is right depends on how tight the finals circuit is — Follow-the-Gap took 24 collisions in 194 s on SRL 2025 ICRA, which is a DQ. Measured on the FINALS map with `tools/ftg_sim.py` (Sep 19): the mapping profile is 12 laps / 1 collision / ~57 s per lap, `ftg.yaml` 12 / 7. **The entrypoint only checks that this file exists when `PIT_CREW_MODE=reactive` was asked for, never on the supervisor's fallback path** -- `tests/test_submission_assets.py::TestTheReactiveFallbackIsInTheImage` covers it on the host instead. |

`OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS` and `MKL_NUM_THREADS` are pinned to 1
by the script: the controller's numpy solves are 20x30 matrices, and OpenBLAS
spinning every core on them starved the simulator and the bridge on the same
host (measured 1030% CPU). One thread is faster at this size anyway. This
changes the achieved sensor rate, which matters — see the logger-period gate in
the runbook ([5A](../docs/FINALS_RUNBOOK.md#5a--the-logger-period-gate-mandatory-on-every-scored-run)).

## What starts, and what happens when something dies

In `raceline` mode the script runs `python3 -m pit_crew.preflight` **before any
actuator node**, cross-checking that the map, line and centerline describe the
same circuit, that no asset runs through walls or is an open loop, that the
speed scale is sane, and that the map's stamped `lidar_offset` matches
`PIT_CREW_LIDAR_OFFSET`. A failure exits non-zero and nothing is commanded.

Then, each in its own session (`setsid`, so the whole process group can be
signalled):

1. `ros2 launch autodrive_roboracer bringup_headless.launch.py` — the devkit
   bridge.
2. `raceline` mode: `pit_crew localization` (`map_prefix`, `centerline_csv`,
   `lidar_offset`) and `pit_crew $CONTROLLER` (`raceline_csv`, `speed_scale`,
   plus `map_prefix` for `mpc3`/`mpc2`).
   `reactive` mode: `pit_crew follow_the_gap` with
   `src/pit_crew/config/ftg_mapping.yaml`.

A supervisor loop then `wait -n`s on the children:

- **Bridge exits** → unrecoverable; shut down with status 1.
- **A worker exits** (even successfully) → restart only the node that died, so a
  converged particle filter is not thrown away mid-lap, with a 0.5 s pause so a
  node that dies instantly cannot spin. Up to `MAX_RESTARTS` = 4.
- **More than 4 restarts in `raceline` mode** → stop the race stack and fall
  back to `reactive` (Follow-the-Gap needs no map, no line and no localization),
  with the restart counter reset so the fallback gets its own 4 attempts.
  **`RESTARTS` is cumulative over the whole session and is never reset on a
  successful recovery**, so five node deaths spread over twelve laps drop the
  race to Follow-the-Gap even though every one of them was recovered from. At
  ~57 s per fallback lap that is the difference between a race and a finish.
- **Reactive fallback also exhausts its restarts** → exit 1.

This exists because the organisers run all 12 laps in one go: a single unhandled
exception used to cost every remaining lap and the whole score.

## Acceptance checklist

Race the image exactly as the organisers do — no source or data mounts, the
image's own entrypoint, the official docker flags, Ultra graphics — with:

```bash
# the organisers' documented path: --entrypoint /bin/bash, script run inside
./scripts/test_submission_image.sh bongracers/roboracer:iros2026-finals 12
# the argument-less CMD path, which must work too
ENTRY_MODE=cmd ./scripts/test_submission_image.sh bongracers/roboracer:iros2026-finals 12
.venv/bin/python tools/score_race.py data/logs/submission_<ts>.csv
```

The script runs `xhost local:root` first, sets Ultra, and checks that a second
`docker exec` shell adds no node. `ENTRY_MODE` defaults to `bash`.

- Run the image with no mounts; verify the bridge and both team nodes come up.
- Wait before connecting; confirm the car only begins when Connection is pressed.
- Open a second shell; confirm it creates no second controller (the script
  does this automatically and fails the run if the node list changes).
- Run the host asset check before building:
  `.venv/bin/python -m pytest -q tests/test_submission_assets.py`. It derives
  the entrypoint's `MAP`/`LINE`/`CENTER` defaults by running the entrypoint's
  own shell and asserts every one of them is produced by a Dockerfile `COPY`
  and passes preflight — the "edited the default, forgot the COPY" failure.
- Record one warm-up lap plus ten simulator-counted race laps, and score them
  with `tools/score_race.py`. Quote the **`OFFICIAL (simulator last_lap_time)`**
  line and its `+10 s` collision penalties — never `10 * median`, geometric
  laps, or the sample clock. `UNVERIFIED:` or `OFFICIAL race time unavailable`
  means re-run, not argue. See the runbook's
  [5B](../docs/FINALS_RUNBOOK.md#5b--scoring-use-the-official-numbers).
- Check the logger period against the runbook's
  [5A](../docs/FINALS_RUNBOOK.md#5a--the-logger-period-gate-mandatory-on-every-scored-run)
  gate. The controller's behaviour depends on the sensor rate, so a run at the
  wrong rate proves nothing either way.
- `test_submission_image.sh` races the image on **Xvfb** (~0.09-0.11 s). The
  organisers run a real display, which on this host is ~0.058 s and is where
  four earlier configurations collided. A clean image test is not a substitute
  for the runbook's real-display gate (`VISIBLE=1 ... ./scripts/run_exp.sh`);
  run both.
- Test stale sensor inputs, child-process failure (kill a worker, confirm only
  it restarts) and orderly shutdown (Ctrl-C on the `-it` path, `docker stop
  --time 10` on the `CMD` path; actuators zeroed either way).
- Measure the reactive fallback on the track being raced before trusting it:
  `.venv/bin/python tools/ftg_sim.py finals_mesh --params
  src/config/ftg_mapping.yaml --laps 12`. On the finals map (Sep 19) the
  shipped `ftg_mapping.yaml` profile completes 12 laps in the right direction
  with 1 collision at ~57 s/lap; `ftg.yaml` (`v_max` 1.5) takes 7. The fallback
  is survivable here, not competitive -- the right shape for a last resort, but
  it is only ever reached after five node deaths, and the restart counter that
  gets there never resets (see below).
- Preserve the validated image digest before any performance experiments, and
  keep experiments on a separate branch.

Only LiDAR, IMU, encoders and throttle feedback feed the runtime team nodes. IPS,
odometry, TF, reset and scoring topics are excluded from their inputs;
development loggers may use them for evaluation but are never launched by this
image.

## Which config to ship

Do not decide from this file. The runbook's
[controller decision table (5C)](../docs/FINALS_RUNBOOK.md#5c--controller-decision-table-for-the-finals)
is the rule: take the first row whose condition holds — `mpc3` on the race line,
`mpc3` on the safe line, or the rate-proven legacy `mpc` on the centerline —
decided on a 100-lap run at Ultra on a quiet host, scored the official way.
Remember that a collision is +10 s and more than ten is a DQ.

Point the shipped image at the chosen assets either by editing the defaults in
`submission/autodrive_devkit.sh` (preferred, because the organisers run the
script with no environment) or with the environment variables above; see the
runbook's Step 6. Add the finals centerline **and** the one validated line to
the Dockerfile's raceline list at the `ADD THE FINALS LINES HERE` marker.

The baked ICRA assets are regression fixtures. A finals entry requires
separately generated and driven assets for the released track plus a new image
built against the finals api tag; a Porto-only image must not be presented as
finals-ready.
