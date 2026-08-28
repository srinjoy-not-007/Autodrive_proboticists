# Runbook — running the stack on Apple Silicon macOS

Every command needed to start, stop, reset and tear down a run on an M-series
Mac. Verified end to end on 2026-08-25 (M4, macOS 26.5, Docker Desktop 29.6.2):
6 laps driven, LiDAR streaming, Follow-the-Gap lapping the Porto track.

**This is not the setup `scripts/` assumes.** Those scripts run the simulator
*inside Docker* with `--gpus all` and an X11 socket, and click the Connection
button with `xdotool` from inside the container. None of that exists on macOS.
Here the simulator is the **native macOS app** and only the devkit is
containerised. Where a repo script still works, it is named below; where it does
not, the working command is given instead.

| Piece | Where it runs | Why |
|---|---|---|
| AutoDRIVE Simulator | **natively on macOS** | universal binary, real Metal GPU access |
| Devkit + our nodes | Docker, `linux/amd64` | no arm64 image exists; needs no GPU |
| Link between them | TCP `127.0.0.1:4567` | devkit is the **server**, sim connects to it |

---

## 0. One-time setup

### Simulator

Standalone builds are GitHub release assets on
`AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing`, tag `2026-iros`.
There is a macOS zip per phase (`explore`, `practice`; `compete` is published
with the finals track).

```bash
mkdir -p ~/projects/autodrive_sim && cd ~/projects/autodrive_sim
curl -L -C - --max-time 900 -o sim_mac.zip \
  "https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-iros/autodrive_simulator_practice_macos.zip"
unzip -q sim_mac.zip
```

The download stalls often — `-C -` resumes rather than restarting.

If macOS refuses to open it:

```bash
chmod -R +x "AutoDRIVE Simulator.app/Contents/MacOS"
xattr -dr com.apple.quarantine "AutoDRIVE Simulator.app"
```

Then allow it once under System Settings → Privacy & Security. (Downloading with
`curl` sets no quarantine flag, so this is usually unnecessary.)

### Devkit image

```bash
docker pull --platform linux/amd64 \
  autodriveecosystem/autodrive_roboracer_api:2026-iros-practice
```

4.31 GB. The `--platform` flag is required — the image is amd64-only and runs
under Docker Desktop's x86 emulation.

---

## 1. Start

### 1a. Simulator

```bash
cd ~/projects/autodrive_sim
open "AutoDRIVE Simulator.app" --args -ip 127.0.0.1 -port 4567
```

Confirm it is on the GPU rather than a software renderer:

```bash
grep -E "Metal|engine version" \
  ~/Library/Logs/"AutoDRIVE Ecosystem"/"AutoDRIVE Simulator"/Player.log
```

Expect `Metal device caps: Apple M4` and `engine version: 2022.3.52f1` — the
same Unity version `tools/compare_sim_builds.py` verified on both Docker builds.

### 1b. Devkit container

Started by hand, **not** `scripts/run_devkit.sh` (that one passes `--gpus all`
and mounts `/tmp/.X11-unix`, neither of which exists here). `-p 4567:4567`
replaces `--network=host`, which is unreliable on Docker Desktop.

```bash
REPO=~/projects/Autodrive_proboticists
docker rm -f autodrive_roboracer_api 2>/dev/null
docker run --name autodrive_roboracer_api -d --platform linux/amd64 \
  -p 4567:4567 \
  -v "$REPO/src:/home/autodrive_devkit/src/pit_crew" \
  -v "$REPO/data:/home/data" \
  autodriveecosystem/autodrive_roboracer_api:2026-iros-practice \
  -c "sleep infinity"
```

`sleep infinity` keeps the container alive so nodes can be `exec`'d in and
inspected. Launching the bridge as the container's main process instead means a
crash takes the container (and its logs) with it.

### 1c. Bridge

```bash
docker exec -d autodrive_roboracer_api /bin/bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /home/autodrive_devkit/install/setup.bash && \
   ros2 launch autodrive_roboracer bringup_headless.launch.py > /tmp/bridge.log 2>&1"

# confirm it is listening
docker exec autodrive_roboracer_api netstat -tln | grep 4567
```

### 1d. Connect

**Click `Connection` in the simulator window** — left sidebar; the hamburger at
top-left toggles it if collapsed. There is no way to automate this on macOS.

Verify data is flowing:

```bash
docker exec autodrive_roboracer_api /bin/bash -c \
  "source /opt/ros/humble/setup.bash && \
   timeout 10 ros2 topic echo --once /autodrive/roboracer_1/lidar --field ranges | head -c 200"
```

### 1e. Build our package

```bash
docker exec autodrive_roboracer_api /bin/bash -c \
  "source /opt/ros/humble/setup.bash && cd /home/autodrive_devkit && \
   colcon build --packages-select pit_crew --symlink-install"
```

~8 s. `src/` is bind-mounted with `--symlink-install`, so host edits take effect
on node restart — no rebuild needed for Python-only changes.

---

## 2. Drive

Set once per shell:

```bash
R="source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"
```

### Follow-the-Gap — map-free, works on any track

```bash
docker exec -d autodrive_roboracer_api /bin/bash -c \
  "$R && ros2 run pit_crew follow_the_gap --ros-args \
   --params-file /home/autodrive_devkit/src/pit_crew/config/ftg.yaml > /tmp/ftg.log 2>&1"
```

`ftg.yaml` is v_max 1.5 m/s; `ftg_mapping.yaml` is the slower 1.0 m/s recon
profile. The 33.43 s / 11-clean-lap figure in `docs/STATUS.md` was **not** set
with `ftg.yaml` — do not compare lap times across profiles.

### Shipping stack — localization + MPC on the safe line

```bash
docker exec -d autodrive_roboracer_api /bin/bash -c \
  "$R && ros2 run pit_crew localization --ros-args \
   -p map_prefix:=/home/data/maps/track_v2 \
   -p centerline_csv:=/home/data/racelines/track_v2_centerline.csv \
   -p init_x:=<X> -p init_y:=<Y> > /tmp/loc.log 2>&1"
sleep 7
docker exec -d autodrive_roboracer_api /bin/bash -c \
  "$R && ros2 run pit_crew mpc --ros-args \
   -p raceline_csv:=/home/data/racelines/track_v2_safe.csv \
   -p speed_scale:=1.0 > /tmp/mpc.log 2>&1"
```

`init_x` / `init_y` come from the map's trajectory file — the filter needs a
starting guess:

```bash
head -2 data/maps/track_v2_traj.csv | tail -1 | cut -d, -f2,3
```

Requires `scipy` in the container (`pip3 install scipy`); Follow-the-Gap does
not.

### Logger (development only)

```bash
docker exec -d autodrive_roboracer_api /bin/bash -c \
  "$R && python3 /home/autodrive_devkit/src/pit_crew/tools/race_logger.py > /tmp/logger.log 2>&1"
```

Writes `data/logs/race_<timestamp>.csv`. Uses runtime-restricted topics (IPS,
lap and collision counts) — fine for evaluation, **never** in the submission.

---

## 3. Stop

`scripts/stop_run.sh` works unmodified — it only ever talks to the container.

```bash
./scripts/stop_run.sh
```

It kills our nodes, waits 2 s, then publishes zeros to both actuators.

> **Known defect.** Commands **latch** in this simulator, and the script zeros
> the actuators *after* killing the nodes. The car keeps driving on its last
> command through that gap. Observed 2026-08-25: the collision counter went from
> 1 to 12 during shutdown, with no logger running to attribute them. Zero the
> actuators *first* if the number matters:
>
> ```bash
> docker exec autodrive_roboracer_api /bin/bash -c \
>   "source /opt/ros/humble/setup.bash && \
>    timeout 2 ros2 topic pub -r 20 /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 '{data: 0.0}'"
> ./scripts/stop_run.sh
> ```

Stop one node without touching the rest — note the bracket, otherwise `pkill`
matches the `docker exec` wrapper's own command line and kills the caller:

```bash
docker exec autodrive_roboracer_api pkill -f '[f]ollow_the_gap'
```

---

## 4. Reset

### Reset the car to the last checkpoint

```bash
docker exec autodrive_roboracer_api /bin/bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool '{data: true}'"
```

`std_msgs/Bool`. **This does not free a wedged car** — for that, restart the
simulator (§5). Restart localization after a reset, or re-init its pose: the
filter recovers by resampling along the centerline, but only once it notices.

### Reset the lap and collision counters

There is no topic for this. `lap_count` and `collision_count` are **cumulative
and only reset when the simulator process restarts** — not the devkit, not our
nodes. Either restart the sim, or difference against the first row of the run:

```bash
docker exec autodrive_roboracer_api /bin/bash -c \
  "source /opt/ros/humble/setup.bash && \
   timeout 8 ros2 topic echo --once /autodrive/roboracer_1/collision_count"
```

Misreading this has produced wrong conclusions twice in this project's history.

### Reset the ROS daemon

After any container restart, stale discovery data makes `ros2 topic list` lie:

```bash
docker exec autodrive_roboracer_api /bin/bash -c \
  "source /opt/ros/humble/setup.bash && ros2 daemon stop; ros2 daemon start"
```

---

## 5. Teardown

```bash
./scripts/stop_run.sh
docker rm -f autodrive_roboracer_api
pkill -f "AutoDRIVE Simulator"
```

Restarting the simulator is also the only way to clear a wedged car and to zero
the cumulative counters.

---

## 6. Checks and gotchas

### Confirm the sensor rate

```bash
docker exec autodrive_roboracer_api /bin/bash -c \
  "source /opt/ros/humble/setup.bash && \
   timeout 15 ros2 topic hz /autodrive/roboracer_1/lidar --window 30"
```

Expect **8–10 Hz**, matching the ~10 Hz documented in `CLAUDE.md`. Measured
8.2–9.5 Hz on 2026-08-25 with a single clean connection.

> **If you see roughly double that, you have two connections open.** An earlier
> measurement here read 21.6 Hz; the cause was the simulator connected twice
> (the bridge log showed `Connected!` once per connection, and `netstat` showed
> two ESTABLISHED sockets on 4567), so every sensor message was published twice.
> Check with `docker exec autodrive_roboracer_api netstat -tn | grep 4567` —
> there should be exactly one. Restart the bridge to clear a stale socket.

### Do not run the simulator headless

`-batchmode -nographics` keeps publishing a **stale** LiDAR scan forever — the
LiDAR is GPU-raytraced and needs real rendering. This applies to the native
build too. Always run it windowed.

### Repo scripts that do NOT work here

| Script | Why |
|---|---|
| `sim_restart_connect.sh` | launches the sim in Docker, clicks Connection via `xdotool`/X11 |
| `run_sim.sh` | Docker sim, `--gpus all` |
| `run_devkit.sh` | `--gpus all`, mounts `/tmp/.X11-unix` |
| `smoke_test.sh` | calls `run_sim.sh headless` (frozen LiDAR) and expects 40 Hz |
| `new_track_drill.sh` | calls `sim_restart_connect.sh` in stage 1 |

`stop_run.sh`, `build_map.sh` and `shell_devkit.sh` work unmodified — they only
ever `docker exec` into the devkit container.

### Host toolchain for the offline tools

`tools/` runs on the host, not in the container, and needs a `.venv`. Use
**python3.12**: `trajectory_planning_helpers` pins `quadprog==0.1.7`, which
ships source-only with no arm64 wheel and must compile (Xcode CLT required).

```bash
/opt/homebrew/bin/python3.12 -m venv .venv
.venv/bin/pip install numpy scipy scikit-image matplotlib pyyaml pillow \
                     trajectory_planning_helpers
```

If `quadprog==0.1.7` fails to build, install the package without dependencies
and add a modern `quadprog` (which has a cp312 arm64 wheel).
