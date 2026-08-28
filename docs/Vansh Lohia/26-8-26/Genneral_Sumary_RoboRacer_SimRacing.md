# RoboRacer Sim Racing League @ IROS 2026 — Essential Details for Participating Teams

Note: Rules/Timelines are subject to change.

---

## 1. Key Dates (Anywhere-on-Earth deadlines)

| Date | Milestone |
|---|---|
| **Aug 5, 2026** | Registration opens |
| **Sep 6, 2026** | Registration closes |
| **Sep 12, 2026** | Phase 1 (Qualification) submission window closes |
| Sep 13, 2026 | Qualification evaluated |
| Sep 14, 2026 | Qualification results posted |
| Sep 16, 2026 | Final competition track released |
| **Sep 19, 2026** | Phase 2 (Final Race) submission window closes |
| Sep 20, 2026 | Final race evaluated |
| Sep 21, 2026 | Final competition results posted |
| Sep 27–Oct 1, 2026 | IROS 2026 (Pittsburgh, PA) — in-person conference/race |

## 2. Registration

- **Registration form:** https://forms.gle/MiuoSMYQ4z6QACLS6
- Open to anyone worldwide; solo or multi-person teams allowed; a person may belong to **only one** team; multiple teams from the same org are fine.
- Registered teams are listed publicly on the event page (name, members, organization, country) — table updates within ~7 days of registering. Contact organizers if your team doesn't appear after 7 days.

## 3. Key Links

| Resource | Link |
|---|---|
| Main event page | https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/ |
| **Competition Rules** | https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-rules-2026 |
| **Technical Guide** | https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026 |
| Registration form | https://forms.gle/MiuoSMYQ4z6QACLS6 |
| Phase 1 submission form | https://forms.gle/vD23wGS8JkmAZXt79 |
| Phase 2 submission form | (link not yet live on the page as of this writing) |
| Slack workspace (preferred support channel) | https://join.slack.com/t/autodrive-ecosystem/shared_invite/zt-2oeg2hce8-0JvasvnBM1M_wUdDTWRuKw |
| GitHub repo / Issues / Discussions | https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing |
| Docker Hub — Simulator images | https://hub.docker.com/r/autodriveecosystem/autodrive_roboracer_sim |
| Docker Hub — Devkit/API images | https://hub.docker.com/r/autodriveecosystem/autodrive_roboracer_api |
| Local simulator/devkit downloads (Linux/Win/macOS, `explore`/`practice`/`compete` tags) | Release page: `AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing` GitHub releases, tag `2026-iros` |
| Contact | Chinmay Samak (csamak@clemson.edu) / Tanmay Samak (tsamak@clemson.edu) |

## 4. Competition Format

- **Two stages:** Qualification Race → Time-Attack (Final) Race.
- **Qualification:** complete **10 consecutive autonomous laps** on the known practice track. Speed doesn't matter here — but you must not exceed the collision tolerance. Passing = eligibility for the final.
- **Final race (time-attack):** complete 10 consecutive autonomous laps **as fast as possible** on a track you haven't seen before. Ranked ascending by total race time.
- **Every race = 12 laps:** 1 warm-up (untimed, no penalty) + 10 race laps (timed/scored) + 1 cool-down (untimed, optional "show-off" lap).
- Track and vehicle are standardized for every team — **only the algorithm differs.**

## 5. Scoring / Penalties — Critical Numbers

- **Collision definition:** any contact between the vehicle's colliders and the track bounds (wheels touching ground excluded).
- **Collision penalty:** +10 seconds per collision, cumulative (10s after 1st, 20s after 2nd, 30s after 3rd, …).
- **Disqualification threshold:** **more than 10 collisions** in a single race event = disqualified.
- Each collision **resets the vehicle to the last checkpoint** — your localization must be robust to this.
- **Lap timer does not reset** on collision.
- Checkpoint locations are **not disclosed** to teams in advance.
- **Primary ranking metric:** total race time (10 laps). Ties broken using best lap time / other metrics.
- Race referee's decision is final (rebuttals only entertained in extreme cases, with evidence).

## 6. What's Allowed / Not Allowed

**Allowed:** any algorithm approach using raw perception data — reactive planning, map-based localization, raceline optimization, deep learning, RL, hybrid methods, etc.

**Strictly prohibited (instant disqualification):**
- Modifying the simulator executable, vehicle (chassis/powertrain/sensors), environment (track/layout/ground/weather/time), communication protocol (must stay WebSocket), the API (must stay ROS 2), or the containerization tooling (must stay Docker).
- Using **simulation ground-truth data** or controlling anything besides vehicle actuators.
- Exploiting the framework unethically — tapping the backend, frame-grabbing the frontend, exploiting loopholes, tampering with data logging, etc. This is treated as serious malpractice with a public "malpractice" citation.
- Modifying the provided `autodrive_roboracer` ROS 2 package itself — build your own separate package(s) instead.

## 7. Data Streams (ROS 2 Topics) — Access Restrictions

Only these are usable **at race/inference run-time**. Anything marked "Restricted" is for debugging/training only — using it live during racing is against the rules.

| Topic | Message Type | Access |
|---|---|---|
| `/autodrive/roboracer_1/lidar` | `sensor_msgs/msg/LaserScan` | Input (allowed) |
| `/autodrive/roboracer_1/front_camera` | `sensor_msgs/msg/Image` | Input (allowed) |
| `/autodrive/roboracer_1/imu` | `sensor_msgs/msg/Imu` | Input (allowed) |
| `/autodrive/roboracer_1/left_encoder` | `sensor_msgs/msg/JointState` | Input (allowed) |
| `/autodrive/roboracer_1/right_encoder` | `sensor_msgs/msg/JointState` | Input (allowed) |
| `/autodrive/roboracer_1/steering` | `std_msgs/msg/Float32` | Input (allowed) |
| `/autodrive/roboracer_1/throttle` | `std_msgs/msg/Float32` | Input (allowed) |
| `/autodrive/roboracer_1/steering_command` | `std_msgs/msg/Float32` | **Output** (you publish this) |
| `/autodrive/roboracer_1/throttle_command` | `std_msgs/msg/Float32` | **Output** (you publish this) |
| `/autodrive/roboracer_1/ips` | `geometry_msgs/msg/Point` | ⚠ Restricted (debug/train only) |
| `/autodrive/roboracer_1/odom` | `nav_msgs/msg/Odometry` | ⚠ Restricted |
| `/autodrive/reset_command` | `std_msgs/msg/Bool` | ⚠ Restricted (publish `True` then `False` to reset — don't leave it `True`) |
| `/autodrive/roboracer_1/collision_count` | `std_msgs/msg/Int32` | ⚠ Restricted |
| `/autodrive/roboracer_1/lap_count` | `std_msgs/msg/Int32` | ⚠ Restricted |
| `/autodrive/roboracer_1/lap_time`, `best_lap_time`, `last_lap_time` | `std_msgs/msg/Float32` | ⚠ Restricted |
| `/tf` | `tf2_msgs/msg/TFMessage` | ⚠ Restricted |

## 8. Vehicle & Sensor Specs (need-to-know for algorithm tuning)

**Vehicle:**
- Length 0.500 m, width 0.270 m, wheelbase 0.324 m, track width 0.236 m, wheel radius 0.059 m
- Total mass 3.906 kg; Ackermann steering; all-wheel drive
- Steering limits: [-1,1] normalized → ±0.5236 rad (±30°); steering rate 3.2 rad/s
- Throttle limits: [-1,1] normalized
- Top speed: **22.88 m/s**

**LIDAR:** 40 Hz, 0.25° resolution, 1080 points/scan, range 0.06–10.0 m, FOV −135° to +135° (270° total)

**Camera:** 48.83° FOV, 16:9 target image (75% JPG compression)

**Encoders:** 16 pulses/rev, ×120 conversion ratio

**Track ("Porto") environment:**
- ~30×10 m overall footprint
- Border = air ducts, ~33 cm diameter
- Minimum width: 3 car-widths (90 cm)
- Polished-concrete surface (reflective — perception can be tricky)
- May include gaps in ducting that let LIDAR "see through" — plan for this
- Track changes between qualification/final and between competition editions — don't assume it's identical to prior years

## 9. System Requirements

**AutoDRIVE Simulator — Minimum:** Ubuntu 20.04+/Win10+/macOS 10.14+, quad-core CPU, 8 GB RAM, integrated or low-end GPU (2 GB VRAM), 10 GB disk, 1280×720@60Hz, 1 Mbps internet.
**Simulator — Recommended:** Octa-core CPU, 16 GB RAM, GTX 1660/RTX 2060+ (4+ GB VRAM), 20 GB disk, 1920×1080@144Hz, 10 Mbps internet.

**AutoDRIVE Devkit — Minimum:** Ubuntu 20.04/Win10 (VS2019)/macOS 10.14, dual-core CPU, 4 GB RAM, 5 GB disk.
**Devkit — Recommended:** Ubuntu 20.04, quad-core CPU, 8 GB RAM, low-end discrete GPU, 10 GB disk.

Organizers evaluate on: **Intel Core i9-14900K, RTX 4090, 64 GB RAM** — design/test your algorithm with this reference performance level in mind (don't assume your dev machine is comparable).

## 10. Submission Process — Step by Step

1. **Develop** your ROS 2 racing package inside the provided `autodrive_devkit` container, **without modifying** the pre-included `autodrive_roboracer` package (create a separate package instead).
2. **Configure the entrypoint script** (`autodrive_devkit.sh` in the `autodrive_roboracer_api` container) to auto-launch everything needed (sourcing, env vars, nodes) — **do not** rely on `~/.bashrc`. Organizers must be able to open a plain bash session in your container without your code auto-running every time.
3. **Containerize** with Docker, based off the **official base devkit image** (modifications to base container elements are not allowed; you can only add/modify ROS 2 packages).
4. **Commit and push** your image to Docker Hub:
   ```
   docker commit -m "<message>" -a "<username>" <CONTAINER_ID> <username>/<image_name>:<TAG>
   docker login
   docker push <username>/<image_name>:<TAG>
   ```
5. If using a **private** Docker Hub repo, add `autodriveecosystem` as a collaborator.
6. Add clear step-by-step run instructions to your Docker Hub repository overview page.
7. **Submit the link** via the appropriate Google Form:
   - Phase 1 (Qualification): https://forms.gle/vD23wGS8JkmAZXt79 — closes **Sep 12, 2026 AoE**
   - Phase 2 (Final): link posted closer to the date — closes **Sep 19, 2026 AoE**
8. You do **not** need to submit the simulator container — only your devkit/algorithm container. Organizers run the simulator container on their side.

## 11. Container Image Tags

Format: `<YEAR>-<EVENT>-<PHASE>`. For this event: `2026-iros-explore`, `2026-iros-practice`, `2026-iros-compete` (compete tag published closer to race date).

```bash
docker pull autodriveecosystem/autodrive_roboracer_sim:<TAG>
docker pull autodriveecosystem/autodrive_roboracer_api:<TAG>
```

## 12. Running Containers (Reference Commands)

**Simulator container:**
```bash
xhost local:root
docker run --name autodrive_roboracer_sim --rm -it --entrypoint /bin/bash \
  --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_roboracer_sim:<TAG>
```

**Devkit container:**
```bash
xhost local:root
docker run --name autodrive_roboracer_api --rm -it --entrypoint /bin/bash \
  --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_roboracer_api:<TAG>
```

**GUI mode:**
```bash
./AutoDRIVE\ Simulator.x86_64
ros2 launch autodrive_roboracer bringup_graphics.launch.py
```
Then in the simulator's Menu Panel: enter devkit IP (default `127.0.0.1` if same machine) → click **Connection** → toggle **Driving Mode** to Autonomous once connected.

**Headless mode (no rendering):**
```bash
./AutoDRIVE\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567
# or, to keep camera rendering only:
xvfb-run ./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567
ros2 launch autodrive_roboracer bringup_headless.launch.py
```

Distributed setup (simulator and devkit on separate machines) is supported — just point the simulator at the devkit machine's IP; default port 4567.

## 13. Practical Gotchas Called Out by Organizers

- Reset via `/autodrive/reset_command`: publish `True`, then **remember to publish `False` again** — otherwise it keeps resetting in a loop.
- If GPU + Docker gives you trouble, you can run the simulator natively (outside Docker) and only containerize the devkit — since only the devkit container needs to be submitted, this is a fine workaround.
- Avoid Docker Desktop on native Linux — switch to the default context (`docker context use default`) if needed.
- Data recorder in the simulator (hotkey `R`) exports at ~30 Hz to CSV + timestamped JPGs — useful for offline training/analysis, but remember: restricted topics (ground truth, odom, etc.) can be used for **training**, just not at **inference/race time**.

## 14. Support Channels

- **Preferred:** AutoDRIVE Slack (link above) for general questions.
- **Technical/bug issues:** GitHub Issues or Discussions on `AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing`.
- **Private/sensitive matters:** email Chinmay Samak or Tanmay Samak directly.
- Organizers explicitly note they **cannot provide extensive 1:1 technical support** — baseline competence in ROS 2, Docker, Git, and autonomous racing concepts is assumed.

## 15. Citation (if publishing research based on this framework)

```bibtex
@article{AutoDRIVE-Ecosystem-2023,
  author = {Samak, Tanmay and Samak, Chinmay and Kandhasamy, Sivanathan and Krovi, Venkat and Xie, Ming},
  title = {AutoDRIVE: A Comprehensive, Flexible and Integrated Digital Twin Ecosystem for Autonomous Driving Research & Education},
  journal = {Robotics}, volume = {12}, year = {2023}, number = {3}, article-number = {77},
  url = {https://www.mdpi.com/2218-6581/12/3/77}, issn = {2218-6581}, doi = {10.3390/robotics12030077}
}
```
