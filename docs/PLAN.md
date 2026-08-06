# Winning Plan — RoboRacer Sim Racing League @ IROS 2026

## 1. What the race actually rewards

- **Qualification (Porto, known track, due Sep 12):** 10 clean laps. Speed irrelevant. → optimize for *zero risk*.
- **Finals (unseen track, released Sep 16, due Sep 19):** total time over 10 laps + 10 s per collision, >10 collisions = DQ. Tiebreak = best lap.
- The "unseen" track is public for **3 days** before submission → whoever re-tunes fastest on the new track wins. The deliverable is therefore not a controller, it is a **track-agnostic pipeline** that turns any new track into a tuned, near-optimal racing stack within hours.
- A single collision (+10 s) wipes out ~0.5–1 s/lap of aggression across 10 laps. **Consistency beats peak pace.** Target 0 collisions, always.
- Collision auto-resets the car to the last checkpoint → the stack must recover from arbitrary teleports (localization re-initialization), otherwise one touch cascades into a DQ.

## 2. SOTA survey (what wins F1TENTH/RoboRacer races)

| Approach | Examples | Verdict for us |
|---|---|---|
| **Reactive** (Follow-the-Gap / disparity extender) | UNC 2019 winner; standard baseline | Robust, mapless, mid-pack pace. Use as **week-1 baseline, safety fallback, and data-collection driver** |
| **Map + raceline + tracking** (SLAM → min-curvature raceline → PF localization → Pure Pursuit/MAP) | [ForzaETH race stack](https://github.com/ForzaETH/race_stack) (JFR 2024, physical-race winners); [TUMFTM raceline optimizer](https://github.com/TUMFTM/global_racetrajectory_optimization) | **The winning formula** in time-attack. Highest ceiling, and explicitly permitted by the rules |
| **MPC** (kinematic / MPCC) | Unifying F1TENTH survey ([arXiv:2402.18558](https://arxiv.org/abs/2402.18558)) | Higher tuning cost than PP for marginal gain at 1:10 scale; optional upgrade, not the core bet |
| **End-to-end learning** (TinyLidarNet, SAC/PPO RL) | 3rd place, 12th F1TENTH GP; RoboRacer survey ([arXiv:2506.15899](https://arxiv.org/abs/2506.15899)) | No sim2real gap here (eval in same sim), but AutoDRIVE isn't a fast vectorized training env and benchmarks show planning stacks beat e2e RL. **Not the primary bet**; optional residual-tuning experiment |

Sources: [RoboRacer survey](https://arxiv.org/abs/2506.15899) · [F1TENTH benchmark survey](https://arxiv.org/abs/2402.18558) · [ForzaETH stack](https://github.com/ForzaETH/race_stack) · [ForzaETH particle filter](https://github.com/ForzaETH/particle_filter) · [TUMFTM optimizer](https://github.com/TUMFTM/global_racetrajectory_optimization) · [Foxglove league write-up](https://foxglove.dev/blog/autodrive-roboracer-sim-racing-league)

## 3. Architecture (ForzaETH-style, adapted to AutoDRIVE)

**Offline (once per track, must run in < 1 h):**
1. Drive track safely with reactive controller (or teleop), record LiDAR + encoders + IMU.
2. Wheel+IMU odometry (EKF) → **slam_toolbox** → occupancy grid map.
3. Extract centerline + track widths → **TUMFTM min-curvature raceline** + friction-limited velocity profile (calibrated a_max/braking from sim experiments).
4. Speed-scale sweep (95/100/105 % …) in batch headless sim runs → pick fastest config with zero collisions over 12+ laps.

**Online (runs in the submission container):**
- **State estimation:** encoder+IMU EKF odometry → **particle-filter localization** (MCL, ray-marching on GPU/CPU) against the offline map. Teleport detection → global re-init (handles post-collision resets).
- **Control:** Pure Pursuit with speed-adaptive lookahead on the optimized raceline + feedforward velocity profile (curvature-aware braking). Upgrade path: ForzaETH MAP controller / kinematic MPC if time allows.
- **Safety layer:** LiDAR time-to-collision monitor → emergency slow/steer-away; localization-confidence watchdog → drop to reactive Follow-the-Gap until PF converges. This layer is what guarantees qualification and caps worst-case finals damage.

All of it lives in our own ROS 2 package(s) in `src/` (mounted as `pit_crew` in the devkit workspace), consuming only permitted topics (LiDAR, IMU, encoders, camera, steering/throttle feedback).

## 4. Phase strategy

- **Qualification submission:** raceline stack at conservative speed (or even reactive stack) — whatever has 0 collisions over 30+ consecutive test laps. Submit ~Sep 8, days early.
- **Sep 16–19 sprint (pre-scripted, rehearsed on Porto):** Day 1 = map + raceline + first runs; Day 2 = overnight speed sweeps, tuning; Day 3 = 3× full 12-lap validation runs at final config, submit. Rehearse this exact drill on the practice track in week 4 so the sprint is mechanical.

## 5. Timeline (today = Aug 6)

| Week | Goal | Exit criterion |
|---|---|---|
| **Now** | **Register** (closes Sep 6), join Slack | Confirmation received |
| Aug 6–13 | Infra + baseline: teleop, data recorder, Follow-the-Gap driver; calibrate dynamics (throttle→speed, braking, steering rate) | Reactive stack completes 10 clean laps on Porto |
| Aug 13–20 | Mapping + localization: EKF odom, slam_toolbox map of Porto, particle filter running at 40 Hz | PF pose stable at speed, survives manual resets |
| Aug 20–27 | Raceline + tracking: TUMFTM optimization, velocity profile, Pure Pursuit; batch headless eval harness | Beats reactive lap time by >20 % with 0 collisions |
| Aug 27–Sep 3 | Robustness + speed: safety layer, teleport recovery, parameter sweeps; containerize (`autodrive_devkit.sh` autostart) | 30 consecutive clean laps; container runs from cold start untouched |
| Sep 3–12 | Qualification: freeze, DockerHub push, submit early, buffer for issues | Phase 1 submitted ✔ |
| Sep 12–16 | Sharpen finals pace (MPC/MAP controller experiment optional); rehearse new-track drill end-to-end on Porto | Drill < 1 day from track → tuned stack |
| Sep 16–19 | Finals sprint on released track | Phase 2 submitted with validated config |

## 6. Risks & mitigations

- **Localization drift at high speed** → safety fallback to reactive; conservative speed profile margins near walls.
- **Sim real-time performance on laptop** (GUI + PF + our nodes): profile early; headless sim for batch runs; RTX 5050/16 GB RAM is comfortably above the 2 GB VRAM minimum.
- **Unknown checkpoint locations** (auto-reset spawns) → global relocalization, not assumed start pose.
- **Container drift** ("works on my machine") → test the *submission container* weekly from cold start, not just the dev setup.
- **Sim quirks vs f1tenth_gym** (AWD, 428 Nm, 22.9 m/s top speed — braking dominates) → calibrate all dynamics constants from in-sim experiments, never from literature.
