# Public competitor submissions — RoboRacer Sim Racing League @ IROS 2026

Research snapshot, **2026-09-19** (finals submission day). Built with
`tools/hub_scan.py` (DockerHub), `gh search repos|code` (GitHub), the
organisers' own repos, and `docker create` + `docker cp` on every candidate
image (**no competitor image was ever run**).

**Policy: we learn techniques, we never copy code or racelines.** Every number
below was read out of a file we extracted or off the official results page; the
source is named in each row.

Our reference: **Team Proboticists**, image `bongracers/roboracer`, qualification
6th of 32 at **70.72 s / 0 collisions / 6.97 s best lap**. Finals stack measured
**107.8 s / 10 laps** real display (0.058 s sensor period), **113.5 s** Xvfb,
100 laps / 0 collisions at both.

---

## 1. The leaderboard, and who we actually have to catch

Source: the competition page's own markdown,
`AutoDRIVE-Ecosystem/autodrive-ecosystem.github.io`,
`docs/competitions/roboracer-sim-racing-iros-2026.md` lines 287-320. Race time
is 10 laps of Porto (28.6 m); a collision costs +10 s.
**Phase 2 results are not posted — the page says Sep 21, 2026.**

| # | Team | Race | Coll | Adj | Best lap | Registered lead (Registration.xlsx) |
|---|---|---|---|---|---|---|
| 1 | **NTU DeepSpeed** | 49.83 | 0 | 49.83 | **4.94** | Ting-Ju Chen, Nanyang Technological University |
| 2 | **IDEA_LAB** | 60.99 | 0 | 60.99 | 6.07 | Junseo Jang, Gyeongsang National University |
| 3 | **Penguin** | 64.61 | 0 | 64.61 | 6.45 | "Private", Madagascar |
| 4 | **Barq Racing** | 65.67 | 0 | 65.67 | 6.49 | Abdullah Naeem, Pakistan |
| 5 | **E-Rally** | 67.33 | 0 | 67.33 | 6.67 | Ahmed Mohammed, Helwan University |
| 6 | **Team Proboticists (us)** | 70.72 | 0 | 70.72 | 6.97 | Srinjoy Ganguly, IIT Kharagpur |
| 7 | Δ 2RQ | 72.72 | 0 | 72.72 | 7.13 | Krishn Singh, IIT Madras |
| 8 | bracavisionai | 75.76 | 0 | 75.76 | 7.45 | Luis Bracamontes, Mexico |
| 9 | FirstName LastName | 81.76 | 0 | 81.76 | 8.12 | Sandip Das, India |
| 10 | Bhuna Khichuri Racers | 83.14 | 0 | 83.14 | 8.28 | Al Mahir Ahmed, BRAC University |
| 11 | CEM Navigators | 74.28 | 1 | 84.28 | 7.32 | Shyam Rithin S. K., SRM IST |
| 12 | Assiut Motorsport | 84.65 | 0 | 84.65 | 8.56 | Ahmed Ashraf Hashem, Assiut University |
| 13 | Go-Go-Taro | 85.81 | 0 | 85.81 | 8.49 | Soya Aoki, Japan |
| 14 | Autonomous Ground Vehicle | 77.82 | 1 | 87.82 | 7.53 | Jinansh Dalal, IIT Kharagpur |
| 15 | MonacoF1 | 88.79 | 0 | 88.79 | 8.86 | Emiliano Alban, Ekumen |
| 16 | Team Abhiyaan | 92.56 | 0 | 92.56 | 9.10 | IIT Madras |
| 17 | APEX | 77.68 | 2 | 97.68 | 7.15 | Dongbeen Jeon, Inje University |
| 18 | Penn RoboRacer | 99.99 | 0 | 99.99 | 9.98 | Keshav Mohta, University of Pennsylvania |
| 19 | Cedric Hollande | 70.35 | 3 | 100.35 | 6.80 | xLAB, University of Pennsylvania |
| 20 | Light Year | 105.02 | 0 | 105.02 | 10.41 | Aahan Kumbham, Panther Creek High School |
| 21 | Sagol | 101.41 | 1 | 111.41 | 9.24 | JoonCheol Park, South Korea |
| 22 | UNISAFE | 103.85 | 1 | 113.85 | 9.03 | Jaechan Shin, UNIST |
| 23 | YallaAE | 116.43 | 0 | 116.43 | 11.52 | Bushra AlShehhi, MBZUAI |
| 24 | rulim | 117.05 | 0 | 117.05 | 11.70 | Mikhail Shulga, MIPT |
| 25 | FEBAuto | 82.26 | 4 | 122.26 | 7.91 | Pranav Bhatt, UC Berkeley |
| 26 | Innomer | 124.42 | 0 | 124.42 | 12.13 | Mann Bhanushali, India |
| 27 | SpaceRacer | 150.30 | 0 | 150.30 | 14.78 | Haorong Yuan, Shenzhen University |
| 28 | Siga Siga Racing Team | 181.55 | 0 | 181.55 | 17.84 | Kirill Smirnov, Cyprus |
| 29 | We ain't slippin | 192.02 | 0 | 192.02 | 19.01 | Niklas Holle, Germany |
| 30 | armoured_alpha | 205.28 | 0 | 205.28 | 20.45 | Akash Kanagala, nxtwave |
| 31 | ASUF1Tenth | 257.88 | 0 | 257.88 | 25.69 | Mohamed Hany, Ain Shams University |
| 32 | F1A | 317.90 | 0 | 317.90 | 31.78 | Shadow Ri, Egypt |

The lead-to-handle mapping comes from
`AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing-Participation/Registration.xlsx`,
sheet `IROS 2026`, 55 registered teams (32 submitted). That spreadsheet is the
key that turns anonymous DockerHub namespaces into leaderboard rows, and is the
single most useful artefact found in this sweep.

**Four of the five teams ahead of us published nothing we could find.** Ranks
1-4 have no DockerHub repo and no GitHub repo discoverable by name, handle,
institution or code search (details in §5). E-Rally are the exception: their
image is not public but their stack is, on GitHub (§3.1). Of everything we
could read, only Cedric Hollande's raw 70.35 s beats our 70.72 s — and it cost
them three collisions and thirteen places.

---

## 2. Public images and repos found

Ranked scan output: `data/research/hub_scan_20260919.json` (raw) and
`docs/research/public_submissions_hub_table.md` (auto-generated table). The sweep
saw **1684 DockerHub repos**, tag-listed **258** of them and ranked **141** above
the score floor. Of the 147 in the first pass that matched a competition signal,
**53 carry a competition-grammar tag, 17 carry an `iros` tag, and 19 were pushed
inside one of the two submission windows** (Sep 5-13 qualification, Sep 15-20
finals). Eighteen images were pulled and extracted for this note; all were
deleted afterwards.

*Scoring caveat: the heuristic is deliberately generous, so `mitbmw/autodrive-backend`
and `norlabulaval/dockerized-norlab-control-deep-rl-f1tenth` rank high on name
and push date while being unrelated. Read the table as a shortlist to eyeball,
not a verdict.*

### Extracted and read (finals-window pushes first)

| Team (rank) | Image / repo | Tag | Pushed | Size | What it is |
|---|---|---|---|---|---|
| CEM Navigators (11) | `shyamrithin95/roboracer-cem-navigators` | `2026-iros-compete-raceline` | **2026-09-19 13:41** | 1.11 GB | MCL (likelihood field) + pure pursuit on a stored line, 50 Hz |
| CEM Navigators (11) | same | `2026-iros-practice-v3` | 2026-09-11 | 1.07 GB | **the 74.28 s qualifier was pure disparity-extender gap following** — no map, no line |
| Penn RoboRacer (18) | `mohtak/autodrive_roboracer_api` | `2026-iros-compete` | 2026-09-18 02:23 | 1.21 GB | nav2 AMCL + pure pursuit (default) or kinematic MPC, anchor-spline raceline |
| Δ 2RQ (7) | `krishnsingh/autodrive_roboracer_api` | `nm1` | 2026-09-12 19:01 | 1.08 GB | pure pursuit at 50 Hz **off the restricted `/ips` topic**, online speed horizon |
| MonacoF1 (15) | `ealban/monaco_f1tenth` | `2026-iros-compete` | 2026-09-19 03:29 | 3.36 GB | digest `sha256:9be6da7a…` — **identical to the copy we already analysed**, nothing new |
| bracavisionai (8) | `bracavisionai/roborace_iros_2026` | `finals` | 2026-09-19 12:56 | 1.29 GB | see §3 |
| FEBAuto (25) | `pranman1/roboracer` | `phase2-learned`, `competition` | 2026-09-17 02:58 | 1.14 GB | see §3; also `qualification-mpc` |
| glpuga | `glpuga/autodrive_roboracer_api` | `2026-iros-compete` | 2026-09-15 15:30 | 1.13 GB | pushed *before* the finals track released Sep 16 |
| APEX (17) | `dongbeen/apex-autodrive` | `iros2026-qualification-v1` | 2026-09-13 | 1.07 GB | description: "sensor-only MPC" |
| Cedric Hollande (19) | `cedrichld/autodrive_roboracer_api` | `2026-iros-qualification` | 2026-09-13 | 4.49 GB | xLAB Penn |
| UNISAFE (22) | `jcshinrml/autonomous-racing` | `20260912` | 2026-09-12 | 1.11 GB | "self-contained ROS 2 racing controller for AutoDRIVE" |
| Innomer (26) | `inn0mer/autodrive_roboracer_api_iros` | `team-innomer-qualifications` | 2026-09-13 | 2.92 GB | |
| Light Year (20) | `aahanyay/roboracer-sim-league-light-year` | `2026-iros-practice` | 2026-09-13 | 1.07 GB | |
| SpaceRacer (27) | `haorongyuen/spaceracer_ws` | `iros2026-phase1-v1` | 2026-09-07 | 1.07 GB | |
| ASUF1Tenth (31) | `belal0066/asuf1tenth_roboracer_api` | `2026-iros-practice` | 2026-09-14 | 1.80 GB | |
| Assiut Motorsport (12) | `mohamedelgohary978/assiut-motorsport-roboracer` | `qualification` | 2026-09-14 | 1.36 GB | + repo `MohamedElgohary918/roboracer-pure-pursuit` |
| F1A (32) | `kem0o0/roboracer-iros-2026` | `qualifiacation-F1A` | 2026-09-12 | 1.07 GB | |
| armoured_alpha (30) | `akashkanagala/autodrive_roboracer_api` | `2026-iros-explore` | 2026-09-14 | 1.07 GB | |
| YallaAE (23) | `engbushra/roboracer-ppo-submission` | `2026` | 2026-09-13 | **7.12 GB** | the only RL entry found |
| — | `vulcan758/rtnr_roboracer` | `2026-iros-practice` | 2026-09-12 | 1.10 GB | + repo `Vulcan758/AutoDriveOptimisticFTG` |

### Public GitHub repos (no image, or image + repo)

`braca51e/f1tenth_bracavisionai`, `prajwalthakur/Apex-Autonomy`,
`vaul-ulaval/autodrive_roboracer_ws`, `uci-f1tenth/rustoracer`,
`uci-f1tenth/race_stack`, `Vulcan758/AutoDriveOptimisticFTG`,
`Innomer/Roboracer-Sim-Challenge-ICRA2026`,
`MohamedElgohary918/roboracer-pure-pursuit`, `HamdanAlhajeri/autodrive-roboracer`,
`Gelrignard/autodrive_rl`, `ASUF1tenth/follow_the_gap`,
`sagolsdv/roboracer-sagol-jengban`, `Pranman1/feb-racing`,
`junhakim308/autodrive-roboracer-navigation`, `JisangYun00/autodrive_roboracer_system`,
`kaz4n/autodrive_devkit1`, `alistair-english/roboracer`, `ut-av/roboracer_ws`.

Earlier editions (same teams recur, useful for technique but not for this
track): `vaul/autodrive_roboracer_api:serge-v2`,
`bracavisionai/autodrive_roboracer_api:2026-icra-*`,
`asierkaballero/autodrive_roboracer_team_ikerlan:icra2026_final`,
`traxon99/2026-icra-roboracer`, `rostamza/autodrive_roboracer_api`,
`keshavmohta/autodrive_roboracer_api:2026-icra-compete`,
`nabilafifahq/roboracer-t7` (11 tags incl. `cartographer-ekf`, `orbslam3-mechazo`).

---

## 3. What they do that we do not

### 3.1 E-Rally (rank 5, 67.33 s, best lap 6.67 s) — the only readable stack that beat us

`github.com/Ashiry2005/F1tenth_opt`, pushed 2026-08-31, ~5000 lines, and
unusually well documented (their `config/*.yaml` are prose essays with the
measurement behind every number). It is a *minimal* pure-pursuit + AMCL package,
so everything below is cheap.

**Their Porto profile, from `maps/practice_map_new/raceline.csv`'s own header:**

```
a_lat 6.20   a_acc 3.60   a_brake 4.885   v_max 6.64
veh_width 0.35 + safety 0.30/side        len 29.682 m   lap_est 6.984 s
```

Against ours (**ay 4.5 / ax 6.5 / brake ~5**) that is the mirror image: they buy
**38 % more lateral** and give back roughly half the longitudinal. Their braking
is *coast only* — `brake_scale` is documented as a dead knob, "ALL deceleration
is coast + rolling drag", achieved p90 **−3.487 m/s²**. So they are clean at
a_lat 6.20 while our own note says ay 5.5 gives 8 collisions. The difference is
not the tyre; it is what follows.

*(Two internal inconsistencies in their own docs, noted so we do not quote them
as gospel: the `brake_scale` comment argues against a planned −3.86 while the
shipped line's header says `a_brake 4.885`, and `e_trim 0.10` is described as
"just above this track's measured xt_mean 0.130" when 0.10 < 0.130. Both read
like prose left behind by a later retune.)*

1. **Wheel-free odometry: IMU heading + LiDAR scan-to-scan ICP, no encoders at
   all** (`odom_source: laser_imu`; `odom_beam_stride 4`, `odom_max_corr 0.5 m`,
   `odom_max_iter 12`, `odom_cv_weight 0.03`, `odom_v_max 8.0`). Their stated
   reason: "no wheel encoders, so tyre slip never enters the estimate". We have
   `scan_odometry` implemented and *not adopted*, and we fought the wheel
   encoder's backward steps for weeks (`MAX_CONSEC_REJECT`, per-wheel filters).
2. **A primed filter and a converge gate.** `initial_prime_scans: 15` runs 15
   in-place sensor updates before the car moves; the controller then pins target
   speed to `creep_speed 1.0 m/s` until smoothed `pos_std` holds under 0.35 m
   for `lock_dwell 0.30 s` (`unlock_dwell 0.80 s`). "Lap 1 starts localized
   rather than guessing." Note the **debounced latch with separate lock and
   unlock dwell** — that is exactly the state machine our `arbiter` note says it
   is missing.
3. **Throttle is feed-forward only, from a MEASURED non-linear table.**
   `u = table(v_target) + k_acc * a_target`, no speed feedback anywhere.
   `DEFAULT_SPEED_TABLE` is 12 measured points: dv/dthr ≈ **25 (m/s)/thr from
   0.05 to 0.35 throttle, falling to ≈8 by 0.60** as drag catches up. Our single
   0.040/m/s constant is 1/25 — correct inside the racing range and wrong above
   ~8.5 m/s. Their `k_acc` is **0.010**, sized from the *residual acceleration
   deficit per plan-ax bin* (median 0.0104), and they explicitly reject `1/g`
   ("0.0315 … made wall contact"). Our `throttle_a_gain` experiment used
   **0.041** and gave 5 collisions — 4× theirs, and sized a different way.
4. **Speed preview as a rigid spatial shift**: `speed_preview_tau 0.20` commands
   `plan(s + v·tau)` rather than `plan(s)`. Sized against the measured lag:
   "v_tgt→v_meas cross-correlates at 0.540 s"; they swept tau against worst-case
   a_lat and shipped 0.20 because 0.40+ leaves proven grip. The longitudinal
   plant they fit is **D 0.140 s + tau 0.760 s** — settling needs 2.28 s, and
   "the longest throttle hold anywhere on the track is 1.64 s, typical 0.12-0.50 s:
   the car is always mid-transient". That is the quantitative version of our own
   "the car is under its own plan, and that is load-bearing".
5. **Lookahead priced by a corner-cut allowance**, not by speed:
   `Ld = clip(sqrt(8·e_allow / kappa_ahead), ld_min 0.6, ld_max 3.6)` with
   `e_allow 0.10 m`, solved by bisection (8 steps, 0.009 m bound) as a fixed
   point so `kappa_ahead` is the max |κ| of the *line* over the next Ld metres.
   Long on straights, short in hairpins, with the geometric error *bounded* by
   construction.
6. **Cross-track trim with a deadband sized from the measured bias**:
   `e_trim 0.10` deadband, `e_ramp 0.10` so the trim is at full authority by
   0.20 m, "where the measured steady bias sits". The principle — size the
   deadband and the ramp from the *measured* steady cross-track error rather
   than guessing a gain — is the transferable part.
7. Control is a **fixed 50 Hz timer, explicitly not scan-driven**, with
   `use_sim_time: false` forced in three places because AutoDRIVE publishes no
   `/clock`. Likelihood-field MCL, only **300 particles**, 180 beams,
   `beta 0.35` (beam-product temperature < 1 "flattens it, keeps diversity"),
   `update_min_d/a 0.05`.
8. They take the LiDAR offset as **`lidar_dx 0.2733`** — the guide's figure,
   which is stated w.r.t. the rear axle. UCI's bridge uses 0.2733 too. Ours is
   0.1933. The guide's frame table also gives `ips x 0.08`, and
   **0.2733 − 0.08 = 0.1933 exactly**, so the most likely reading is that our
   figure is the same mount expressed relative to the IPS/IMU frame our pose
   comes from — i.e. both are right in their own frame. *This is an inference
   from the guide's table, not something we re-derived; confirm before acting,
   but do not "fix" 0.1933 to 0.2733 on the strength of a competitor's config.*

### 3.2 FEBAuto (rank 25, UC Berkeley) — the richest technique haul

`pranman1/roboracer:phase2-learned` (2026-09-17) and `Pranman1/feb-racing`.
They finished 25th (4 collisions), so the *stack* is not proof of speed — but it
contains three mechanisms aimed exactly at our known failure mode, and a sysid
file (`config/sysid_params.json`) that is directly comparable to ours.

1. **Iterative Learning Control baked into the reference line.** Their shipped
   `config/track_raceline.json` carries a `learned_total` array: a per-waypoint
   lateral reference correction, **`learned_laps: 10`, `learned_rounds: 3`,
   range −0.050 to +0.155 m**, learned from a driven log (`learned_from:
   RL_ctl_1.csv`). `map_core.py` cites Kapania & Gerdes (ACC 2015,
   arXiv:1902.00611): `d_{j+1}(k) = d_j(k) − kp·e_j(k) − kd·(e_j(k)−e_j(k−1))`,
   `kp 0.02`, `kd 0.4 rad/m`, clip 0.20. Their own comment is the argument:
   *"the track repeats, so a corner's correction is applied BEFORE the error
   occurs: actuator dead time and the 10 Hz rate do not limit it, and it needs
   no vehicle model — which is why it should succeed where our model-based
   attempts failed."* **Important nuance: the online ILC (`ilc_on`) defaults to
   False and their entrypoint never enables it — the learning happened offline
   over three rounds and the result is baked into the shipped line.** That is
   the cheap, safe version: no new in-loop behaviour, no risk of a runaway
   correction, and it is exactly the shape of `tools/ilc_prior.py` and
   `data/ilc/run_prior_porto_mesh_mc0.63_ay6.75.npz`, which are sitting
   untracked in our repo. A competitor has shipped the idea.
2. **A footprint-rolling, arc-sweeping turn-in guard that EASES rather than
   switches** (`map_core.scan_safe_delta`). Roll the six-corner car footprint
   along the arc the candidate command would produce, over a horizon of
   `max(1.5, 0.45·v)` m, skipping the first `act_delay·v` metres (dead time); if
   the swept footprint touches a LiDAR return, step the command down through
   `[1.0, 0.85, 0.7, 0.55, 0.4, 0.25, 0.1, 0.0]` and take the arc with the most
   clearance. Their diagnosis: *"pure pursuit starts turning toward the lookahead
   point before the car has passed the inner wall end … every hit was the front
   corner on the wall end"*. APEX ship the same idea as
   `lidar_steering_guard_clearance 1.7` / `maximum_blend 0.35`. **Two
   independent teams blend a LiDAR guard into the command; nobody switches
   controllers** — which is the arbiter lesson, stated positively.
3. **Steering-limit relaxation in slow corners.** `map_delta_lim 0.30` rad at
   speed (the slip peak) but `map_delta_lock 0.5236` allowed below
   `map_v_lock 0.8 m/s`, blended to `map_v_peak 2.2`. Their comment is a direct
   challenge to our `ay_limit`: *"at every speed scale we have run, the limit was
   BELOW the geometric demand and fell further as we sped up. That is the
   collision cliff — the car understeers into the hairpin wall because it is not
   allowed to ask for the angle the line needs."* Our own Sep 19 measurement
   (achieved/kinematic curvature 0.96 at full lock) says the understeer that
   motivated `ay_limit` is not there.
4. **Rate-adaptive speed scaling, and why**: `rate_nominal_s 0.11`,
   `rate_scale_min 0.35`, recovery +0.03/tick, with the comment *"under the
   organisers' screen recorder the frame rate drops and the gap between scans"*
   grows to **0.3-0.5 s wall**. We already ship `rate_adaptive`; this is
   independent confirmation of the failure mode and a second set of constants.
5. **Braking hysteresis with a minimum duration**: `brake_on_mps 1.5`,
   `brake_off_mps 0.2`, plus a floor on brake duration, because *"the clean run
   still showed ~9 brake taps of 0.1 s per lap and 5.8 throttle reversals/s"*.
   And while the brake is commanded they **decay the speed estimate at the
   measured brake rate** instead of trusting the encoder.
6. **Start merge**: `map_start_ld_m 1.2` for the first `map_start_dist_m 3.0` m,
   added after *"a 25-lap run collided once at t=0.7 s from the spawn"*.
7. Their sysid, for cross-checking ours: `v_ss_per_throttle 24.95`
   (= 0.0401/m/s — **matches our 0.040 exactly**), `ay_98pct 4.7` (ours 4.5-4.64),
   `L_eff 0.348`, `steer_rad_per_cmd 0.5236`, and a
   **`grip_map`** of achievable curvature vs speed
   (0.95 m/s → κ 1.58; 2.6 → 0.75; 3.5 → 0.50; 4.3 → 0.45) with
   `grip_safety 0.75`. Their shipped line: 44.99 m, `est_lap_s 10.61`, driven
   **11.60-11.80 s**, ten laps clean — slower than our 10.78 s/lap.
8. **Disagreement to note, not to copy:** their `brake_note` claims "throttle 0
   decelerates ~16 m/s² nearly independent of v (15 samples)". CEM Navigators
   measured **4.3 m/s²** over 25 cuts; we measured 4.55 held / 6.4 from 7 m/s.
   Two of three agree with us; 16 m/s² is >1 g and almost certainly a bad probe.
9. They also model steering as a **pure 0.25 s transport delay**
   (`act_delay_s 0.25`, `act_note: "the lag+rate-limit fit (0.10/0.10/1.5) gave
   17 collisions vs 5 on the car"`). We model `steer_tau 0.04` plus a 3.2 rad/s
   rate limit. 0.25 s is 2.5 sensor periods and is a big claim — **check it
   against our own `tools/delay_model_id.py` output before believing it.**

### 3.3 CEM Navigators (rank 11) — independent confirmation of our brake finding

`shyamrithin95/roboracer-cem-navigators`, finals tag pushed 2026-09-19 13:41.

- Their qualifier (74.28 s, 1 collision, best 7.32 s) was **pure reactive
  disparity-extender gap following**, no map and no line — a useful calibration
  of what reactive alone is worth on Porto.
- `raceline_tracker.py`: *"Negative throttle is never emitted: it engages reverse
  on this vehicle rather than braking. Deceleration is by drivetrain drag,
  measured at **4.3 m/s² mean over 25 coastdown cuts**, and the velocity profile
  was generated with 4.1 to leave margin."* **Independent confirmation of our
  Sept 18 brake discovery**, and their planning margin (4.1 against a measured
  4.3) is more conservative than ours (plan ≤5.5 against a 4.55 ceiling).
- `coastdown.py` is a nice instrument: cut throttle only when steering is near
  zero *and* forward clearance is large, abandon the window if a corner arrives,
  discard any cut where steering moved — so the measurement is drag, not tyre
  scrub. Many short cuts instead of one long one, because *"the longest straight
  is about 6 m"*.
- Their tracker caps lookahead at **1.1 m, explicitly below the corridor
  half-width**: *"at the default 3.0 m the aim point can cross the thin barrier
  between the deck and the upper span and the vehicle steers into it."*
- **Finals-track intel** (`particle_filter.py` header): the IROS track has *"two
  near-identical towers and two parallel straights"*, `slam_toolbox` does not
  converge on it ("map→odom swung by metres and tens of degrees per lap across
  three configurations"), and the spawn is **(0.80, 3.16), yaw −1.571**.
  They also record an **organiser ruling on Slack, 2026-09-17: an offline-built
  map is permitted**; FEBAuto record the same ruling dated 2026-09-12.
- Localization fallback: if the nearest waypoint is further than
  `lost_distance_m 1.5`, **throttle goes to 0 and steering holds** — a stop, not
  a recovery. Their comment: *"a path tracker will steer confidently at a line
  that is not where it thinks it is."*
- Their finals line: 43.96 m, v 1.57-6.50, ideal lap **11.64 s** — slower than
  our 10.78 s/lap.

### 3.4 APEX (rank 17) — sampling MPC with a blended LiDAR guard

`dongbeen/apex-autodrive:iros2026-qualification-v1`,
`src/apex_racing_stack/config/autodrive_mpc_submission.yaml`:
`control_rate 20`, `mpc_horizon 20 × mpc_dt 0.05` (1.0 s), `mpc_samples 256`,
`mpc_iterations 3` (CEM), `mpc_max_lateral_accel **9.0**`, `mpc_max_speed 8.0`,
`mpc_wall_margin 0.1` with `wall_weight 200` / `wall_collision_weight 50000`,
`mpc_contour_weight 28`, `friction_weight 50`, `mpc_steer_rate_weight 2.5`,
`mpc_max_steer_rate 3.2`, `mpc_latency_compensation 0.03`,
**`mpc_speed_command_preview 0.2`** (the same 0.2 s speed preview E-Rally sized
independently), `longitudinal_feedforward_gain 0.04` (our number again),
`enable_yaw_stability_governor: true`, and the blended guard
`lidar_steering_guard_clearance 1.7` / `maximum_blend 0.35`.
Localization is a coarse→fine grid scan-matcher at 20 Hz against an `.npz` map
(`matcher_beam_stride 10`, `coarse_step 0.1`, `fine_step 0.025`) from a fixed
initial pose (0.8, 3.1583, −1.5707). Raceline is TUM
`trajectory_planning_helpers` format, 400 points.

### 3.5 Penn RoboRacer (rank 18) and Δ 2RQ (rank 7)

- **Penn** (`mohtak/autodrive_roboracer_api:2026-iros-compete`, 2026-09-18):
  nav2 AMCL (`likelihood_field_prob`, 500-2000 particles, `alpha* 0.05`,
  `update_min_d/a 0.0`) + pure pursuit as the shipped default with kinematic MPC
  as an option. `throttle_gain 0.043`, PI on speed (`kp 0.28, ki 0.05`),
  `max_steering_rad 0.38` (deliberately below the 0.5236 lock),
  `velocity_scale 1.25`, adaptive lookahead 0.65-2.5 m. Raceline is an
  **anchor-point spline** (40 hand-placed anchors) with `alat 4`, `margin 0.21`,
  `vmax 10`; their finals line is 44.76 m, `lap_time_est 11.18 s`. They built a
  web "Playground" with sliders to tune it, which is why the params file reads
  "taken from the live UI overlay".
- **Δ 2RQ** (`krishnsingh/autodrive_roboracer_api:nm1`): 50 Hz timer, pure
  pursuit with `Ld = (0.75 + 0.15 v)/(1 + 1.15 κ_ahead)` clipped 0.46-1.70, an
  online **backward speed pass over a 3.6 m horizon**
  (`v = min_s sqrt(v_lim(s)² + 2·a_dec·s)`, `a_dec 6.2`, `v_grip = sqrt(μg/κ)`
  with μ 0.72), a 1.5× steering gain boost above κ 0.30, and a LiDAR wall-nudge
  term. **They take pose and speed straight off `/autodrive/roboracer_1/ips`**,
  which the 2026 guide lists as Restricted ("should not be used while
  autonomously racing at run-time"). Their raceline peaks at 9.0 m/s with a
  0.45 m clearance column. Their in-image `auto_map_and_optimize.py` would drive
  a 1.4 m/s recon lap, accumulate wall hits, extract the corridor, solve a
  min-curvature QP at 0.22 m margin and write the CSV — the same drill as our
  `new_track_drill.sh`, but run *inside* the submission.

### 3.6 Cedric Hollande / xLAB Penn (rank 19) — **raw race time 70.35 s, faster than ours**

`cedrichld/autodrive_roboracer_api:2026-iros-qualification`, 4.49 GB. Ranked 19th
only because of **3 collisions** (+30 s); the underlying 70.35 s / 6.80 s best lap
is quicker than our 70.72 s / 6.97 s. This is the fastest *readable* car in the
field, and its three collisions are the whole story.

- **JAX MPPI**, `src/autodrive_league/config/params_autodrive.yaml`:
  `state_predictor: dynamic_ST` (yaw rate + slip, not kinematic),
  `n_samples 8192`, `n_steps 8`, `n_iterations 10`, `sim_time_step 0.1`,
  `temperature 0.015`, `control_sample_std_steer 0.8`, `std_accel 10.0`,
  `accel_scale 7.0`, `steer_vel_scale 1.0` (the action is a **steering rate**).
  Rewards: `xy 0.25`, `velocity 0.5`, `yaw 0.01`, `wall_cost_weight 20` at
  `margin 0.25`, `power 0.6`.
- **`control_trigger_mode: odom_gate`** — the best architectural idea in the
  corpus. The *pose callback* rate-gates and dispatches the control step; the
  fixed timer is demoted to a 5 Hz **watchdog** that fires only if pose-driven
  dispatch has not run in `control_watchdog_max_silence_sec 0.10`. Their stated
  reason: *"so the SingleThreadedExecutor cannot starve pose ingestion behind a
  continuously-ready timer."* That is the principled answer to our sensor-period
  lottery — control is pose-driven with a timer as a liveness floor, rather than
  either purely per-scan (us) or purely free-running (everyone else at 40-50 Hz).
- **Per-waypoint friction column in the raceline CSV (col 10)** — MPPI rolls out
  with per-step μ along the reference, falling back to a scalar `friction 0.8`
  only if the column is absent. This is item 1 of §4 in a different clothing: a
  *per-sector* grip belief baked into the line instead of one global cap.
- Localization: **MIT `range_libc` GPU ray-marching particle filter**,
  `max_particles 8000`, `angle_step 18`, `range_method rmgpu`,
  `rangelib_variant 2`, `squash_factor 2.2`, `z_hit 0.75`, `sigma_hit 8.0`,
  motion dispersion 0.05 / 0.025 / 0.25 — the same component VAUL fork. 8000
  particles is 4-27× what anyone else in this field runs, and it is affordable
  only because the ray-marching is on the GPU.
- `speed_profile_lookahead_steps: 3` (target is the **min** speed over the next
  N reference steps) and `speed_profile_iterations: 3` (rebuild the reference
  with the profiled speeds, twice or three times).
- An **MPPI warm-start watchdog**: on a timing jump, a wall-gap or 3 bad
  callbacks, throw away the shifted control sequence instead of carrying a bad
  plan forward (`mppi_guard_*`).
- Their bridge (`adapter_node.py`) independently measures **"throttle 0.08 →
  1.99 m/s steady state … ~0.04 throttle per m/s and VERY torquey"** — our
  number for the fourth time — but then brakes with `brake_throttle: -0.3`,
  i.e. **reverse drive**. Given our own measurement that 0.3 negative throttle
  produced 176 collisions on a correct speed sensor, that is the most plausible
  explanation for their three collisions and the +30 s that cost them 13 places.
  **The single fastest readable car in the field lost purely to the brake bug we
  already fixed.**

### 3.7 bracavisionai (rank 8) — **map-anchored speed zones, shipped today**

`bracavisionai/roborace_iros_2026:finals`, pushed 2026-09-19 12:56. Their stack
lives in `/iros_ws` (built by `RUN`, so `docker history` does not name it — copy
`/iros_ws/src` explicitly). `src/pure_pursuit/config/pure_pursuit_params.yaml`
is the single most directly useful competitor file found:

- **Location-based speed zones.** Circular regions on the map, each with its own
  speed multiplier, **cosine-blended in space** from the global multiplier at the
  outer edge of an annulus to the zone value at the centre; the blend starts at
  `radius × influence_scale` (1.15), overlapping zones resolved by *smallest*
  nominal radius. They ship four on the finals track: centres
  (−2.0, −13.0) r 5.45 ×0.67, (4.75, −9.0) r 3.5 ×0.98, (4.32, −0.2) r 2.25 ×0.9,
  (2.34, 4.0) r 1.75 ×0.86, on top of a global `speed_multiplier 0.8`. Their own
  comment: *"Slow down in crash-prone areas (or speed up on safe straights)."*
  **This is item 1 of §4, already in a shipping finals image**, and the *spatial*
  cosine blend is better than Apex's hard per-sector scalar because it cannot
  step the target speed discontinuously.
- **A deliberate warm-up and cool-down lap.** `warmup_laps: 1` at
  `warmup_speed_multiplier 0.16`, a 0.1 s ramp into race mode, then
  `cooldown_start_lap: 16` ramping back down over 1.0 s. Note the comment that
  their lap counter runs **~2 ahead of the simulator display**, so the index is
  offset by +2 — a counting hazard worth knowing about.
- Curvature pre-brake: scan `curvature_lookahead_points 15` ahead,
  `curvature_decel_gain 0.2`, floored at `min_curvature_speed_factor 0.6` of the
  profile speed.
- ForzaETH-derived stability set, cited as such: `max_steering_change 0.3` per
  control step, `speed_lookahead_time 0.2 s`, steering downscaled from 1.0 at
  5.0 m/s to 0.2 at 8.0 m/s, lateral-error-adaptive lookahead, adaptive
  `L = clamp(0.6 + 0.4·v, 0.6, 3.2)`. `control_rate 60 Hz`. `steering_gain 0.99`.
- They use `wheel_base: 0.3302`, not the devkit's 0.324 — a small unexplained
  disagreement with our `vehicle.py`.
- They also ship an **LLM agent loop inside the submission image**
  (`src/agent_optimizer/roboracer_optimizer`: `plan / research / implement /
  fix_build / test / evaluate / strategist / decide / finalize` nodes, with
  `prompts.py`, `llm.py`, and the competition rules and technical guide bundled
  as resources, plus `tools/telemetry.py`, `tools/results.py`, `tools/shell.py`).
  So at least one other team is tuning the same way we are.

**A convergence worth noticing.** Four independent stacks — E-Rally
(`speed_preview_tau 0.20`), APEX (`mpc_speed_command_preview 0.2`), FEBAuto
(`speed_lookahead_s 0.2`) and bracavisionai (`speed_lookahead_time 0.2`) — all
land on **0.2 s of speed preview**, i.e. look up the profile at `s + 0.2·v`.
E-Rally show the sizing: the measured v_tgt→v_meas lag is ~0.54 s and tau must
sit *below* it. Our `thr_brake_lead_s` is **0.05**. That is the cheapest
single number in this whole note to sweep.

### 3.8 Dead ends, recorded so nobody re-checks them

- `ealban/monaco_f1tenth:2026-iros-compete` (2026-09-19 03:29) is digest
  `sha256:9be6da7a…` — **byte-identical to the copy we already analysed.** Ekumen
  pushed nothing new.
- `glpuga/autodrive_roboracer_api:2026-iros-*` is a plain **ROS 2 Jazzy rebuild
  of the devkit** with no team code at all (`exec "$@"` entrypoint) — not an
  entry.
- (bracavisionai moved to §3.7 — it turned out to be the most useful image in
  the sweep once `/iros_ws` was copied. Note their launch passes
  `tf_compliance_mode:=true`, which reads like a rules-compliance switch.)
- `engbushra/roboracer-ppo-submission:2026` (YallaAE, 7.12 GB) is the only RL
  entry found and finished 23rd at 116.43 s.
- `nabilafifahq/roboracer-t7` (+ `derekh0803`, `cabbagelover`) has eleven tags
  including `cartographer-ekf` and `orbslam3-mechazo` — a physical-car team.
- Worthless, checked so nobody re-checks: `Gelrignard/autodrive_rl` (one commit,
  `ppo_test.py` is 0 bytes, `--eval` is dead code, lap-count reward is bugged
  absolute-not-delta, no evidence a lap was ever finished);
  `ASUF1tenth/follow_the_gap` (README is the unedited F1TENTH "Lab 4" assignment,
  `SUBMISSION.md` still says `[FILL ME IN]`); `JisangYun00/autodrive_roboracer_system`
  (stock devkit + an empty submodule); `sagolsdv/roboracer-sagol-jengban` (CAD only);
  `ut-av/roboracer_ws` (UT Austin physical car, unrelated);
  `Pranman1/feb-racing` (Berkeley's own hosting harness, not an entry);
  `HamdanAlhajeri/SDP` (105-line tutorial gap-follower);
  `kaz4n/autodrive_devkit1` (a real 500-sample MPPI at 100 Hz, but its README
  performance table is explicitly labelled "(Expected)" — zero measured results).

### 3.9 GitHub-only stacks worth naming

- **bracavisionai (rank 8)** — `braca51e/f1tenth_bracavisionai`. The "learned path
  tracker" is **not** end-to-end: `f1tenthNet/model_lidar_goal_speed.py` is a 1-D
  CNN over 1081 raw beams with two heads, `goal = (x, y)` lookahead point in the
  `front_axle` frame and `speed`; a hand-written Stanley law then tracks that
  point. It is trained by **imitating their own map-based AMCL + TUM-raceline
  stack** (`dataset_lidar_goal_speed.py` columns `frame, tf_goal_x, tf_goal_y,
  speed, lidar`), i.e. the net replaces *localization + raceline + profile*, not
  the controller, and races map-free. Net runs on the scan callback (~10 Hz), the
  tracker at `control_dt 0.03` = **33 Hz**, latching the last goal. Steering law
  carries an **IMU yaw-rate damping term (`kd_yaw −0.059`) and a derivative of
  the sim's own `/steering` feedback (`kd_steer 0.01`)** — an actuator-lead term.
  Speed is estimated from the applied-throttle topic, `v = 23.04·thr_fb + 0.2037`,
  and throttle is `1.1 × (0.04338·v_target − 0.00867)` — a deliberate **10 %
  over-command** on the open-loop map. Never publishes 0.0 and never negative:
  **no braking at all.** Underneath: AMCL (500-2000 particles, beam-skip) over a
  **60 Hz blended kinematic/dynamic single-track odometry**, fused in a 50 Hz EKF.
- **Apex Autonomy** — `prajwalthakur/Apex-Autonomy`, a real ForzaETH `race_stack`
  port (ICRA 2026, 12th/79). `controller_frequency: 50.0`, regulated pure pursuit
  with **`ld = 1.2·v − 60·κ + 2.5·lat_err`** clipped to [0.7, 2.4] m — note the
  *negative curvature term*, so lookahead shortens in corners — and
  `max_steering_angle 0.40` rad, below the lock. Speed is the TUM global profile
  times **per-sector hand scalars** (`speed_scaling.yaml`: global 0.9, four
  sectors at 0.8 / 0.6 / 0.9 / 0.5 by waypoint index) and then times
  `exp(−|lat_err|·norm_curv)`. Fallback: below 0.1 m/s for 3.0 s → follow-the-gap.
  Caveat: their `ggv.csv` is a flat 12/12 m/s² and `racecar.ini` still holds TUM's
  full-size-car defaults (v_max 70, 1200 kg, 3.4 m wide) — unmodified upstream,
  not tuning.
- **UCI** — `uci-f1tenth/rustoracer` (RL, 40 Hz) and `uci-f1tenth/race_stack`
  (`warporacer_node.py`, **60 Hz**, MLP as pure numpy in preallocated buffers, no
  torch, BLAS pinned to one thread). Both **command a steering RATE and integrate
  it themselves at the sim's own 3.2 rad/s limit, re-seeding the integrator from
  the sim's `/steering` feedback every tick**, so command and actual never
  diverge. `warporacer` also clamps `v_cmd ≤ v_meas + 2.0` (anti-windup) and on a
  0.2 s stale scan zeroes both integrators and publishes speed 0.0 / steer 0.0 —
  which through the bridge is exactly-0.0 throttle, i.e. a real brake. Their RL
  reward is `Δwaypoint/n·100·(1 + v/10) − 0.1·exp(−3·EDT(x,y)) − 100 on collision`
  with per-episode domain randomization of lf, lr, h, mass, I_z, a_max, r_w and
  the Pacejka coefficients. Their bridge takes `/odom` from `/ips` — restricted.
- **VAUL (Université Laval)** — 41 repos read; **the winning stack is not public.**
  `autodrive_roboracer_ws` ships only a textbook PID wall-follower at a constant
  2.2 m/s. What their *forks* reveal is the toolbox: a **GPU ray-marching particle
  filter** (MIT `rangelibc`, `range_method: rmgpu`, 2000 particles, `angle_step 18`,
  `squash_factor 2.2`, `sigma_hit 8.0`) instead of AMCL — which is how they afford
  2000 particles — plus ForzaETH **On-Track-SysID** (Pacejka fit) and TUM
  `global_racetrajectory_optimization` (whose `racecar.ini`/`ggv.csv` are
  unmodified upstream defaults, so no tuning is exposed).
- **Innomer (rank 26 here; 3rd in ICRA 2026 qualification, best lap 7.84 s)** —
  `Innomer/Roboracer-Sim-Challenge-ICRA2026`. Map-free: a 41-candidate arc rollout
  blended with pure pursuit, run **entirely in the LiDAR callback** (the only
  competitive repo that does not outrun the sensor), dead-reckoning
  `yaw = 0.60·yaw_enc + 0.40·yaw_imu`. Speed heuristic
  `0.50·progress + 0.50·clearance`, then `×(1 − 0.45·(|steer|/0.5236)^1.5)`.
  Their `racer_tuner.py` is an **automated exhaustive hyper-parameter search that
  drives the live sim and uses `/autodrive/reset_command` between trials**, with
  RandomForest importance ranking — unique in the corpus, and a good model for
  our Xvfb harness (not for the display gate).
- **Assiut Motorsport (rank 12)** — `MohamedElgohary918/roboracer-pure-pursuit`:
  40 Hz timer, `ay_max 4.3`, `v_max 9.5`, `ld 0.55-1.40 m`, speed PID
  `kp 0.13 / kd 0.008` + `kff_v 0.054`, `min_throttle −0.25` (they believe
  negative throttle brakes). **No localization at all — it subscribes straight to
  the restricted `/autodrive/roboracer_1/odom`.**
- **HamdanAlhajeri/autodrive-roboracer** — the most honest repo in the set, and
  **independently found our brake**: any decel request ≥ 0.1 m/s² returns
  throttle *exactly* 0.0, with a unit test named
  `test_braking_never_reverses_and_overspeed_cuts_throttle`. They also **zero the
  PI integral on brake entry** and let `target_speed` track *measured* speed down
  while braking, so throttle neither fights the brake nor lurches on release.
  But `braking_calibrated: false` and they are capped at 2.5 m/s — they have the
  mechanism and not the number.
- `alistair-english/roboracer` publishes exactly `0.0f` when its best arc collides
  at the first sample — a **third independent sighting of the brake**. Its
  `docs/racing_architecture.md` proposes remapping the bridge's `/tf` → `/sim/tf`
  and publishing their own under the standard names; read that as rules-gaming,
  unverified, and stay away.
- `traxon99/tln_variants` (TinyLidarNet): TFLite 1-D CNN scan → (steer, speed) in
  the scan callback, ~100-500 µs, injects LiDAR noise in sim for sim-to-real.
  Corroborates that bracavisionai's architecture is a known family.

---

## 4. Ranked: what to try, with estimated gain

Our reference: **107.8 s / 10 laps real display, 113.5 s Xvfb, 0 collisions.**
Estimates are per 10-lap race and are **mine, not measured** — house rule 1
applies, so nothing here is a number until it comes off a run.

**Timing note:** the finals image is frozen at Gate D today (Sep 19). Nothing
below is a finals change. Read this as the backlog for the next edition, and as
a check on which of our standing "do not re-derive" notes other people disagree
with.

| # | Technique | Source | Mechanism for us | Est. gain | Confidence |
|---|---|---|---|---|---|
| 1 | **Map-anchored speed zones on top of the global profile** | bracavisionai's *shipping finals image* (4 circular zones, cosine-blended in space, ×0.67…×0.98 over a global ×0.8); Apex `speed_scaling.yaml` (global 0.9; sectors 0.8/0.6/0.9/0.5); Cedric's per-waypoint friction column | Our only speed knob is a global `ay` cap, so the one corner that collides sets the speed of the whole lap — exactly the dead end in our own notes ("one corner produces every collision"; "raising ay does not work"). An s-indexed (or map-anchored) multiplier lets us slow s≈19 alone and raise everything else. Use the **spatial cosine blend**, not a hard sector boundary, so the target speed never steps. Applies offline in `make_track_lines.py`; no controller change. | **2-5 s** | med-high |
| 2 | **Per-waypoint learned lateral correction of the reference line (ILC)** | FEBAuto `track_raceline.json: learned_total` (10 laps, 3 rounds, −0.050…+0.155 m); Kapania & Gerdes kp 0.02 kd 0.4 | Shift the *reference*, not the command, by the measured steady tracking error at each index. The correction lands before the error, so neither the 0.25 s actuator lag nor the 10 Hz rate limits it, and it needs no vehicle model. Attacks our 0.31 m (0.59 m worst) cross-track through the one corner. **We already have `tools/ilc_prior.py` and a prior `.npz` untracked — finish it.** | **1.5-4 s** | med-high |
| 3 | **Flat over-command scalar (1.05-1.15×) on the open-loop throttle map** | bracavisionai `1.1 × (0.04338·v − 0.00867)` | We sit below plan on 62 % of samples by 0.96 m/s where ax > 3, and both closed-loop fixes we tried were worse (`throttle_kp` 0.08 → 75 collisions; `throttle_a_gain` 0.041 → 5). A constant multiplier is neither: still open loop, no new bandwidth, pays the deficit down on straights. One-parameter sweep, safe to abort. Compare with E-Rally's `k_acc` **0.010** — 4× smaller than the 0.041 we tried, and sized from a *residual-deficit-per-ax-bin* regression rather than 1/g. | **1-3 s** | med |
| 4 | **Blended, footprint-aware LiDAR turn-in guard (ease, never switch)** | FEBAuto `map_core.scan_safe_delta`; APEX `lidar_steering_guard_*`; both blend | Roll the car footprint along the arc the command would produce, skip the first `act_delay·v` m, and if it sweeps a return, step the command down through a fixed ladder. **This is the arbiter lesson stated positively**: two independent teams blend, nobody switches. Our arbiter "flips state almost every scan"; a monotone blend cannot. | **0.5-2 s**, plus real collision insurance | med |
| 5 | **Wheel-free pose: LiDAR ICP + IMU odometry instead of encoders** | E-Rally `odom_source: laser_imu` (stride 4, max_corr 0.5, 12 iters, cv prior 0.03) | Removes tyre slip and the encoder's 6.7 % backward steps from the estimate entirely — the thing that forced our calibrated-bounds + `MAX_CONSEC_REJECT` latch and that mis-tuned `brake_gain` once already. We have `scan_odometry` implemented and unadopted. Cheaper first step: blend the applied-`/throttle` topic (`v ≈ 23-25·thr`) in as a speed prior, which is **validatable entirely offline against IPS on existing logs, zero sim time.** | 0-1 s directly; large DQ-risk reduction | high on robustness |
| 6 | **Primed filter + converge gate with separate lock/unlock dwell** | E-Rally `initial_prime_scans 15`, creep 1.0 m/s until `pos_std < 0.35 m` for `lock_dwell 0.30 s`, `unlock_dwell 0.80 s` | Buys lap 1 outright and gives us the dwell-time state machine our arbiter note asks for. Cheap. | 0.3-1 s on lap 1 | med |
| 7 | **Raise the lateral budget once pose is better** | E-Rally `a_lat 6.20 / a_acc 3.60 / a_brake 4.885`, clean at rank 5 | They run 38 % more lateral than us and *less* longitudinal, and they are clean — but only on top of items 5 and 6. Do not raise `ay` before the pose is fixed; our own notes are emphatic that raising it alone does not work. | 2-4 s, gated | low until 5/6 land |
| 8 | **Pose-gated control dispatch with the timer demoted to a watchdog** | Cedric/xLAB `control_trigger_mode: odom_gate`, `control_watchdog_hz 5.0`, `max_silence 0.10 s` | The pose callback dispatches the control step; the fixed timer only fires if no pose-driven dispatch has happened in 100 ms. Reason given: a free-running timer starves pose ingestion in a single-threaded executor. A cleaner answer to our sensor-period lottery than either pure per-scan (us) or a free 50 Hz timer (everyone else), and it composes with `rate_adaptive`. | 0-1 s, mostly variance | med-low |
| 9 | **Look the speed profile up at `s + 0.2·v`** | E-Rally `speed_preview_tau 0.20`, APEX `mpc_speed_command_preview 0.2`, FEBAuto `speed_lookahead_s 0.2`, bracavisionai `speed_lookahead_time 0.2` — **four independent stacks, same number** | Command now what the line will want 0.2 s from now, so a car that responds τ late arrives at `s` doing what the generator asked. E-Rally's sizing rule: τ must sit *below* the measured v_tgt→v_meas lag (~0.54 s for them). Our `thr_brake_lead_s` is **0.05**. One number, one sweep — the cheapest item in this note. | **1-2 s** | med |

**Three things our own notes disagree with, worth a re-test rather than a
rewrite.** (i) FEBAuto relax the steering limit in slow corners and call our
`ay_limit` "the collision cliff"; our Sep 19 grip-curve measurement (achieved /
kinematic curvature 0.96 at full lock) says the understeer that motivated
`ay_limit` is not there, so the two findings are now in tension. (ii) FEBAuto
model steering as a **pure 0.25 s transport delay** and report 5 collisions
against 17 for a lag + rate-limit fit; we ship `steer_tau 0.04` plus a 3.2 rad/s
rate limit. (iii) E-Rally run `a_lat 6.20` clean where our note says 5.5 gives
8 collisions. In all three cases the honest reading is that *their* pose is
better than ours, not that our measurement was wrong — which is why items 5 and
6 gate item 7.

**Two cautions to record.** (a) Several competitors' gains assume a pose we do not
have: Δ 2RQ, Assiut, `junhakim308` and UCI's bridge all read `/ips` or `/odom`,
which the 2026 guide lists as Restricted — **do not import their gains directly.**
(b) Our brake discovery is *not* unique — Hamdan, alistair and CEM all publish
exactly 0.0 — but **nobody else has calibrated it** (Hamdan ships
`braking_calibrated: false` at a 2.5 m/s ceiling; CEM plan 4.1 against a measured
4.3). The measured 4.55 m/s² ceiling is still our edge.

---

## 5. What we could not get

- **Ranks 1-4 (NTU DeepSpeed, IDEA_LAB, Penguin, Barq Racing) — no submission
  image, no code, no writeup.** Probed exhaustively: 15-35 candidate DockerHub
  namespaces each (via `hub.docker.com/v2/repositories/<ns>/` and
  `/v2/users/<ns>/`, where a 404 is a clean existence test), plus `gh search
  users|repos|code`, institution pages and LinkedIn. Their images are almost
  certainly private — DockerHub lets you push a private repo and grant the
  organisers pull access, and the rules never required the image be public.
  What *was* confirmed: `github.com/NTUDeepSpeed` is real (site
  `ntudeepspeed.github.io`, all nine rostered members match) but its repos are
  physical-car F1TENTH stacks with no AutoDRIVE code, and its own achievements
  page still lists IROS 2026 as "Upcoming"; IDEA_LAB is IDEA Lab @ Gyeongsang
  National University (Prof. Gun-Woo Kim), a recurring entrant since IROS 2024,
  with a labmate forking `autodrive_roboracer_ws` and an MPC/overtaking
  framework — circumstantial evidence the lab's direction is MPC; Penguin
  registered as "Private" from Madagascar and has essentially zero public
  footprint (all ~79 GitHub users with location=Madagascar checked). **The only
  public trace of NTU DeepSpeed's 4.94 s Porto lap is the leaderboard row.**
- **The per-team YouTube links** on the results page are present but empty
  (`[:material-youtube: YouTube]()`), so no onboard video for the top runs yet.
  Worth re-checking after Sep 21 — a DeepSpeed onboard would be the single most
  informative artefact available.
- **Phase 2 results** (due Sep 21) and **VAUL's race stack** (their public repos
  are the devkit, a wall follower and a particle filter only).
- **web.archive.org** is blocked from this environment and has no snapshot of
  the competition page in the qualification window anyway.
- E-Rally (rank 5) is the exception: their
  submission image was not found, but `github.com/Ashiry2005/F1tenth_opt`
  (2026-08-31) is a packaged build of the same stack, twelve days before the
  deadline, and is read in §3.1.
- The reactive-only entries were skimmed and are what their rank suggests:
  Light Year (20) runs `ros2 run ftg_racer follow_gap`; SpaceRacer (27) runs
  `reactive_racing_autodrive`; F1A (32), ASUF1Tenth (31) and armoured_alpha (30)
  likewise. Nothing there beyond §3.3's disparity-extender numbers.

---

## 6. Reproducing this

```bash
# DockerHub sweep (unauthenticated, rate-limited, ~5 min with the narrow filters)
nice -n 19 taskset -c 28-31 env OMP_NUM_THREADS=1 .venv/bin/python \
    tools/hub_scan.py --out data/research --md docs/research/public_submissions.md

# re-rank a previous sweep without touching the network
.venv/bin/python tools/hub_scan.py --reuse data/research/hub_scan_20260919.json

# the organisers' own key: team -> lead -> institution, per edition.
# Sheets: IROS 2024, CDC 2024, ICRA 2025, CDC-TF 2025, ICRA 2026, IROS 2026.
curl -sLO "https://raw.githubusercontent.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing-Participation/main/Registration.xlsx"

# results + rules as markdown rather than rendered HTML. The restricted-topic
# table is at ~line 433 of the guide.
curl -sL "https://raw.githubusercontent.com/AutoDRIVE-Ecosystem/autodrive-ecosystem.github.io/main/docs/competitions/roboracer-sim-racing-iros-2026.md"
curl -sL "https://raw.githubusercontent.com/AutoDRIVE-Ecosystem/autodrive-ecosystem.github.io/main/docs/competitions/roboracer-sim-racing-guide-2026.md"

# GitHub: code search finds entries that DockerHub search misses
gh search code "roboracer_1/throttle_command" --limit 60 --json repository,path
gh search repos "autodrive_roboracer" --limit 40 --json fullName,pushedAt
```

**Extracting a competitor image without ever running it**: `docker pull`, then
`docker history --no-trunc` — every `COPY`/`ADD` destination is in the layer
commands, which is how you learn their layout — then `docker create` +
`docker cp` of `/home/autodrive_devkit`, `/home/autodrive_devkit.sh` and whatever
extra roots the history named (`/iros_ws`, `/opt/racing_ws`, `/ws`), then
`docker rmi`. `docker create` never starts the container. All images pulled for
this sweep were removed afterwards except
`ealban/monaco_f1tenth:2026-iros-compete`, which was already on the host and
turned out to be unchanged.
