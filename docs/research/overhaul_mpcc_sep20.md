# An architectural alternative to mpc3 — three candidates, offline (Sep 20)

**Nothing here has been driven.** Every number is from the log-fitted surrogate
(`tools/surrogate_race.py`, `--plant-fit`), on the shipping line
`data/racelines/finals_mesh_apex_f42k_race.csv` with the shipping mpc3 flags
(`submission/autodrive_devkit.sh`, `barrier_margin 0.34`), at the two sensor
periods that matter: **0.058 s** (a real display) and **0.098 s** (the
organisers' box). Read it with the standing rule from `docs/FINALS_TUNING.md`
§4c: **a surrogate collision is evidence; a surrogate clean sheet is not**, and
five harness-predicted lap-time gains failed to transfer live on Sep 19.

Branch `overhaul-mpcc`. Nothing in the image changes; the shipping defaults are
verified byte-identical (`cmp` on `--dump-rec`, mpc3 at 0.058 s, 4 laps).

---

## 0. The verdict in four lines

| candidate | 0.058 s vs mpc3 | 0.098 s vs mpc3 | verdict |
|---|---|---|---|
| **A. `mpcc` — joint speed+steering MPC** | *(table 3)* | *(table 3)* | **the only one worth a live slot** |
| B. `ffb` — preview feedforward + feedback | *(table 4)* | *(table 4)* | **rejected offline**: slower at the organisers' period in every arm |
| C. `fastpub` — plan re-read between poses | *(table 5)* | *(table 5)* | **null, and the mechanism says it must be**: no topic is faster than the pose and the bridge latches one command per tick |

---

## 1. What was built

Three control laws, each a new module under `src/pit_crew/`, each wired into
`tools/sim_tracker.py` behind `--tracker <name>` and into `scripts/run_exp.sh`
behind `CONTROLLER=<name>` the same way mpc3 is. `src/pit_crew/mpc3.py` and
`tools/sim_tracker.py` each gained three IDENTITY hooks (`extra_solver_args`,
`post_throttle`, `post_steer`, plus a `MPC_CLASS` / `EXTRA_PARAMS` pair) so the
candidates reuse mpc3's entire ROS plumbing — callbacks, rate adaptivity,
watchdog, stale-pose ride-out, teleport recovery, stuck escape, publish guard —
rather than forking it. `tests/test_overhaul_candidates.py` pins that the hooks
are the identity for mpc3 and that each new node constructs and publishes
finite in-range commands against the rclpy stub.

### A. `mpcc_core.MPCC` — speed as a state, throttle/brake as an input

`mpc3` optimises the steering against a speed it does not decide: the ceiling
is the fixed offline profile, cut by the `e_free`/`k_e` heuristic, and the
brake is then chosen by two thresholds inside `ServoThrottle`
(`brake_a_min`, `brake_dv`) on a lead in seconds (`thr_brake_lead_s`) that
`rate_adaptive` scales with the sensor period.

`MPCC` puts the speed in the same Gauss-Newton problem. Decision variables are
the steering rate **and** the longitudinal acceleration at every free step;
state is `(e_lat, e_yaw, delta, v)`. Four things follow, and they are the
candidate:

1. **The corridor gets a gradient with respect to speed.** In `MPC3._rollout`
   the arc step `ds` is frozen by the outer loop, so changing the speed plan
   inside a solve cannot change the predicted lateral error at all — the
   barrier literally cannot ask the car to slow down, which is why the
   `e_free`/`k_e` cut has to exist. In `MPCC._roll` the arc step is
   `0.5 (v_k + v_{k+1}) dt` with `v` a decision variable, and the reference
   curvature and both corridor walls are looked up **at the arc length the plan
   actually reaches**, linearly interpolated (a nearest-cell lookup has no
   usable finite-difference gradient: a 0.05 m/s² perturbation moves the arc by
   less than one 0.05 m cell). Removing this was measured: with the frozen
   grid, `mpcc` takes 4-6 collisions per 24 laps at s 24 (the tightest corner
   of the lap) at 0.098 s; with it, zero.
2. **The measured envelope, as a soft constraint.** Per the refit
   (`tools/sysid_breakaway.py`), breakaway on this tyre is a **yaw-rate**
   limit, not a constant-ay circle: attached to `r = v|kappa| = 2.63 rad/s`,
   steady-state break from 2.79, and a second ceiling `ay = v²|kappa| ≈ 10.5`
   that only binds above ~5 m/s. Both enter as one-sided quadratic hinges
   (`w_break`, `w_ay`), **not** as a switch — the staircase says a 0.19-0.29 s
   excursion past 2.71 still delivers 0.95 of what is asked, and the surrogate
   agent measured that a hard threshold in the *plant* made the harness worse.
   Where each binds is the sanity check pinned in the tests: a hairpin
   (1.6 1/m) is capped by the yaw rate (1.70 m/s), a fast sweep (0.3 1/m) by
   the lateral acceleration (5.92 m/s).
3. **The real brake, as a discrete option.** The longitudinal input is bounded
   below by the fitted throttle-0.0 law `4.15 + 0.273 v` m/s² (4.99 at v 2-3 up
   to 6.4 from 7 m/s) rather than by the profile's 4.6, and the **solver** —
   not a threshold on a lead — decides the exact 0.0: the brake is published
   when the optimised plan's own acceleration at the instant the command is in
   force is below `-mpcc_brake_a`. `ServoThrottle`'s `brake_mode` is switched
   off in this node.
4. **The fixed profile is kept as reference and terminal set.** The horizon is
   1.2 s; the profile carries the geometry it cannot see — the hand-held safe
   zones at s 17.5-21.5, 36-40 and 42-2.1 that every ay-9 rung was disqualified
   for removing. `v_prof_cap` is therefore **1.00**: MPCC may brake later and
   modulate, it may not invent speed the profile refused. This is not a
   conservative default chosen for taste — at `v_prof_cap 1.25` it laps 0.6 s
   quicker in the surrogate and collides at s 36.5-36.8, exactly the corner the
   live record calls marginal.

Solver: Gauss-Newton, Levenberg damping on the Hessian diagonal (mpc3's
scheme), with a **vectorised finite-difference Jacobian** — the whole
perturbation batch (1 + 2·n_free ≈ 35 columns) is rolled out in one pass over
the horizon, so one Jacobian is ~20 numpy calls on length-35 arrays. Pure
numpy, because the competition image has **numpy 1.22.2 and no scipy and no
CasADi** (checked: `docker run --rm autodriveecosystem/autodrive_roboracer_api:2026-iros-compete`).

Everything after the solve — the readout lead, `w_dcmd`, `cmd_tau`, the publish
gate, the straight-mode pure-pursuit blend, the parameter schedule — is
`MPC3`'s, inherited and unchanged, so an A/B isolates the core.

### B. `ffb_core.FFBSteer` — preview feedforward + low-gain feedback

`delta = atan(L·kappa(preview)) + k·delta_pp(projected error)`. The preview
distance is solved for in two sweeps against the rack's own 3.2 rad/s rate
limit (`d = v·(dead + |delta_want − delta_now|/rate)`), which is what "sized to
the rate limit" means; the feedback is the pure-pursuit form the shipping
`straight_pp_k` blend already uses on straights, with the lateral error
projected over `t0 + tk·period`. The MPC still runs and still owns the speed,
the throttle and the corridor; it is the **safety envelope** — `envelope_trip`
rolls the candidate angle forward on the bicycle model and, if holding it takes
the car out of the corridor inside the horizon (or `|e_lat|` exceeds
`ffb_env_e`), the solver's command is published for that frame instead.

### C. `planreplay.PlanReplay` — re-read the plan between poses

Two facts bound this before any code, and both were checked rather than
assumed:

* **No topic arrives faster than the pose.** On both shipping 100-lap logs
  (`race_20260919_044701`, `_050630`) the IPS row period and the distinct
  `/pit_crew/pose` stamp period are the same number to four decimals —
  0.0580 / 0.0940 s. Every sensor shares one telemetry WebSocket. An inner loop
  has nothing new to integrate; the IMU does not arrive at 100 Hz.
* **The bridge latches one command per telemetry tick**, so extra publishes are
  discarded; only the last one before the tick is applied
  (`MPC3Sim.pickup_time`).

That is also why `control_mode:=timer` lost (`docs/STATUS.md`: +0.25 s/lap and
4-6 collisions in 33 laps at *every* period): timer mode re-**solves** at 20 Hz
from a pose up to a whole period old, with `rate_adaptive` off and
`apply_ahead_s` re-pointed at the tick — three changes at once, two of which
make the solver's information strictly worse. The fixable part is the third:
the solve stays on the pose, at maximum freshness, and only the **readout**
walks forward at the wall clock, so the number latched at the tick is the
plan's angle for that tick rather than one read up to a period earlier. The
replayed angle is clipped to what the rack could have slewed since the solve,
so the publish gate's and the straight blend's effect on the solved command is
not thrown away.

---

## 2. Method

```bash
taskset -c 22-27 nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  MKL_NUM_THREADS=1 .venv/bin/python tools/surrogate_race.py \
  --tracker mpcc --line finals_mesh_apex_f42k_race --frame 0.098 \
  --laps 100 --seeds 2 \
  -r barrier_margin=0.34 -r straight_pp_k=0.5 -r straight_d=3.0 \
  -r straight_kappa=0.10 -r straight_pp_period_max=0.075 -r mpcc_brake_a=2.5
```

`tools/surrogate_race.py` gained `--tracker`; everything else is unchanged, so
the plant, the sensor chain, the seeds and the shipping flag set are exactly
the ones §4c validated. `mpc3` is the baseline in the same command with
`--tracker mpc3`.

---

## 3. Candidate A — the numbers

### 3.1 Screening, 24 laps per cell

Baseline `mpc3` on the same line and flags: **11.070 s** at 0.058 and
**11.560 s** at 0.098.

| arm | 0.058 laps/coll, median | 0.098 laps/coll, median |
|---|---|---|
| `mpcc` defaults, `mpcc_brake_a 2.5` | 24 / 0, 11.090 | 24 / 0, **11.200** |
| `mpcc_brake_a 1.5` | 24 / 0, 11.140 | 24 / 0, 11.370 |
| `lon_brake_plan 5.6` | 24 / 0, **11.000** | 24 / 0, 11.210 |
| `lon_brake_plan 4.6` (= the profile's brake) | 24 / 0, 11.100 | 24 / 0, 11.280 |
| `mpcc_iters 1` (cheaper solve) | 24 / 0, 11.080 | 24 / 0, 11.330 |
| `mpcc_servo_reserve 0.5` | 24 / 0, 10.970 | 24 / 0, 11.660 |

Two things to read out of that. **The gain is not only the bigger brake
number** — at `lon_brake_plan 4.6`, i.e. planning on exactly the deceleration
the offline profile plans on, `mpcc` is still 0.28 s/lap quicker at the
organisers' period; the rest is the model-based brake TIMING and the
speed-aware barrier. And **the servo reserve is not a free knob**: it is the
one place where the joint plan has to be handed to a speed servo that eases off
near its setpoint, and getting it wrong costs more than the architecture wins.

### 3.2 Where the time comes from (0.098 s, 24 laps, per 2 m bin)

| zone | mpcc − mpc3 |
|---|---|
| straight s 0-16 | **−0.135 s** |
| first hairpin s 16-22 | **−0.140 s** |
| s 24-27 (the tightest corner) | −0.100 s |
| s 37 corner (s 32-41) | −0.105 s |
| last hairpin s 41-46 | **+0.085 s** |
| **lap** | **−0.395 s** |

The single biggest bin is s 16.0 (−0.080 s), which is the braking zone into the
first hairpin — the corner the Sep 20 braking audit measured the car entering
+0.25 m/s hot at the organisers' period, and the one that costs the shipping
stack 0.2 s/lap there. The last hairpin goes the other way, which is the
expected sign: the live record says that corner is capped by the steering lock,
so a controller that plans more entry speed there pays for it.

### 3.3 Gates, 100 laps per seed

PLACEHOLDER_GATE

### 3.4 The cost: solve time

Same host, same pinned cores, 6 laps at 0.098 s:

| | median | p95 | max |
|---|---|---|---|
| `mpc3` | 5.83 ms | 6.41 ms | 7.04 ms |
| `mpcc` | **13.04 ms** | **18.20 ms** | 23.37 ms |

**This is the one real shipping risk in candidate A.** It is inside the 20 ms
budget on this host, but the live solve is not this host's: `docs/FINALS_TUNING`
measures mpc3's live solve at 10-40 ms, i.e. up to 6x the harness figure, and
mpc3 already counts `solve over period` in its periodic log line. At 2.2x mpc3
the worst live case would drop poses at the display period. Mitigations, in
order: `mpcc_iters:=1` (13.04 -> ~9 ms, costs 0.13 s/lap at 0.098 and nothing
at 0.058), `mpcc_outer:=1`, `horizon:=16`. **Read the node's `solve NN.N ms`
line in the first 20 s of the first live run before trusting any lap time from
it.**

(A rollout micro-optimisation — one corridor lookup per step instead of two,
and skipping the saturation exponential when nothing is over the knee — took
the median 16.63 -> 13.04 ms and reproduces the same 6-lap run to every printed
digit, so the screening numbers above stand.)

## 4. Candidate B — the numbers

Baseline `mpc3`: **11.070 s** at 0.058, **11.560 s** at 0.098, 24 laps per cell.

| arm | 0.058 laps/coll, median | 0.098 laps/coll, median |
|---|---|---|
| defaults (`ffb_dead_s 0.175`, `k 1.0`, `Ld 0.55 v`) | 24 / 0, 11.270 | — |
| `ffb_dead_s 0.13` | 24 / 0, **10.970** | 24 / 0, 12.660 |
| `ffb_dead_s 0.22` | 24 / 0, 11.570 | 24 / 0, 12.630 |
| `ffb_k 1.4` | 24 / 0, 11.590 | 24 / **13**, 13.000 (all at s 0.9-1.4) |
| `ffb_ld_k 0.8` | 24 / 0, 11.080 | 24 / 0, 11.720 |
| `ffb_cmd_tau 0` | 24 / 0, 11.360 | 24 / 0, 12.250 |
| `k 1.4, Ld 0.8 v, dead 0.13` | 24 / 0, **10.850** | 24 / 0, 12.220 |

Gate, the best arm (`k 1.4, Ld 0.8 v, dead 0.13`), 100 laps per period:

| period | laps / collisions | median | vs mpc3 |
|---|---|---|---|
| 0.058 | **100 / 0** | **10.880** | **−0.135 s/lap** |
| 0.098 | **100 / 0** | 12.180 | **+0.635 s/lap** |

**Rejected offline.** The hypothesis has half a leg to stand on: at the DISPLAY
period the best arm is 0.22 s/lap quicker than mpc3, which is consistent with
the straight-line swivel being a re-solve artefact (and with the live
`straight_pp_k 0.5` result, which is this law applied to straights only and is
already shipping). At the ORGANISERS' period every arm is 0.16 to 1.10 s/lap
SLOWER, and the firmest one takes 13 collisions in 24 laps on the exit of the
last hairpin. The reason is visible in the law: its anticipation is a preview
distance in metres computed from a dead time in seconds, so at a 69 % longer
period it looks at the wrong place, while the MPC's anticipation is `nd`
horizon steps of committed steering and moves with the period for free. A
per-period preview schedule is the obvious next arm, and it is exactly the
family (`lead_mode:=time`, `lead_tk`, the throttle-lead pin) that the live
record closed in both directions on Sep 19. Not worth a live slot.

## 5. Candidate C — the numbers

| arm | 0.058 laps/coll, median | 0.098 laps/coll, median |
|---|---|---|
| `replay_gain 1`, 100 Hz | 24 / 0, **11.050** | 24 / 0, 11.570 |
| `replay_gain 1`, 200 Hz | 24 / 0, 11.020 | 24 / 0, 11.570 |
| `replay_gain 1`, no rate cap | 24 / 0, 11.060 | 24 / 0, 11.560 |

Gate (`replay_gain 1`, 100 Hz), 100 laps per period:

| period | laps / collisions | median | vs mpc3 |
|---|---|---|---|
| 0.058 | **100 / 0** | 11.000 | −0.015 s/lap |
| 0.098 | **100 / 0** | 11.570 | +0.025 s/lap |

**Null, and the mechanism says it had to be.** The whole prize is at most one
sensor period of readout freshness, because the bridge latches one command per
tick and nothing arrives between ticks; `cmd_tau 0.10` then absorbs most of
even that, since the replayed angle is filtered with the same time constant.
What the exercise IS worth is the two measurements at the top of section 1C:
**no topic on this stack is faster than the pose** (checked in both shipping
100-lap logs, to four decimal places) and the bridge is one-command-per-tick,
so "run the controller faster" is closed for a reason that is not a tuning
accident. `control_mode:=timer`'s loss is now explained rather than merely
recorded: it was not the rate, it was re-solving from a stale pose with
`rate_adaptive` off.

---

## 6. What to do with this

**Candidate A is the one worth a live slot, and its claim is specifically about
the ORGANISERS' period.** It is neutral on a real display and ~0.35-0.40 s/lap
quicker at 0.098 s, which is the right shape: image v7 was promoted on exactly
that trade (−0.13 s/lap at the organisers' period, +0.05 on the display).

Standing caveats, in the order they should be believed:

1. **A surrogate clean sheet is not evidence.** Five harness-predicted gains
   failed to transfer live on Sep 19 and the surrogate's own verdict table is
   8/15 with seven false cleans. What IS evidence here is the collisions it
   found while the candidate was being built — at s 36.5-36.8 when
   `v_prof_cap` was 1.25, at s 24 when the arc grid was frozen, at s 21 when
   the servo reserve was two-sided — every one of them a corner the live record
   already names. The shipped defaults are the configuration that survived
   those.
2. **The pace claim is the part the surrogate is good at** (per-2 m segment
   times reproduce the live log to a median of 0.02 s), and it is a 0.4 s/lap
   claim, not a 0.05 s/lap one — i.e. outside the ±0.06 s/lap accuracy §4c
   measured, and outside the one disagreement it has on this geometry
   (0.19 s/lap on `apex_f42k`). It would still not be the first 0.4 s/lap
   surrogate claim to evaporate.
3. **Solve time (3.4) is the thing most likely to turn this into a DQ**, and it
   is measurable in the first 20 s of the first run.
4. This is a controller change, so it owes **Gate C (100 Xvfb laps, in band,
   0 collisions) and Gate D (32 display laps, 0 collisions)** in its own right,
   and it must not go near the image until both pass.

### The live run, in order

Gate C first, in band (`RATE_MIN_DT`/`RATE_MAX_DT`, the Sep 20 method rule), on
a quiet host:

```bash
LOC_ARGS='-p max_step_mps:=9.0 -p publish_on_reloc:=false' \
RATE_MIN_DT=0.094 RATE_MAX_DT=0.100 \
SIM_QUALITY=ultra CONTROLLER=mpcc MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=100 \
  ./scripts/run_exp.sh mpcc_x100 apex_f42k_race \
  -p map_prefix:=/home/data/maps/finals_mesh \
  -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p steer_tau:=0.04 \
  -p barrier_margin:=0.34 -p thr_brake_lead_s:=0.05 \
  -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6 \
  -p pub_gate_corr:=0.05 \
  -p servo_k_p:=0.5 -p servo_head_max:=0.3 -p servo_head_kappa:=0.4 \
  -p straight_pp_k:=0.5 -p straight_d:=3.0 -p straight_kappa:=0.10 \
  -p straight_pp_period_max:=0.075 \
  -p mpcc_brake_a:=2.5
```

Then Gate D, the same flags with `VISIBLE=1 ... LAPS=32 ./scripts/run_exp.sh
mpcc_disp apex_f42k_race ...`.

`brake_mode:=zero` is passed for one reason: the node logs the override
(`ServoThrottle brake_mode zero -> none`), so a run that lost the flag is
visible in the first second of the log. In `mpcc` the brake is the SOLVER's.

**Abort rules.** Any collision at either period; the node's periodic
`solve NN.N ms` exceeding the measured period (switch to `mpcc_iters:=1` and
restart — do not read that run); driven ay p99 above 9.3; a rising cte p95.

**If Gate C passes and the solve time is tight**, the cheaper rungs in order
are `mpcc_iters:=1`, then `mpcc_outer:=1`. If Gate C fails at a corner, the
knob that moved it most offline is `lon_brake_plan` (5.2 -> 4.6 gives 0.12
s/lap back and brakes where the profile does).

### What would be next if it works

`v_prof_cap` is the whole remaining lever and it is pinned at 1.00 — `mpcc` is
not allowed to be faster than the offline profile anywhere, only to reach it
later. At 1.25 the surrogate says 0.6 s/lap more and collides at s 36.5-36.8.
The honest version of that experiment is a per-zone cap (1.00 in the three
hand-held safe zones, 1.05-1.10 elsewhere), which is the same shape as
`finals_mesh_sched.json` and owes the same two gates.

### Files

| file | what |
|---|---|
| `src/pit_crew/mpcc_core.py` | candidate A's solver |
| `src/pit_crew/mpcc.py` | its ROS node (`CONTROLLER=mpcc`) |
| `src/pit_crew/ffb_core.py`, `ffb.py` | candidate B |
| `src/pit_crew/planreplay.py`, `fastpub.py` | candidate C |
| `tools/sim_tracker.py` | `--tracker mpcc/ffb/fastpub`; three identity hooks |
| `tools/surrogate_race.py` | `--tracker` |
| `src/pit_crew/mpc3.py` | `MPC_CLASS` / `EXTRA_PARAMS` plus three identity hooks |
| `tests/test_overhaul_candidates.py` | the identity property, the envelope, node liveness |
