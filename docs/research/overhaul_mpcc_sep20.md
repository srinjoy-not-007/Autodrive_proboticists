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

Baseline, same line and flags: `mpc3` laps **11.015 s** at 0.058 and
**11.545 s** at 0.098 (200 surrogate laps each, 0 collisions).

| candidate | 0.058 s | 0.098 s | verdict |
|---|---|---|---|
| **A. `mpcc` — joint speed+steering MPC** | −0.085 s/lap, **5 collisions / 300 laps** | **−0.425 s/lap, 0 / 300** | **clears the bar at the organisers' period and only there.** Does not pass both gates, so no shipping slot; one bounded live experiment is justified (§6) |
| B. `ffb` — preview feedforward + feedback | −0.135 s/lap, 0 / 100 | **+0.635 s/lap**, 0 / 100 | **rejected offline** — its anticipation is a distance computed from a constant dead time, so it looks at the wrong place at a 69 % longer period |
| C. `fastpub` — plan re-read between poses | −0.015 s/lap, 0 / 100 | +0.025 s/lap, 0 / 100 | **null, and the mechanism says it had to be**: no topic on this stack is faster than the pose (measured) and the bridge latches one command per tick |

The one-line answer to the brief's question — *can an architectural alternative
beat mpc3 by ≥ 0.3 s/lap at the slow period without collisions?* — is **yes at
that period and no overall**: candidate A does 0.425 s/lap over 300 clean
surrogate laps at 0.098 s, and then takes five collisions in 300 laps at
0.058 s at a corner the live record already names.

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

### 3.2 Where the time comes from (0.098 s, 24 laps, per 2 m bin, final config)

| zone | mpcc − mpc3 |
|---|---|
| straight s 0-16 | **−0.140 s** |
| first hairpin s 16-22 | **−0.135 s** |
| s 24-27 (the tightest corner) | −0.135 s |
| s 37 corner (s 32-41) | −0.135 s |
| last hairpin s 41-46 | **+0.060 s** |
| **lap** | **−0.485 s** |

The single biggest bin is s 16.0 (−0.080 s), which is the braking zone into the
first hairpin — the corner the Sep 20 braking audit measured the car entering
+0.25 m/s hot at the organisers' period, and the one that costs the shipping
stack 0.2 s/lap there. The last hairpin goes the other way, which is the
expected sign: the live record says that corner is capped by the steering lock,
so a controller that plans more entry speed there pays for it.

### 3.3 The 100-lap gates found a collision, and what fixed it

The 24-lap screen above is clean everywhere. **The 200-lap gate is not**, and
that is the whole reason for running it:

| arm | period | laps / collisions | median | at s |
|---|---|---|---|---|
| `mpc3` | 0.058 | 200 / **0** | 11.015 | — |
| `mpc3` | 0.098 | 200 / **0** | 11.545 | — |
| `mpcc` v1 | 0.058 | 200 / **3** | 11.080 | 0.9 ; 24.1, 25.2 |
| `mpcc` v1 | 0.098 | 200 / **2** | 11.250 | 0.9, 3.2 |

Both failure sites are corners the live record already names: **s 0.9-3.2 is
the last hairpin's exit onto the start straight** — the `_ay90h2` failure
("cte grows monotonically from s 1.5 to −0.4 m by s 4.3", triggered by the
ay-9 exit speed of the last hairpin) — and **s 24-25 is the tightest point of
the lap**. Both are exit-acceleration failures, i.e. the one thing
`docs/FINALS_TUNING` says running wide tracks.

Two candidate fixes, 48 laps per cell:

| arm | 0.058 | 0.098 |
|---|---|---|
| fade the servo reserve by curvature (`mpcc_res_kappa 0.4`) | 48 / 0, **14.400** | 48 / 0, **13.910** |
| cap the setpoint by the envelope at the CURRENT curvature (`mpcc_cap_now`) | 48 / 0, **11.060** | 48 / 0, **11.220** |
| both | 48 / 0, 14.400 | 48 / 0, 13.910 |

**The curvature fade is a trap and is now default-off with the number in the
comment.** Unlike `servo_head_kappa`, which fades only the extra head on top of
the setpoint, it fades the reserve that keeps the speed servo saturated at all,
so the car sits under plan through every corner: 14.40 s/lap against 11.06.

**`mpcc_cap_now` is the version of the idea that works** and is on by default:
the setpoint is additionally capped by `envelope_speed(kappa under the car)`.
On a hairpin exit the line's curvature falls away inside one lead distance
while the car is still turning, so a setpoint read ahead is a licence to
accelerate mid-corner; capping it by the measured envelope at the curvature the
car is actually at is the same statement the solver's own hinge makes, applied
to the number the throttle receives.

That fixed the DISPLAY period and not the organisers' one. 200 laps per period,
`mpcc_cap_now` on:

| arm | period | laps / collisions | median | at s |
|---|---|---|---|---|
| `mpcc` v2 | 0.058 | **200 / 0** | 11.060 | — |
| `mpcc` v2 | 0.098 | 200 / **3** | 11.265 | 0.8, 3.2 ; 24.6 |
| `mpcc` v2, `lon_brake_plan 4.6` | 0.058 | 200 / **1** | 11.065 | 24.4 |
| `mpcc` v2, `lon_brake_plan 4.6` | 0.098 | 200 / **2** | 11.245 | 24.3, 20.6 |

Same two corners, and `lon_brake_plan` is not the variable — planning on the
profile's own 4.6 m/s² of brake collides as well. So it is not "arrives hot
because it braked late": it is the corner EXIT.

**The fix that works is the combined-slip bound on the longitudinal input**
(`mpcc_ellipse`, default 1.0). A tyre already spending `ay` of its envelope on
turning has `sqrt(1 - (ay/ay_max)²)` of the longitudinal axis left, so the
plan's accelerating bound is scaled by that — one-sided, because entering a
corner slower has never been the failure here. It is item (2) on
`docs/FINALS_TUNING`'s own literature list ("exit-acceleration-limited
(combined-slip) forward pass, matching the finding that running wide tracks
exit acceleration"), and it makes the candidate both **safer and faster**:
48 laps per cell, `ell 1.0` gives 10.940 at 0.058 and **11.110** at 0.098
against `ell 0` 11.060 / 11.220, `ell 0.5` 11.050 / 11.230, and
`ell 1.0 + grip_use 0.85` 11.040 (3 collisions) / 11.250.

Gates on the final configuration, 300 laps per period (3 seeds × 100):

| arm | period | laps / collisions | median | vs mpc3 |
|---|---|---|---|---|
| `mpc3` | 0.058 | 200 / 0 | 11.015 | — |
| `mpc3` | 0.098 | 200 / 0 | 11.545 | — |
| **`mpcc` final** | **0.098** | **300 / 0** | **11.120** | **−0.425 s/lap** |
| `mpcc` final | 0.058 | 300 / **5** | 10.930 | −0.085 s/lap; s 1.0-1.4 ×4, s 38.0 |

**That is the honest state of candidate A, and it is not a clean sweep.** At
the organisers' period it does exactly what the brief asked: 0.425 s/lap over
300 surrogate laps with no collisions, well past the 0.3 s/lap bar. At the
display period it is only 0.085 s/lap up and it takes 5 collisions in 300
laps — about one in 60 — all but one at s 1.0-1.4, the last hairpin's exit,
which is the same corner `_h2` and `_ay90h2` failed at live.

Note the direction of the trade across the three versions: **v2 (no ellipse) is
clean at 0.058 and collides at 0.098; the final one is clean at 0.098 and
collides at 0.058.** Same corner family both times. A third attempt at closing
it — capping the setpoint by the worst curvature over 0.8-2.5 m of line BEHIND
the car — is recorded in `mpcc_cap_back_m` as closed: every value over-caps the
whole lap (11.33-12.86 s/lap) and two of the three still collide.

Under this project's own rule — *a non-zero collision count at either period
disqualifies a rung*, because failures here are clusters and not a rate —
**candidate A does not earn a shipping gate as it stands.** What it has earned
is a narrower experiment; see section 6.

### 3.4 The mechanism, in the trace

12 laps at 0.098 s, shipping flags, median speed and brake duty per 1 m of `s`
through the braking zone into the first hairpin:

| s | speed mpc3 / mpcc | brake duty mpc3 / mpcc |
|---|---|---|
| 10 | 7.73 / **8.16** (+0.43) | **69 %** / 4 % |
| 11 | 7.15 / **7.94** (+0.80) | **72 %** / 1 % |
| 12 | 6.47 / **7.46** (+0.99) | 47 % / 28 % |
| 13 | 5.89 / **6.81** (+0.92) | 43 % / **83 %** |
| 14 | 5.36 / 5.95 (+0.59) | 48 % / 67 % |
| 16 | 3.78 / 4.32 (+0.54) | 46 % / 57 % |
| 18 (entry) | 1.73 / 1.91 (+0.18) | 0 % / 15 % |
| 19 (apex) | 1.86 / **1.78** (−0.08) | 8 % / 37 % |

**mpc3 starts braking at s 10.1 and holds ~70 % duty for two metres; `mpcc`
runs 1 % duty there and brakes hard at s 13.** It carries up to 1 m/s more
through the zone, arrives at the entry only 0.18 m/s hot and takes the apex
0.08 m/s SLOWER than mpc3 — brake later and harder, rather than earlier and
longer, which is the thesis the candidate was built on. Over the lap the
exact-0.0 brake is published on 22.9 % of samples against mpc3's 16.7 %.

### 3.5 The cost: solve time

Same host, same pinned cores, 6 laps at 0.098 s:

| | median | p95 | max |
|---|---|---|---|
| `mpc3` | 5.83 ms | 6.41 ms | 7.04 ms |
| `mpcc` (final) | **14.92 ms** | **21.45 ms** | 27.58 ms |

**This is the one real shipping risk in candidate A.** It is inside the 20 ms
budget on this host, but the live solve is not this host's: `docs/FINALS_TUNING`
measures mpc3's live solve at 10-40 ms, i.e. up to 6x the harness figure, and
mpc3 already counts `solve over period` in its periodic log line. At 2.2x mpc3
the worst live case would drop poses at the display period. Mitigations, in
order: `mpcc_iters:=1` (~-35 % of the solve, costs 0.13 s/lap at 0.098 and
nothing at 0.058), `mpcc_outer:=1`, `horizon:=16`. **Read the node's
`solve NN.N ms` line in the first 20 s of the first live run before trusting
any lap time from it.**

(A rollout micro-optimisation — one corridor lookup per step instead of two,
and skipping the saturation exponential when nothing is over the knee — took
the median 16.63 -> 13.04 ms and reproduced the same 6-lap run to every printed
digit; the combined-slip bound then put ~2 ms back.)

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

**Candidate A is the only one worth any live time, and it is NOT a shipping
candidate.** The claim it earns is narrow and specific: *at the organisers'
sensor period, giving the controller the speed as a decision variable is worth
0.4 s/lap, and the mechanism is that it brakes later and limits exit
acceleration by the measured envelope rather than by two thresholds.* The claim
it does not earn is "drop it in": it collides at the display period, at one
corner, roughly once in 60 laps.

Two ways to spend it, and they are different decisions:

* **The cheap one (recommended): one 32-lap Xvfb run, in band, to see whether
  the 0.4 s/lap is real at all.** The surrogate's pace is the part §4c
  validated, but five harness-predicted gains failed to transfer live on
  Sep 19 and this is a bigger claim than any of them. If the lap time does not
  move live, the whole architecture question is answered for the cost of one
  slot and nothing else needs doing. **Do not run Gate D on it as it stands.**
* **The expensive one: close the last hairpin's exit first.** The failure is at
  s 1.0-1.4 at 0.058 s, where `v_prof_cap` is already 1.00 and `mpcc_ellipse`
  already limits exit acceleration; the obvious next lever is a per-zone
  `v_prof_cap` below 1.0 over s 42-2.5, which is the same shape as
  `finals_mesh_sched.json` and was not tried. Only after that is it worth
  Gate C + Gate D.

Standing caveats, in the order they should be believed:

1. **A surrogate clean sheet is not evidence — but its collisions are.** The
   collisions it found while the candidate was being built are the useful part:
   s 36.5-36.8 when `v_prof_cap` was 1.25, s 24 when the arc grid was frozen,
   s 21 when the servo reserve was two-sided, s 0.8-3.2 twice. Every one is a
   corner the live record already names, which is the strongest available
   argument that the remaining 0.058 s failure is real too.
2. **The pace claim is the part the surrogate is good at** (per-2 m segment
   times reproduce the live log to a median of 0.02 s), and it is a 0.4 s/lap
   claim, not a 0.05 s/lap one — i.e. outside the ±0.06 s/lap accuracy §4c
   measured, and outside the one disagreement it has on this geometry
   (0.19 s/lap on `apex_f42k`). It would still not be the first 0.4 s/lap
   surrogate claim to evaporate.
3. **Solve time (3.5) is the thing most likely to turn this into a DQ**, and it
   is measurable in the first 20 s of the first run.
4. It must not go near the image on anything less than Gate C **and** Gate D,
   and it cannot pass Gate D today.

### The live run

The 32-lap Xvfb probe, in band (`RATE_MIN_DT`/`RATE_MAX_DT`, the Sep 20 method
rule), on a quiet host. `LAPS=100` turns the same command into Gate C for
whoever closes the last hairpin first:

```bash
LOC_ARGS='-p max_step_mps:=9.0 -p publish_on_reloc:=false' \
RATE_MIN_DT=0.094 RATE_MAX_DT=0.100 \
SIM_QUALITY=ultra CONTROLLER=mpcc MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
  ./scripts/run_exp.sh mpcc_x32 apex_f42k_race \
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

What to read out of it, in order: (1) the node's periodic `solve NN.N ms`
against the measured period — if it exceeds it, switch to `mpcc_iters:=1` and
restart, and do not read that run at all; (2) the lap median against a same-
session mpc3 control on the same line and band (the surrogate says
11.545 -> 11.120, so live it should be ~0.4 s/lap); (3) collisions, and where.

`brake_mode:=zero` is passed for one reason: the node logs the override
(`ServoThrottle brake_mode zero -> none`), so a run that lost the flag is
visible in the first second of the log. In `mpcc` the brake is the SOLVER's.

**Abort rules.** Any collision; the solve over the period; driven ay p99 above
9.3; a rising cte p95.

**Knobs, with what each is worth offline.** `mpcc_iters:=1` cheaper solve for
+0.13 s/lap at 0.098; `lon_brake_plan:=4.6` brakes where the profile does, for
+0.12 s/lap; `mpcc_ellipse:=0` restores the version that is clean at 0.058 and
collides at 0.098 (11.060 / 11.265); `mpcc_brake_a` 1.5 costs 0.17 s/lap at
0.098. `mpcc_res_kappa` and `mpcc_cap_back_m` are closed — both are recorded
in the defaults with the numbers that closed them.

### What would be next if the pace is real live

1. **The last hairpin's exit at the display period**, which is the one thing
   between this and a shipping gate. `v_prof_cap` is a single global 1.00; a
   per-zone version below 1.0 over s 42-2.5 is the obvious next arm and is the
   same shape as `finals_mesh_sched.json`.
2. **Then `v_prof_cap` upward everywhere else.** It is the whole remaining
   lever: `mpcc` is currently not allowed to be faster than the offline profile
   anywhere, only to reach it later. At a flat 1.25 the surrogate says 0.6 s/lap
   more and collides at s 36.5-36.8, so this one is per-zone too.

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
