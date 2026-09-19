# Lap-time levers, ranked (research + prototype, Sep 20 2026)

Brief: find NEW, applicable ways to cut the 10-lap race time from **107.8 s
display / 113.5 s Xvfb** (image v7, `finals_mesh_apex_f42k_race.csv`, 100 laps
clean at both periods). Three web/code agents ran in parallel; everything below
is either a source we had not read or a number measured in this session.
Nothing here was driven — the display remains the gate.

---

## 0. The two facts this session added

**(a) 42 % of the ideal lap is spent braking, and the plan brakes at 4.6 m/s²
while the plant holds 6.0-6.8.** Braking zones on the shipping line
(`ax < -1.0`), from the line itself:

| s | length | v | \|kappa\| p50 | time |
|---|---|---|---|---|
| **10.8-18.7** | 8.00 m | 8.60 -> 1.84 | **0.00** | **1.575 s** |
| **31.5-37.3** | 5.80 m | 6.78 -> 2.36 | 0.16 | **1.282 s** |
| six others | 0.2-2.0 m | — | 0.08-0.38 | 1.462 s |

Total 4.319 s of a 10.282 s ideal lap. The two big zones are **straight**, so a
locked wheel costs nothing there. Re-read of `data/logs/brake_20260919_132335.csv`
(7 trials, 84 pooled samples, first 3 samples of each trial dropped as the
command-latency transient): sustained deceleration medians **6.00 / 6.06 / 6.34
/ 6.62 / 6.78 / 6.29 m/s²** in the 1-2 ... 6-7 m/s bins — essentially flat at
~6.3, not speed-dependent, and **well above the planned 4.6**. The old "held
ceiling 4.55 m/s²" is the low-speed asset figure and is superseded by the probe.

**(b) The controller, not the profile, is what brakes at 4.15.** The harness's
fitted plant constant is `FIT_BRAKE_AT_ZERO = 4.15` (`tools/sim_tracker.py:192`),
fitted from race logs *before* the probe existed. It is not a plant limit — it
is the **effective** rate the current controller achieves, because
`brake_limited_target()` in `src/pit_crew/throttle2.py` computes its own
brake-limited setpoint at `a_brake` (default **4.0**), so the profile's ramp is
overridden by the controller's internal 4.0 curve no matter what the line says.
`a_brake` and `brake_a_full` have to move **together with** the profile; moving
either alone does nothing (measured, section 3).

---

## 1. Ranked candidates

### 1. Locked-wheel straight-line braking — profile + `a_brake`/`brake_a_full` (PROTOTYPED)
**Mechanism.** Plan 6.0 m/s² of deceleration where `|kappa| <= 0.15`, tapering
back to 4.6 by `|kappa| >= 0.50`, and raise the controller's own
`a_brake`/`brake_a_full` to match. The brake point moves ~1.8 m later in the
s 10.8 zone and ~1.0 m later in the s 31.5 zone, both at near-zero curvature.
**Why it transfers.** The sim brake is exact-zero-throttle only, and
`brake_mode:=zero` already publishes a hard 0.0; the plant answer to that is
6.0-6.8 m/s². Nothing about it is rate-sensitive — the deceleration is a plant
constant, not a loop gain, so it survives the 0.058/0.098 lottery. A locked
wheel steers nothing, which is exactly why the rate is gated on curvature.
**Gain.** Ideal lap 10.282 -> **10.062 s**. Surrogate: **-0.20 / -0.24 s/lap**
(0.058 / 0.098) against the fitted plant, **-0.08 / -0.12** against the
probe-measured plant — call it **0.8-2.5 s per race**.
**Cost.** ~2 h (done). **Collision risk.** Low-moderate: the extra braking is
on the straight, but a longer zero-throttle hold means less steering authority
in the last metre before turn-in. Harness clearance is *not* degraded (min
footprint 0.135 m vs the doc's 0.071-0.127 for the shipping arm).
**Cheapest validation.** Done — section 3.

### 2. The hairpin-1 dwell cap (PROTOTYPED, free)
**Mechanism.** Breakaway is a constant asked **yaw rate** `r = v*|kappa|`
(coordinator's fit, `tools/sysid_breakaway.py`): attached cells reach 2.63
rad/s, cells that let go start at 2.79, but on track 0.19-0.29 s excursions
past that still deliver 0.95 of asked. So the usable limit is a **dwell**, not
a level. Measured on the shipping line, exactly four runs exceed 2.4 rad/s:

| s | dwell | r max | note |
|---|---|---|---|
| **18.8-19.5** | **0.400 s** | 3.12 | **the corner that produces every collision** |
| 24.1-24.7 | 0.210 s | 2.70 | the apexing family's weak spot (b25/b50 hit here) |
| 37.4-37.6 | 0.126 s | 2.69 | the "released corner" |
| 43.3-43.6 | 0.236 s | 3.19 | hairpin 2 — clean over 100 laps |

Hairpin 1 is **the only place on the lap held past steady-state breakaway for
longer than any excursion the live car has ever been shown to survive**, and it
is the only corner that collides. That is a new, measured, mechanistic account
of the s 19 cut.
**Gain.** Safety, not time: shaving the middle of that run to a 0.25 s dwell
(matching hairpin 2's demonstrated-safe 0.236 s) costs **0.015 s of ideal lap**
and 0.24 m/s over 0.3 m. Surrogate: indistinguishable from arm 1 (within 0.01
s/lap at both frames).
**Cost.** 0 h extra (same tool). **Risk.** Essentially none — it only reduces
speed, in the slowest corner on the track.

### 3. `ax_max_machines(v)` — stop the profile promising acceleration we cannot make
**Mechanism.** TUM's `calc_vel_profile` takes a *separate* drivetrain limiter
`ax_max_machines` from the tyre ggv and uses `min(tyre, machine)`. ForzaETH's is
9.109 m/s² at 0 falling linearly to 1.475 at 15 m/s; **ours is flat 6.5**.
Measured here from the probe's acceleration phase at a constant 0.40 throttle:
**2.8 / 4.2 / 4.2 / 3.8 / 3.7 m/s²** at 0-1 / 3-4 / 4-5 / 5-6 / 6-7 m/s — and
in a race the open-loop map only ever commands ~0.040*v ≈ 0.34. The plan's
6.5 is fiction on every corner exit; the repo already records "below plan on
62 % of samples, by 0.96 m/s wherever ax > 3".
**Why it transfers / doesn't.** Lowering the plan to the truth makes the ideal
lap *slower*, so it is not a time lever by itself — it is an **accuracy** lever:
the exit servo and `e_cut` stop being permanently saturated, and the MPC's
rollout stops predicting a speed the car will never reach. Pair it with the
throttle feedforward (`throttle_a_gain`, or the `thr_pi_*` arm already in
flight) which is what actually buys the acceleration back.
**Gain.** Unknown sign alone; ~0.1-0.2 s/lap in combination. **Cost** 2 h.
**Risk** low offline, medium live. **Validation:** reprofile with a measured
`ax_max_machines` column, surrogate both frames; abort if the exit servo
saturation does not fall.

### 4. Corridor-bound adaptation from the repeatable cross-track error
**Mechanism.** Split the measured lateral deviation into a repeatable and a
non-repeatable part, shift the optimiser's track bounds locally by the
repeatable part (arc-length indexed, hard low-pass), re-solve min-time.
Sources: **TUM/A2RL winner, arXiv 2602.08571** (*"shifting the track bounds
locally using the s-based solver constraints"*), and **Schwehn et al., ITSC
2026, arXiv 2608.14448** — "Control-Informed Constraint Adaptation", which
reports **-1.8 s of lap time** at 25 ms median runtime with no increase in
collision risk.
**Why it transfers.** It costs nothing at runtime, adds no new failure mode,
and is indexed in arc length — immune to the sensor-period lottery, which is a
*non-repeating* disturbance. We already have the input: 100-lap clean logs at
both periods, and the harness measures the car at **+0.042 to +0.171 m inside
the line in corners**. It is also the principled form of what `apex_f42k` found
by hand ("the line finally goes where the car was always going").
**Gain.** 0.1-0.3 s/lap, guessed from the apex_f42k precedent (-0.143 s ideal
for the same mechanism applied by eye). **Cost** 4-6 h. **Risk** moderate —
the ILMPC guard (arXiv 2508.01103) exists precisely because learners shortcut
corners, and every collision we have is a corner cut, so a shortcut guard is
mandatory. **Validation:** build the shift from the two 100-lap logs, re-run
`optimize_raceline.py`, surrogate both frames, then the usual two gates.

### 5. `dyn_model_exp = 1.0` — the combined-g shape is wrong for a locked wheel
**Mechanism.** `tph.calc_vel_profile`'s `vel_calc_opts['dyn_model_exp']` sets
`ax_avail = ax_max * (1 - (ay/ay_max)^e)^(1/e)`; e=2 is an ellipse (trail
braking available), e=1 a diamond (none). **Our brake locks the wheels, so
there is no lateral force at all while braking** — e=1.0 is the physically
correct shape and both TUM and ForzaETH ship 1.0. If our profiler is on the
default ellipse we are over-promising trail braking into the s 19 entry, which
is the exact corner that cuts.
**Gain.** Small on time (probably negative), real on the entry margin.
**Cost** 1 h. **Risk** low. **Validation:** regenerate with e=1.0, diff the
entry speeds at s 17-19, surrogate both frames.

### 6. FBGA — forward/backward with a generic velocity-dependent envelope
**Mechanism.** [DRIVEWISE/FBGA](https://github.com/DRIVEWISE/FBGA) (MIT,
arXiv 2509.26428, RA-L 2026): the forward and backward passes take the
acceleration limits as **callables**, so an arbitrary non-convex,
speed-dependent, *asymmetric* envelope is native — `GG_shape1(v)` max ax,
`GG_shape2(v)` min ax, `gg_range(v)` lateral. Within **0.11-0.36 %** of a full
OCP, ~10³x faster, 15-20 ms for a 5214-node lap.
**Why it transfers.** Our envelope is exactly that shape: drive ≈ 4.2 and
falling, brake = 0 or 6.3, lateral = `r_max/|kappa|` with a 10.5 ay ceiling.
Our own `long_passes()` (in the new tool) is a hand-rolled two-limit version of
it. **Gain** none beyond arms 1-3 on this track — the value is a clean
implementation for the *next* track. **Cost** 6-8 h + a C++/pybind build.
**Verdict: post-finals.**

### 7. Global velocity-scaler sweep to locate our own cliff
ForzaETH's published tuning result (arXiv 2403.11784): a uniform speed scaler
sweep gives *linearly* decreasing lap time down to 78 %, best **6.34 s at
79 %**, and **collisions from 80 % up** — one knob that finds the cliff, then
Bayesian per-sector refinement. Their `sector_tuner` blends the scaler linearly
over ±10 waypoints at each sector boundary, which is a cleaner replacement for
our three hand-placed slow zones. **We have `speed_scale` already.** Cost 1 h of
harness time; gain is knowledge, not seconds — it tells us how much of our
margin is uniform and how much is one corner. Worth doing while the display is
busy.

### 8. `mincurv_iqp` instead of one-shot min-curvature
`opt_min_curv` linearises curvature about the reference line. That is fine at
TUM's full-size kappa ~0.12 1/m and questionable at **our 1.8 1/m**.
`tph.iqp_handler` re-splines and re-solves until the curvature error converges;
ForzaETH ship `mincurv_iqp` as their only global line with F1TENTH settings
(`iqp_iters_min 5`, `iqp_curverror_allowed 0.1`). If `optimize_raceline.py` is
single-shot this is a free geometry gain. **Cost** 2 h, **gain** unknown,
**risk** it changes the shipping line, so it needs the full gate ladder.
Check first whether we already iterate (`--sweep-iters 2` suggests partly).

---

## 2. Explicitly NOT applicable (so nobody re-searches)

- **GP / residual / adaptive dynamics learning** (cautious GP-NMPC 1711.06586,
  BayesRace, HyperMPC 2508.06181, LLA-MPC 2505.19512, AD-MPCC 2607.00141).
  All buy back *model mismatch* or *friction variation*. Our achieved/kinematic
  curvature is 0.96-1.01 and the surface never changes. There is no residual.
- **MPCC.** Documented failure mode is the contouring/lag/progress weight
  conflict on narrow high-curvature tracks (CA-PF-MPCC, CEP 2026); the part
  that pays (CiMPCC, 2502.03695, +11-12 % on F1TENTH) is curvature -> reference
  velocity, which our offline profile already gives us. Switching is a downgrade.
- **Event-/self-triggered MPC.** Solves *less* often to save compute. We are
  compute-rich and measurement-starved; `control_mode:=timer` already showed
  the cost.
- **LMPC / safe-set racing** (1610.06534, IteraOptiRacing 2507.09714). Already
  rejected live; the safe set expands *toward the wall*, wrong sign under a
  10 s penalty.
- **CBF safety filters** (2505.02395, 2601.12143). Structurally identical to
  our arbiter: a last-millisecond override on a chattering estimate.
- **Drifting for the hairpins** (BE-LMPC 2608.28723). Needs modulable rear-axle
  force and recoverable sideslip; our locked-wheel bang-bang brake gives zero
  trail braking. Do not pursue.
- **Chance-constrained / risk-aware planning.** Every paper found concerns
  *moving* obstacles. **Nothing in the literature optimises expected race time
  = lap + 10 s x P(collision) against a static corridor.** Our `MIN_CLEARANCE`
  + drive-it-to-believe-it rule is the state of the art here; arXiv 2608.14448
  (arm 4) is the closest usable substitute.
- **TUM `opt_mintime_traj` variable friction.** Friction varies *spatially*,
  never with speed, and `mue` is a scalar; speed enters only via aero downforce,
  which is zero at 1:10. It does **not** solve our problem despite the name.
  (It does forbid simultaneous pedals — `f_drive * f_brake == 0`, matching our
  sim — but it is continuous, not bang-bang, and IPOPT takes minutes.)
- **Bang-bang braking in the literature: a genuine gap.** No racing work
  assumes on/off braking. Closest is hybrid-MPC/MLD two-mode longitudinal
  control for highway tracking. The correct theory is bang-singular-bang
  min-time; the practical move is what `brake_mode:=pulse` already does.
- **Other entrants.** No RoboRacer Sim Racing League entrant besides
  MonacoF1/Ekumen (already mined) has published a stack. ICRA 2025 podium was
  UPenn BitShifters and ForzaETH; neither published their sim entry. The
  **AutoDRIVE-Ecosystem org has 17 repos and no reference racing stack, no
  example controller and no raceline anywhere**; `-Racetracks` is meshes only.
  VAUL's RACE stack is still not public.
- **Hairpins tighter than the steering lock: no research answer.** Nothing
  quantifies it; our `apex_deviation()` charge is already more rigorous than
  anything published.

---

## 3. What was prototyped, and the offline numbers

New tool **`tools/grip_envelope.py`** — re-draws a raceline's longitudinal
ramps at the measured brake without touching geometry or corner caps, and
optionally applies the yaw-rate dwell cap. Design notes:

- The ceiling is the input profile wherever it was *not* brake-limited, and the
  **reconstructed generation cap** `v_lat_generated()` (the `--ay 9.0 /
  --ay-tight 4.5 / --kappa-tight 1.15` ramp `make_track_lines.PROFILE` used)
  where it was. Without that reconstruction the first cut opened mid-corner
  braking points *above* their lateral cap and reported a fake -0.55 s.
- `--only` restores the input byte-for-byte outside the named windows, so each
  arm is a single-variable experiment. The control arm (`--brake-hi 4.6`, same
  construction) still differs from the input by **-0.063 s** of ideal lap —
  that is the lateral-jerk limiter, which this tool does not reproduce — so the
  honest brake gain is measured against the control, not the input.

Lines built (committed):
```
tools/grip_envelope.py data/racelines/finals_mesh_apex_f42k_race.csv \
  --brake-hi 6.0 --only 10.5:18.0,31.0:36.5 \
  --out data/racelines/finals_mesh_f42k_lb60_race          # ideal 10.062 (-0.220)
tools/grip_envelope.py ... --brake-hi 6.0 --r-dwell 0.25 \
  --out data/racelines/finals_mesh_f42k_lb60d25_race       # ideal 10.077 (-0.205)
tools/grip_envelope.py ... --brake-hi 4.6 --brake-lo 4.6 \
  --out data/racelines/finals_mesh_f42k_ctl_race           # control, ideal 10.218
```

`tools/surrogate_race.py`, 32 laps x 3 seeds = **96 laps per cell**, shipping
flags plus `barrier_margin=0.34`:

| arm | 0.058 s | 0.098 s | collisions |
|---|---|---|---|
| shipping `apex_f42k` | 10.990 | 11.560 | 0 / 96 each |
| control `f42k_ctl` | 10.990 | 11.530 | 0 |
| `f42k_lb60` (line only) | 10.980 | 11.510 | 0 |
| `f42k_lb60d25` (line only) | 11.000 | 11.520 | 0 |
| **shipping line + `a_brake 6.0` + `brake_a_full 6.0`** | **10.910** | **11.460** | 0 |
| **`f42k_lb60` + those two flags** | **10.790** | **11.320** | 0 |
| **`f42k_lb60d25` + those two flags** | **10.800** | **11.310** | 0 |

**The line alone is worth almost nothing (-0.01 / -0.05); the pairing is worth
-0.19 / -0.25 s/lap.** That is the whole finding: `brake_limited_target()` runs
its own `a_brake 4.0` curve and overrides the profile.

Bracket against the probe-measured plant (`--plant-brake-at-zero 6.2` instead
of the fitted 4.15), same 96 laps/cell:

| arm | 0.058 s | 0.098 s |
|---|---|---|
| shipping `apex_f42k` | 10.980 | 11.580 |
| `f42k_lb60d25` + both flags | **10.900** | **11.460** |

So the gain is **-0.08 to -0.25 s/lap depending on which plant brake you
believe**, i.e. **0.8-2.5 s per race**. Clearance is not spent: the winning arm
at 0.098 gives min footprint clearance **0.135 m**, lap-wide `|e_lat|` p95
0.267 m, and the car runs **+0.042 m inside** the line in corners (the shipping
arm's harness clearance is 0.071-0.127 m).

**Harness caveat, per the house rule:** a surrogate collision is a veto, a
clean sheet is nothing. All seven cells are clean and that is not evidence of
safety. The pace numbers are the usable part (demonstrated ±0.05-0.08 s/lap).

`./scripts/check_pipeline.sh` passes — all tracks drivable, shipping lines
unchanged.

---

## 4. Sources

Papers: [A2RL head-to-head, TUM 2602.08571](https://arxiv.org/abs/2602.08571) ·
[Control-Informed Constraint Adaptation, ITSC 2026, 2608.14448](https://arxiv.org/abs/2608.14448) ·
[RoboRacer planning/control benchmark, ITSC 2026, 2605.19881](https://arxiv.org/abs/2605.19881)
(same platform: cautious 6.919 s / nominal 6.643 / extended g-g-v + online
replanning **6.089 s**, peak ay 9.05) ·
[FBGA velocity profile, RA-L 2026, 2509.26428](https://arxiv.org/abs/2509.26428) ·
[g-g-v formulations, CTS 2024](https://www.sciencedirect.com/science/article/pii/S2405896324004075) ·
[apex-segmented online velocity profiles, 2505.05157](https://arxiv.org/abs/2505.05157) ·
[biasing the artificial race driver (exit-speed weight), 2504.12744](https://arxiv.org/abs/2504.12744) ·
[g-g-g-v black-box extraction, 2504.10225](https://arxiv.org/abs/2504.10225) ·
[hypersampled MPC, ACC 2024, 2310.02623](https://arxiv.org/abs/2310.02623) ·
[delay-aware robust control with time-varying delay, 2208.13856](https://arxiv.org/abs/2208.13856) ·
[closed-loop constraint tightening, 2506.13622](https://arxiv.org/abs/2506.13622) ·
[ILMPC shifted safe set, IROS 2025, 2508.01103](https://arxiv.org/abs/2508.01103) ·
[Kapania & Gerdes ILC, 1902.00611](https://arxiv.org/abs/1902.00611) ·
[CiMPCC, 2502.03695](https://arxiv.org/abs/2502.03695) ·
[ForzaETH race stack, 2403.11784](https://arxiv.org/abs/2403.11784) ·
[On-Track-SysID, 2411.17508](https://arxiv.org/abs/2411.17508).

Code: [DRIVEWISE/FBGA](https://github.com/DRIVEWISE/FBGA) ·
[TUM global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) ·
[trajectory_planning_helpers](https://github.com/TUMFTM/trajectory_planning_helpers) ·
[ForzaETH/race_stack](https://github.com/ForzaETH/race_stack) (`utilities/nodes/sector_tuner/`) ·
[roboracer-benchmark](https://roboracer-benchmark.github.io/planning_control_benchmark/).

In-repo: `data/logs/brake_20260919_132335.csv`,
`data/logs/sysid_20260919_131340.csv`, `tools/sysid_breakaway.py`,
`tools/sim_tracker.py:192` (`FIT_BRAKE_AT_ZERO`),
`src/pit_crew/throttle2.py` (`brake_limited_target`, the `a_brake` override).
Prior review: `docs/research/steering_smoothness_peer_stacks.md`.
