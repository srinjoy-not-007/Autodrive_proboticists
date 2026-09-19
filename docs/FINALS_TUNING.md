# Finals state — SRL 2026 IROS Track (`2026-iros-compete`)

**Submission due Sep 19 AoE = Sep 20, 17:30 IST.** This file is the state of the
finals entry: what is frozen, why it works, what was driven, what is closed, what
is left, and how to ship it. Read it top to bottom; the narrative of how we got
here is at the end and is only needed if a number surprises you.
`docs/FINALS_RUNBOOK.md` is the procedure, `docs/STATUS.md` the long history.

Numbers marked *model* are offline (`tools/reprofile.py`, the relative lap model,
or the 0.058 s harness) — not driven results. Every driven number carries its
log and its sensor period, because the period is the controlling variable here
(real display 0.058 s, Xvfb 0.088-0.106 s, organisers ~0.097 s).

---

## 1. The frozen config, and its evidence

Everything below is an **uncommented default** in `submission/autodrive_devkit.sh`.
Nothing needs to be set on the organisers' box.

```
map      PIT_CREW_MAP default -> /home/data/maps/finals_mesh
line     finals_mesh_race.csv  (44.82 m, |kappa| max 1.821, 0.377 m effective
         clearance, ideal 11.02 s)  controller mpc3
CTRL_ARGS  -p map_prefix:=$MAP -p rate_adaptive:=true
           -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p steer_tau:=0.04
           -p barrier_margin:=0.34        (base)
           -p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05   (the pick; last -p wins)
           -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6
LOC_ARGS   -p max_step_mps:=9.0 -p publish_on_reloc:=false
env        OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=1
```

Driven evidence for exactly this config:

| Where | Period | Result | Log |
|---|---|---|---|
| **Real display, Ultra (the organisers' setup)** | 0.058 s | **32 laps, 0 collisions, OFFICIAL 114.042 s**; throttle exactly 0 on 29 % of samples through the s 19 entry; entry cut +0.19 p50 / +0.25 worst | `race_20260918_162222.csv` |
| Real display, second sample (`brake_a_min` 2.5, a measured no-op) | 0.058 s | **32 laps, 0 collisions, 113.981 s** | `race_20260918_165621.csv` |
| Xvfb, Ultra | 0.094 s | **32 laps, 0 collisions, 119.508 s** | `race_20260918_163645.csv` |
| Porto, same finals args, Xvfb | — | 32 laps, 0 collisions, 75.078 s (4 s off Porto's own tune — expected; the qualifier is done) | `race_20260918_164522.csv` |
| **The built image, raced the organisers' way** (`test_submission_image.sh`, Xvfb) | Xvfb band | **12 laps, 0 collisions, 119.57 s**, both nodes up, extra-shell check clean | — |

**`bongracers/roboracer:iros2026-finals` is built from this tree and is the
fallback submission: it is ready to push right now.** Every configuration without
the brake is a DQ on the display (section 3).

Honest target: 8.0 s/lap would need a line ~40 % flatter **and** planned ay ~8,
and this corridor (half-width 0.57-0.70 m, uniformly 1.34-1.36 m wide) cannot
flatten a line 40 %. The evidence-backed target is **~10.5-11.4 s/lap**, i.e. a
race in the 108-114 s band. *Model*, ax/brake 7.1, vmax 12, on this geometry:

| line flatness | ay 6.75 | ay 8.0 | ay 9.0 |
|---|---|---|---|
| as generated | 9.76 | 9.26 | 8.94 |
| kappa x0.85 | 9.28 | 8.82 | 8.53 |
| kappa x0.70 | 8.76 | 8.35 | 8.09 |
| kappa x0.60 | 8.39 | **8.01** | 7.76 |

---

## 2. The mechanism, in five lines

1. **The car cuts the s 19 entry on every lap.** True signed offset grows
   +0.04 m (s 16.5) → +0.28 (s 18.0-18.5) → **+0.39 m median, +0.48 p95**
   (s 18.5-19.0), then swings 0.25 m wide on the exit. The line's planned body
   clearance there is 0.20-0.30 m, so the display collisions are the tail of a
   cut that happens every lap. Not localization (lateral p95 **0.069 m**), not
   the line, not the barrier.
2. **The simulator brakes only when throttle is exactly 0.0** (`VehicleController.cs`),
   and this stack had never published 0.0 (0.4 % of samples). `BrakeTorque =
   MotorTorque` = 428 N·m, constant. With the brake modelled, the offline harness
   reproduced the display DQ for the first time; `brake_mode:=zero` gives 0/20
   with the entry offset +0.49 → +0.25.
3. **The tyre is LINEAR to its extremum** (slip 0.01, 9.81 m/s²,
   `TireFriction.cs`). The old "sustained limit 4.6-6.6 m/s²" was chattering
   across the knee; driven ay p99 is already 9.6. Planned ay is therefore a real
   lever, gated on driven ay p99 < 9.3.
4. **The held brake ceiling is 4.55 m/s²** plus drag: the constant torque locks
   the wheel in one physics step, so what is held is the longitudinal asymptote.
   **Plan brake ≤ 5.5, never 7.0** — `brake_a_full` 4.6 is right, and the old
   "`--brake` 7.0 is ready to ship" note was never physically achievable.
5. **The cut is an early turn-in on a curvature STEP.** Binned by s, the
   published steering is already +0.08..+0.14 rad over s 16.5-18.0 where the
   line's kappa is 0.03-0.11 (the 1.60 hairpin only starts at s 19.0) — turn-in
   applied 1-2.5 m early, at **both** periods (cte +0.35 display / +0.26 Xvfb at
   s 18.0). The brake closes most of it; the residual is the last open lever.

Also from the source, so nobody re-derives it: drive torque is linear in throttle
to 1.0 (11 m/s ≈ 0.44 throttle) but constant torque means acceleration **decays
with speed**; steering slews at 3.2 rad/s in physics time with no lag and is
independent of graphics quality — Ultra does not change the physics rate; there
is no special startup mode, so the warm-up lap is free.

---

## 3. Every arm driven on the real display today (0.058 s, Ultra, 32 laps asked)

| Arm | Line / change | OFFICIAL race | Laps / collisions | Entry cut (most-inside, s 17.5-19.5) | Verdict |
|---|---|---|---|---|---|
| `fin_base_disp` | race line, barrier 0.40, lead 0.05, **no brake** | 116.793 + 90 = **206.79 s** | 11 / **10** | +0.35..+0.49 at impact | **DQ** |
| `fin_safe_disp` | safe line, same args | 120.327 + 60 = **180.33 s** | 14 / **10** | +0.37..+0.56 | **DQ** — the line is not the variable |
| `fin_bm34_disp` | race line, barrier **0.34** | 116.534 + 70 = **186.53 s** | 13 / **10** | same corner | **DQ** — the barrier is not the variable |
| `fin_brake_disp` | + `brake_mode:=zero` (brake_a_min 1.5) | **114.042 s + 0** | **32 / 0** | +0.19 p50 / +0.25 worst | **CLEAN — frozen** |
| `fin_brk25_disp` | `brake_a_min` 2.5 | **113.981 s + 0** | **32 / 0** | unchanged | **no-op** (the `brake_dv` branch fires first); a second clean sample of the frozen config |
| `fin_bl08_disp` | `thr_brake_lead_s` **0.08** | 115.343 s + 0 | 32 / 0 | p50 +0.206 / worst +0.278 — identical to 0.05's +0.202 / +0.279 | **REJECTED**: no margin bought live, 1.3 s/race lost |
| `fin_ay675_bl12_disp` | ay 6.75 line + lead 0.12 | 114.103 s | 0 | +0.201 / +0.252 | neutral: driven ay p99 6.40 vs 6.27 on the frozen run, the lead's cost cancels the profile's gain (`race_20260918_171628`) |
| `fin_ay675_bl05_disp` | ay 6.75 line + lead **0.05** | **110.829 s + 0** | **32 / 0** | +0.220 p50 / +0.276 worst | **NEW BEST, candidate** — best lap 10.937 s, driven ay p95 5.93 / p99 6.62 (abort limit 9.3), brake on 9.0% of samples. The harness predicted 6/18 collisions for this pairing; live it is clean, so the offline plant is pessimistic about the brake here (`race_20260918_172451`). **FAILED Gate C**: Xvfb (0.098 s) 24 laps / **10 collisions**, all at s 38.5-38.9, 0.17-0.31 m wide (outside) at 2.4-3.1 m/s (`race_20260918_173633`). The harness's 6/18 was right for the organisers' period. Car-to-wall p2 at s 39 is 0.25 m on the frozen profile and 0.23 m on ay 6.75 at 0.098 s; the same corner runs 0.10 m *inside* at 0.058 s. Second marginal zone s 26-27 (0.35 -> 0.29). Follow-up: hybrid profile `finals_mesh_hyb675_race.csv` = ay 6.75 profile with the frozen speeds blended in over s 23.5-28.5 and s 34.5-41.5 (ideal 10.82 vs 11.02 / 10.70) |
| `fin_hyb675_x100` (Xvfb) | **hybrid** `finals_mesh_hyb675_race.csv`: ay 6.75 profile, frozen speeds blended over s 23.5-28.5 and 34.5-41.5 | **117.722 s + 0** (Xvfb 0.094 s) | **100 / 0** | wall p2 0.247 / min 0.195 m at s 38-41 | **passed Gate C**; frozen was 119.508 s on 32 Xvfb laps (`race_20260918_174633`) |
| `fin_hyb675_disp` | hybrid, display | **111.823 s + 0** | **32 / 0** | +0.220 p50 / +0.281 worst; ay p99 6.38 | **CLEAN AT BOTH PERIODS — promotion candidate**; best lap 11.102 s; frozen was 114.04 / 113.98 (`race_20260918_180948`) |
| `fin_servo_disp` | frozen line + `servo_k_p 0.5 servo_head_max 0.3` | 112.447 s + 0 (scored laps clean) | 32 / **1** | +0.205 / +0.265 | **REJECTED at this gain**: 1.6 s faster than frozen, but lap 32 hit the wall on the hairpin EXIT (s 19.97, 0.38 m wide at 1.47 m/s) — the exit throttle it adds is what runs the car wide there. 1 collision / 32 laps is ~3 s of expected penalty per 10-lap race against a 1.6 s gain (`race_20260918_181740`) |
| `fin_pubgate_disp` | frozen line + `pub_gate_corr 0.05` | **113.745 s + 0** | **32 / 0** | +0.205 / — ; s 16-18 mean cmd +0.043 (frozen +0.038) | **works, neutral on time**: actuator steering std on s 2-16 **0.0242 -> 0.0181 rad (-25 %)**, lap time 0.3 s better than frozen (inside noise). Candidate to stack on the hybrid line after its own 100-lap Xvfb gate (`race_20260918_182556`) |
| image test (Xvfb, default entry) | built image with the hybrid line | 118.396 + 10 = 128.40 s | 12 / **1** scored (+1 during the post-run stop at the hairpin, 8 m/s, not scored) | — | **s 38.7 is marginal at the organisers' period for the frozen speeds too**: lap 5 hit 0.32 m wide at 2.41 m/s, dt 0.093 (`submission_20260919_000609`). Fix in flight: `finals_mesh_hyb675b_race.csv` = hybrid shifted 0.08 m to the inside over s 36-42 (line-to-wall 0.548 -> 0.515 on the inside, min half-width 0.50 at s 38.1), exit speed capped at 3.0 over s 38.3-39.8 and ay <= 6.0 in the zone; ideal 10.84 s (`tools/fix_s39_finals.py`). Same image, cmd entry: **12 / 0 / 117.261 s** Xvfb; real display: **12 / 0 / 112.085 s** |
| `fin_hyb675b_x100` (Xvfb) | `finals_mesh_hyb675b_race.csv` | **118.074 s + 0** (0.098 s) | **100 / 0** | wall min at s 39-41 **0.566 m** (hybrid 0.195), s 37-39 0.401 (0.336), s 26-28 0.318 (0.266) | **passed Gate C with real margin**; 0.35 s/race slower than the hybrid on Xvfb. Display runs in flight, alone and with `pub_gate_corr 0.05` (`race_20260918_185202`) |
| `fin_hyb675b_disp` | hyb675b, display | **111.585 s + 0** | **32 / 0** | — | **PROMOTED**: clean at both periods with margin (`race_20260918_191411`) |
| `fin_hyb675b_pg_disp` | hyb675b + `pub_gate_corr 0.05` | **111.653 s + 0** | **32 / 0** | — | time-neutral again (111.59 alone); owes 100 Xvfb laps before it joins the image (`race_20260918_192212`) |
| image v2 (hyb675b) | built image, raced as shipped | Xvfb default **117.129**, Xvfb cmd **117.795**, display **111.743 s**, all + 0 | 12 / 0 each | — | **SHIPPABLE**; supersedes image v1 (`submission_20260919_*`) |
| `fin_hyb675b_pg_x100` (Xvfb) | hyb675b + `pub_gate_corr 0.05` | **117.112 s + 0** | **100 / 0** | — | **passed Gate C; pub_gate 0.05 SHIPS** (`race_20260918_194320`) |
| `fin_lead_x32` (Xvfb 0.095) | hyb675b + pub_gate + full lead fix (`lead_mode time lead_tk 2.8, rate_terms without thr, thr_lead_s 0.126, dead_tick_c 0`) | 119.538 s + 0 | 32 / 0 | s 19.5 apex −0.344 wide (shipping −0.212) | **slower live (+0.17 s/lap) where the harness said −0.67**: the throttle half works (s 15-18 −0.06, s 36-39 −0.07, entry speed 2.97 -> 3.44 at s 18) but the shorter STEERING lead turns in late at the hairpin (|steer| 0.073 vs 0.143 at s 18), runs wide, apex 1.21 vs 1.56 m/s, s 18-21 **+0.25 s**. Next arm: pin the throttle lead only (`race_20260918_200647`) |
| `fin_lead_disp` | same full lead fix, display 0.058 | 115.212 s + 0 | 32 / 0 | — | **REJECTED**: 3.6 s slower than shipping on the display too (`race_20260918_201451`). The steering-lead half is wrong at both periods; the throttle-lead pin is being tested alone |
| `fin_thrpin_x32` (Xvfb) | hyb675b + pub_gate + throttle-lead pin only (`rate_terms:=apply,predict,enc,pose thr_lead_s:=0.126`) | 117.789 s + 0 | 32 / **1** | — | **REJECTED**: no gain over 117.11 and a collision (`race_20260918_202247`). The whole lead-law family is closed live: full fix +0.17 s/lap Xvfb / +3.6 s display, throttle-only neutral + 1 collision |
| `fin_notch_x32` / `fin_notch_disp` | hyb675b + pub_gate + **weave notch** (`weave_notch_k 0.227 q 0.7 straight_d 3.0 straight_kappa 0.10`) | Xvfb **117.082 s** (shipping 117.11); display **111.297 s** (shipping 111.65), both + 0 | 32 / 0 each | — | **works, time-neutral to slightly better**: display straight steering std per 2 m bin 0.0224 (frozen) -> 0.0183 (gate) -> **0.0105 (notch)**, the 3 Hz peak gone; Xvfb weave already small (0.0139 -> 0.0133). **REJECTED Sep 19 04:10**: the two 100-lap Xvfb runs that carried it took 5 and 8 straight-line collisions; the same rung without it took 0 on the straight. 64 clean laps were not enough evidence (`race_20260918_203220`, `_204004`) |
| `fin_ay90_x32` (Xvfb 0.094) | `finals_mesh_win_b_ay90h1_race.csv` (ay 9.0, s 36-40 blend) + pub_gate + notch | 115.966 + 10 = 125.97 s | 32 / **2** | s 19.96 exit −0.439 wide at 1.5 m/s (lap 29); lap 1 at s 3.1 | **DISQUALIFIED at this rung**: 1.2 s/lap faster on clean laps (best 11.19) but the hairpin exit runs wide, the same failure as the servo arm. Fallback `_ay90h2` adds the s 17.5-21.5 blend (`race_20260918_204800`) |
| `fin_ay90_disp` | same ay 9.0 rung, display | 108.970 + 10 = 118.97 s | 32 / **4** | all four at s 19.9-20.2, −0.42..−0.46 wide at 1.5 m/s | fastest race time ever here (best lap 10.742) and ONE corner fails: the s 19 hairpin exit at both periods. `_ay90h2` (s 17.5-21.5 + 36-40 blends, ideal 10.45) in flight (`race_20260918_205557`) |
| `fin_ay90h2_x32` / `fin_ay90h2_disp` | `finals_mesh_win_b_ay90h2_race.csv` (ay 9.0, s 17.5-21.5 + 36-40 blends, ideal 10.45) + pub_gate + notch | Xvfb (0.096) **116.350 s**; display **109.338 s**, both + 0 | 32 / 0 each | Xvfb wall min: s 0-0.5 0.23 (last-hairpin exit), s 42-44.5 0.29, s 39-41 0.32 | **NEW BEST at both periods** (shipping+notch 117.08 / 111.30); best lap 10.874 display. In the 100-lap Xvfb gate; fallback `_ay90h4` blends all three hairpins (ideal 10.64) (`race_20260918_210358`, `_211133`) |
| `fin_ay90h2_x100` (Xvfb 0.097) | ay90h2 + pub_gate + notch, Gate C | 116.108 s + 0 (scored) | 100 / **5** | all five ON THE STRAIGHT (s 3.8-12.3) at 5-6 m/s with localization error 0.4-1.8 m before impact (normal 0.05) | **FAILED Gate C — a localization loss on the straight**, not a corner failure; the estimate p98 on s 3-13 is 3-4 m on this profile vs 1.3 m on shipping. Decomposed: lateral error stays 0.08-0.10 m p98 on both; the ALONG-TRACK error is a constant −0.84 m of publish lag on shipping (p98 −0.50) but reaches **+3.6 m ahead** (p98) on ay90h2 — the estimate slides forward on the featureless straight, the controller sees the s 16 corner early, brakes/turns at 6 m/s and swings ±0.4 m into the wall. **CORRECTED by the localization agent:** the 3-4 m tails are startup (first 2.8 s) and post-teleport transients; masked, the straight's along error is a 0.2-0.3 m random walk (parallel walls leave `s` to the process noise) that cannot steer on a straight whose heading turns 0.015 rad over 14 m. The five collisions are a **lateral tracking failure out of the last hairpin at 0.097 s**: cte grows monotonically from s 1.5 to −0.4 m by s 4.3 with localization lateral error 0.017 m; per-lap max|cte| over s 2-7 is 0.179 p50 / 0.490 max on ay90h2 at 0.097 vs 0.085 / 0.188 on the display and 0.134 / 0.238 on hyb675b at 0.099. The ay-9 exit speed of the last hairpin (2.22 vs 1.57 m/s) is the trigger; `_ay90h4` blends it. Opt-in safety net merged: `along_clamp_mps` (localization, default off). `_ay90h4` (all three hairpins at safe speeds, ideal 10.64) in Gate C (`race_20260918_211852`) |
| `fin_ay90h4_x100` (Xvfb 0.096) | `_ay90h4` (all three hairpins at safe speeds) + pub_gate + notch | 119.994 + 10 s | 100 / **8** | 7 of 8 ON THE STRAIGHT (s 3.9-8.7, 5-6 m/s), max|cte| 0.17-0.44 in the prior 1.5 s | **FAILED Gate C** and slower. Hairpin exits at safe speeds did not help, so the exit-speed story is dead too. **The notch is the only factor common to both failing 100-lap runs (5 and 8 straight collisions) and absent from both clean ones (hyb675b, hyb675b+pub_gate, 100/0 each)**; it had only 64 clean laps. Dropped from the candidate; ay90h2 without it is in Gate C (`race_20260918_214200`) |
| `fin_ay90h4nn_x100` (Xvfb) | `_ay90h4` (ay 9.0, all three hairpins at safe speeds, ideal 10.64) + pub_gate, no notch | **116.973 s + 0** | **100 / 0** | — | **passed Gate C**; only 0.14 s ahead of shipping+pub_gate (117.11) at this period, display run in flight (`race_20260918_222446`) |
| `fin_ay90h4_disp` | `_ay90h4` + pub_gate, display | **110.566 s + 0** | **32 / 0** | — | **WITHDRAWN** (a later 32-lap Xvfb control took a collision at s 26.8): display 110.57 (shipping 111.65), Xvfb 116.97 over 100 laps (117.11) (`race_20260918_224615`) |
| `fin_sched2_x32` (Xvfb, clean host) | hyb675b + pub_gate + `param_schedule finals_mesh_sched.json` | 117.914 s + 0 | 32 / 0 | — | **time-neutral live** (117.11 without); the harness's −0.20 s/lap did not transfer. An earlier pair of schedule runs was contaminated by a 28-core `check_pipeline` (dt p50 0.076 / p95 0.162) and is discarded (`race_20260918_231349`) |
| `fin_h4sched2_x32` / `fin_h4_x32b` (Xvfb) | `_ay90h4` + pub_gate, with / without the schedule | 116.353 + 0 / 116.406 + 0 (scored) | 32 / 0 and 32 / **1** | the control's hit: s 26.8, in +0.26 out −0.38 at the ay-9 speed of the s 24-27 corner | schedule neutral again; **`_ay90h4` is NOT promotable** (1 collision in 132 Xvfb laps) — the s 24-27 corner needs the safe speeds at 0.098 too, which leaves ~nothing of the ay-9 gain. The profile route is exhausted at the organisers' period (`race_20260918_232128`, `_232901`) |
| `fin_ayl16_x32` / `_disp` / `_a5_x32` | hyb675b + pub_gate + `ay_limit 16.0` (third: + `a_accel 5.0`) | Xvfb 117.788; display 112.605; Xvfb 118.245, all + 0 | 32 / 0 each | — | **REJECTED**: slower than shipping at both periods (117.11 / 111.65) where the harness said −0.33 / −0.46 s/lap over 108 laps per frame. **Fourth harness-predicted gain in a row that did not transfer (lead law, schedule, LMPC profile, steering cap): the harness no longer earns live slots for lap-time claims** (`race_20260918_233705`, `_234504`, `_235230`) |
| `fin_geod_x32` (Xvfb 0.093) | `finals_mesh_geod_ay90h1_race.csv`: hand-shifted 0.06 m inward on three exits + ay 9.0 | 120.548 + 70 s | 11 / **10** | s 27.96 off **+0.395 (LEFT)**, then 7 re-collisions wedged at the s 27 respawn; s 19.86 −0.389; s 44.5 | **REJECTED — the hand shift backfires in the S-bends**: the s 26-30.5 window runs into the following right-hander, where left is the OUTSIDE. Exit margin has to come from the optimiser with a one-sided corridor (agent, branch `exit-margin`), not from a lateral shift (`race_20260919_000056`) |
| image v3 (Sep 19 05:35 IST) | hyb675b + pub_gate + audit fixes (restart forgiveness, non-finite command guard, guarded callbacks, brake_mode validation, localization clock-reset / encoder-wedge fixes, thread-pinned tools) | Xvfb default **117.588**, Xvfb cmd **117.036**, display **111.869 s**, all + 0 | 12 / 0 each | — | **THE BANKED IMAGE.** A first attempt took a collision under a load-32 host (an agent's 63-thread optimiser: dt p50 0.073, max gap 0.35 s) and is discarded; scored chains now wait for load < 6 (`submission_20260919_05*`/`06*`) |
| `fin_ship_disp100` | shipping config (hyb675b + pub_gate), display | **111.375 s + 0** | **100 / 0** | — | the shipping config now has 100 clean laps at BOTH periods (`race_20260919_002638`) |
| `fin_exm10_x32` / `_disp` | **optimiser-built exit-margin geometry** `finals_mesh_exitm_m10t6_h2_race.csv` (one-sided +0.10 m corridor on three exits, ay 9.0, s 17.5-21.5 + 36-40 safe; ideal 10.27) | Xvfb 113.734 + 20; display **107.045** + 0 scored | 32 / **9** ; 32 / **1** | EVERY hit is the last hairpin exit (s 44.1-44.3, −0.22..−0.37 wide, two rebounds at s 0.3-0.5). Elsewhere the margin worked: wall min s 20-23 0.57, s 26-30 0.58 (Xvfb) | fastest runs yet at both periods (best lap 10.99 Xvfb / 10.59 display) with ONE failing corner. `_h3` blends the last hairpin back (s 42-44.7 + 0-2, ideal 10.48) — in flight (`race_20260919_004820`, `_005604`) |
| `fin_schedbo_x32` (Xvfb) | hyb675b + `finals_mesh_sched_bo.json` | 119.327 + 50 s | 15 / **10** | — | **REJECTED**: the harness showed 192/0 and −0.49 s/lap. Fifth harness prediction to fail live (`race_20260919_010311`) |
| `fin_exm10h3_x32` / `_disp` | **`finals_mesh_exitm_m10t6_h3_race.csv`**: exit-margin geometry, ay 9.0, safe speeds at s 17.5-21.5, 36-40, 42-44.7+0-2 (ideal 10.48) | Xvfb **114.990 s**; display **109.360 s**, both + 0 | 32 / 0 each (the display log's lap-32 event is the end-of-run stop: cmd and steering exactly 0 through the hairpin) | wall min anywhere: 0.35 m Xvfb, 0.25 m display | **BEST CLEAN CANDIDATE AT BOTH PERIODS** (shipping 117.11 / 111.38). In the 100-lap Xvfb gate (`race_20260919_010749`, `_011513`) |
| `fin_exm10h3_x100` / `_disp100` | exit-margin h3 line, BOTH 100-lap gates | Xvfb **114.158 s**; display **109.210 s**, both + 0 | **100 / 0 each** | — | **PROMOTED (Sep 19 07:30 IST)**: entrypoint `LINE` for finals_mesh, image v4 built (v3 kept as `:iros2026-finals-v3`). Shipping was 117.11 / 111.38 (`race_20260919_012258`, `_014335`) |
| image v4 (Sep 19 08:00 IST) | built image with the exit-margin h3 line | Xvfb default **115.143**, Xvfb cmd **114.353**, display **109.548 s**, all + 0 | 12 / 0 each | — | **THE BANKED IMAGE** (v3: 117.59 / 117.04 / 111.87, kept as `:iros2026-finals-v3`) |
| `fin_exm_h4_x32` (Xvfb) | exit-margin h3 with the FIRST hairpin at full ay 9 (`_h4`, ideal 10.37) | 114.085 s + 0 | 32 / 0 | — | clean, no gain over h3's 114.16 — extra apex speed at s 19 does not convert at 0.098. h5 / h6 half-steps not driven (superseded) (`race_20260919_021641`) |
| `fin_e18a075_x32` (Xvfb 0.093) | later-apex geometry `exitm2_e18n10`, last hairpin at 75 % of the ay-9 raise | 117.034 + 40 s | 26 / **10** | every hit the last hairpin exit again (s 44.25-44.35, −0.30..−0.39) plus a rebound next lap at s 0.3-0.7 | **REJECTED — the last hairpin takes no more than its safe speed at the organisers' period**, whatever the geometry (+0.10 m wall, apex R 0.637). The a10 rung was not driven (`race_20260919_022735`) |
| `fin_h3_tk40_x32` / `_tk34_x32` (Xvfb) | shipping h3 + `lead_mode time`, steering lead 4.0 / 3.4 sensor periods | 125.283 + 70 s ; 114.760 + 0 | 23 / **10** ; 32 / 0 | — | **steering lead CLOSED in both directions**: 4.0 periods (0.39 s) turns in far too early, 3.4 is neutral (h3 alone 114.16-114.99). With the Sep 19 2.8 result (slower at both) the shipping lead is at its optimum (`race_20260919_023603`, `_024232`) |
| `fin_e18a10_disp` / `_a075_disp` | later-apex geometry, last hairpin at 100 % / 75 % of the ay-9 raise, DISPLAY | 108.074 ; 108.202 (scored laps clean) | 32 / **4** ; 32 / **1** | every hit s 44.35-44.55, cte −0.38..−0.45, **with the steering AT FULL LOCK (cmd 0.98-1.00, rack 0.50-0.52 rad)** | **the last hairpin is capped by the steering lock, at BOTH periods**: its radius is within centimetres of the 0.561 m minimum turn circle, so any speed above the safe one needs more than full lock. Closed (`race_20260919_025107`, `_025846`) |
| `fin_h3_servo_x32` / `_disp` | shipping h3 line + **`servo_k_p 0.5 servo_head_max 0.3`** (exit-throttle servo) | Xvfb **113.623 s**; display **107.269 s**, both + 0 | 32 / 0 each | — | **NEW BEST CLEAN AT BOTH PERIODS** (h3 alone 114.16 / 109.21). The same arm took a collision at the first hairpin's exit on the OLD geometry (0.42 m exit margin); the exit-margin geometry (0.55 m) makes it safe. In the 100-lap Xvfb gate (`race_20260919_030650`, `_031419`) |
| `fin_h3_servo_x100` / `_disp100` | h3 + servo, 100-lap gates | Xvfb **114.718 s + 0** (median 11.382 vs h3 11.458); display 108.154 s (scored clean) | 100 / 0 ; 100 / **1** | the display hit: lap 86, s 26.7, cte −0.40 — the s 24-27 corner's exit, where the servo had thinned the wall margin 0.57 -> 0.20-0.23 m | **Xvfb gate passed, display gate FAILED by one lap.** `_h3c` holds s 24-28 at HALF the ay-9 raise (ideal 10.55; full hold `_h3b` 10.65 gives the servo's gain back) — in the display gate (`race_20260919_032153`, `_034239`) |
| `fin_h3c_servo_disp100` / `_x100` | `_h3c` (s 24-28 at half the ay-9 raise, ideal 10.55) + servo | display **108.815 s + 0**; Xvfb 115.834 + 10 | 100 / 0 ; 100 / **6** | Xvfb hits: s 25.6-26.7 (x4), s 30.2, s 38.8 — all 0.30-0.37 m WIDE RIGHT on a corner exit, right wall 0.33-0.36 m from the car centre | **display gate passed, Xvfb gate FAILED — and the SLOWER corner made it worse.** Holding the apex down makes the profile demand a steeper acceleration right after it, and the servo chases that while the car is still turning: running wide tracks EXIT ACCELERATION, not apex speed. Fix: opt-in `servo_head_kappa` (fade the servo headroom to zero as |kappa| under the car rises to the gate) — h3 + servo + gate 0.4 in the display gate (`race_20260919_040336`, `_042330`) |
| `fin_h3_servok_disp100` / `_x100` | h3 + servo + **`servo_head_kappa 0.4`** | display **108.373 s**; Xvfb **114.192 s**, both + 0 | **100 / 0 each** | — | **PROMOTED (Sep 19 11:30 IST)**: both 100-lap gates passed; +0.8 s on the display, level on Xvfb (h3: 109.21 / 114.16). Image v5 built, v4 kept as `:iros2026-finals-v4` (`race_20260919_044701`, `_050630`) |
| image v5 (Sep 19 12:00 IST) | built image: exit-margin h3 line + pub_gate + curvature-gated servo + audit fixes | Xvfb default **114.816**, Xvfb cmd **115.075**, display **108.466 s**, all + 0 | 12 / 0 each | — | **THE BANKED IMAGE** (v4 `:iros2026-finals-v4` 115.14 / 114.35 / 109.55; v3 `-v3` 117.59 / 117.04 / 111.87) |
| gated-servo ladder, display 32 laps each (`servo_head_kappa 0.4`) | `k_p 1.0 / head 0.3`; `k_p 1.0 / head 0.6`; `k_p 1.0 / head 1.0`; `k_p 0.5 / head 0.3 + e_cut_accel_scale 0 kappa 0.3` | 107.659 (median 10.814); 107.973 (10.768); 107.791 (10.759); 108.171 (10.817) | 32 / 0; 32 / 0; 32 / 0; 32 / **2** | the e_cut hits: one event, lap 30 at s 26.0 + rebound, clean sensor timing | gain 1.0 = gain 0.5 (the cap binds, not the gain); headroom plateaus at ~10.76 by 0.6 (shipping 0.3: 10.817). **Carry `k_p 1.0 head 0.6`** (-0.05 s/lap) to Xvfb. The exit speed-cut relaxation hits the s 26 exit again and gains nothing; one re-run queued at the user's request (`race_20260919_054126`, `_055820`, `_060530`, `_055051`) |
| `fin_sv_hd06_x32` (Xvfb) / `fin_sv_ecut2_disp` | gated servo `k_p 1.0 head 0.6`; exit-cut arm re-run on display | 115.903 + 0 (median 11.548) ; 108.005 + 0 (median 10.789) | 32 / 0 each | — | head 0.6 gains 0.05 s/lap on the display and LOSES 0.08 at 0.098 (shipping 0.3: 11.473) — shipping value stays. The exit-cut arm over 64 display laps: one collision event for 0.03 s/lap — not carried (`race_20260919_061256`, `_062036`) |
| `fin_h7_x32` / `fin_h8_x32` (Xvfb) | s 36-40 corner released to full / half ay 9 on the exit-margin geometry (ideal 10.26 / 10.36), shipping servo | 115.248 + 30 ; 113.557 + 0 scored | 27 / **10** ; 32 / **4** | — | **REJECTED**: that corner needs its safe speed at the organisers' period even with 0.6 m of exit wall (`race_20260919_062806`, `_063445`) |
| `fin_dl15_x32` / `fin_dl30_x32` (Xvfb) / `fin_dl15_disp` | shipping config + `servo_decel_look_m` 1.5 / 3.0 (Xvfb), 1.5 (display) | 114.502 (median 11.447); 114.396 (11.434); display 108.586 (10.859), all + 0 | 32 / 0 each | first hairpin s 18-20 on Xvfb: 1.24 -> 1.14 / 1.10 s | the gate repairs the first hairpin at 0.098 (+0.03-0.04 s/lap there) and costs the same on the display (shipping servo 10.817) — **a wash across the two periods; stays opt-in, v5 unchanged** (`race_20260919_064442`, `_065330`, `_070125`) |
| `fin_v86_x32` / `fin_v86_disp` | shipping config with the straight's speed cap raised **8.0 -> 8.6** (`finals_mesh_exitm_m10t6_h3v86_race.csv`) — the user's observation that the straight had room | Xvfb **114.252** (median 11.371, dt 0.092); display **107.400** (median 10.735), both + 0 | 32 / 0 each | — | **clean gain at both periods** (-0.08..-0.10 s/lap). On the straight the car sat at 7.95 of an 8.00 cap at 0.33 of 0.60 throttle; 8.6 is the most the 9.0 m/s encoder guard allows (wheel speed p99 8.96). Owes its 100-lap gates (`race_20260919_075350`, `_080136`) |
| `fin_pp_disp` / `fin_pp_x32` | v86 + **`straight_pp_k 0.5`** (straight-mode pure-pursuit blend; `straight_d 3.0 straight_kappa 0.10`) | display **107.228 + 0**; Xvfb 117.678 + 10 (dt **0.101**) | 32 / 0 ; 32 / **2** | display straight: rack std 0.0148 -> **0.0068 rad (-54 %)**, cmd std 0.0358 -> 0.0111 (-69 %). Xvfb hits: both on the straight, a heading error after the s 4 kink grows to cte +0.33 over 0.4 s, the `straight_e` release hands back to the MPC, `e_free` brakes at 7 m/s, overshoot into the far wall | **the first swivel fix that works on the display at no cost; too gentle at 0.10 s.** A slew limit was tried first and REMOVED (2-15 harness collisions in 6 laps: a rate limit adds lag). Firmer `k 0.3` + period-scaled dead time (`straight_pp_tk`) in flight (`race_20260919_083141`, `_083850`) |
| `fin_b40_x32` (Xvfb, dt **0.102**) | v86 line re-planned at brake 4.0 (ideal +0.07) | 115.700 + 0 scored | 32 / **1** | — | slower (median 11.582) and a collision, BUT run at a 10 % longer period than its baseline (0.092): **confounded, to be re-run in-band** (`race_20260919_084719`) |
| in-band Xvfb A/B (period 0.094-0.100 held by `RATE_MIN_DT/RATE_MAX_DT`) | v86 baseline (dt 0.098); `apex_f42k` + barrier 0.34 (dt 0.095 at check, 0.102 whole-run); v86 + `straight_pp_k 0.3 tk 1.7` (dt 0.095) | 115.066 (median 11.465); 114.205 (11.417); 116.012 (11.650), all + 0 | 32 / 0 each | apex wall min: s 26-30 0.28 (v86 0.58), rest >= 0.35 | apex line: ~-0.05 s/lap at the organisers' period while running a LONGER period (gain understated); firmer straight-PP: clean but +0.19 s/lap — at 0.098 s the baseline barely swivels (rack std 0.006-0.010 vs 0.015 on the display) and the PP law WANDERS 0.35 m and triggers the speed cut (`race_20260919_085901`, `_090635`, `_091619`) |
| `fin_apex_disp` / `fin_apexpp_disp` | `apex_f42k` (+ barrier 0.34) on the display, alone / + period-gated straight-PP (`straight_pp_period_max 0.075`) | 108.006 (median 10.796) / 107.791 (10.841), both + 0 | 32 / 0 each | — | the apexing line is a WASH: -0.05 s/lap at 0.098, +0.06 s/lap at 0.058 vs v86 (10.735). Period-gated PP is free on the display again. v86 stays the front-runner (`race_20260919_092859`, `_093614`) |
| `fin_v86pp_x100` / `_disp100` | **v6** = h3v86 line (straight cap 8.6) + period-gated straight-PP (`straight_pp_k 0.5 straight_d 3.0 straight_kappa 0.10 straight_pp_period_max 0.075`) + shipping servo | Xvfb 115.767 s (median 11.554, dt 0.099 with a 1.1 s gap ridden out, host load ~10); display 108.075 s (median **10.758**, v5 10.817), both + 0 | **100 / 0 each** | — | **PROMOTED — image v6 built** (v5 kept as `:iros2026-finals-v5`). Xvfb median is period/load-confounded; the 32-lap in-band v86 run had 11.371 at 0.092 s (`race_20260919_100453`, `_102721`) |
| image v6 (Sep 19 16:20 IST) | built image: h3v86 line + period-gated straight-PP + gated servo + pub_gate + audit fixes | Xvfb default 115.273 + **10** (1 collision lap 2 at s 26.7, the s 24-27 exit, 0.43 m wide at 3.2 m/s, dt 0.096); Xvfb cmd **115.434 + 0**; display **108.129 + 0** | 12 / 1 ; 12 / 0 ; 12 / 0 | — | **built, NOT yet preferred over v5** (`:iros2026-finals-v5`, 0 collisions in all its tests): the s 26.7 exit hits ~1 in 200 laps on this family. Insurance line `h3v86c` (s 24-28 at half raise) and the PI throttle are being driven; the choice of image follows (`submission_20260919_162108` etc.) |
| `fin_pi_x32` (Xvfb 0.096) | v6 + closed-loop PI throttle (`thr_pi_kp 3 ki 5 prefilter 0.15 slip 0.15`) | — | 4 / **12** | car reached **11.8 m/s** against a plan of 8.6, arrived at the first hairpin at 9.4 m/s | **REJECTED — a runaway.** The encoder estimator rejects wheel speeds above 9.0 m/s, so once the PI pushed past that it was blind to its own speed, read a deficit and wound up to full throttle. The surrogate does not model the guard (it had this arm clean and −0.13 s/lap). Any closed loop here must be bounded by a speed source the guard cannot blind (`race_20260919_111243`) |
| `fin_v86c_x32` (Xvfb 0.096 in-band) | `h3v86c` = v6 line with s 24-28 at half its raise (insurance for the s 26.7 exit) | 114.950 + 0 (median 11.523) | 32 / 0 | — | clean; 0.05 s/lap slower than v6 in-band (11.465), as its ideal predicted. Display run next (`race_20260919_111636`) |
| `fin_b50_x32` (Xvfb 0.095 in-band) | `apex_f42k_b50` (apexing geometry, s 36-40 at HALF the ay-9 raise, barrier 0.34; ideal 10.16) | 112.227 + 0 scored (median **11.244**, best 10.936) | 32 / **2** | s 25.1 (the tightest corner, −0.15) and s 38.5 (the released corner, −0.31 wide) | **fastest ever at the organisers' period by 0.2 s/lap**, not clean. Quarter release `_b25` next (`race_20260919_112931`) |
| `fin_v86c_disp` | insurance line `h3v86c`, display | 29 laps logged (monitor cap killed the logger), median 10.846 | 29 / 0 | — | clean; costs 0.09 s/lap on the display vs v6 (10.758) — about the expected value of the 1-in-200 s 26.7 collision it removes, with the variance gone (`race_20260919_113944`) |
| `fin_b25_x32` (Xvfb 0.096 in-band) | `apex_f42k_b25` (quarter release of s 36-40, barrier 0.34) | 115.148 + 10 (median 11.384) | 32 / **2** | both at s 25.4-25.7, the s 24 corner's EXIT, car only 0.13-0.17 m off the line | **the apexing family's structural weak spot is the s 24-27 exit** (b50 hit it too, and the audit called it the lap's tightest phase). Extending its right-margin window to 24.8 m changed the optimiser's line by 0 bytes (already inside), so it is not a margin the optimiser can add. Family parked for the image; ~1 hit / 20-30 laps there (`race_20260919_115527`) |
| `fin_f42k2_x32` (Xvfb 0.096 in-band) | `apex_f42k` plain (barrier 0.34) with the v6 controller set | **113.525 + 0** (median **11.361**) | 32 / 0 | — | −0.10 s/lap vs v6 in-band (11.465) at the organisers' period; +0.06 s/lap on the display. Now 64 in-band Xvfb laps + 32 display laps clean. In Gate C (`race_20260919_120336`) |
| `fin_f42k_x100` (Xvfb) | `apex_f42k` (barrier 0.34) + v6 controller set, Gate C | **113.482 s + 0** (median **11.338**, best 10.970) | **100 / 0** | — | **best 100-lap result at the organisers' period** (v6's 100-lap: 115.77 at a loaded 0.099 s; in-band 32: 11.465). Display gate running (`race_20260919_121531`) |
| `fin_f42k_disp100` | `apex_f42k` + v6 controller, Gate D | **107.818 s + 0** (median 10.810, best 10.633) | **100 / 0** | — | **PROMOTED — image v7** (v6 kept as `:iros2026-finals-v6`, v5 as `-v5`). vs v6: −0.13 s/lap at the organisers' period, +0.05 s/lap on the display — the organisers' box is the slow-period case, so this is the entry (`race_20260919_123636`) |
| image v7 (Sep 19 18:30 IST) | built image: apex_f42k line (barrier 0.34) + pub_gate + gated servo + period-gated straight-PP + audit fixes | Xvfb default **113.290**, Xvfb cmd **112.873**, display **108.031 s**, all + 0 | 12 / 0 each | — | **THE BANKED IMAGE** (v6 `-v6` 115.27+10/115.43/108.13; v5 `-v5` 114.82/115.08/108.47) |
| `fin_v7_slow_x32` (Xvfb **0.104 s**, Sep 19 19:10 IST) | v7 config at the SLOW END of the period lottery: the sim only draws >= 0.101 s under host load, so 12 nice-19 busy loops on cores 16-27 reproduced it (first draw 0.104, the slowest period ever seen here) | **114.813 + 0** (median 11.472, best 11.141) | 32 / **1** | — | the ten scored laps were clean; one collision on lap 28 at **s 28.7, 0.30 m outside the line, 3.97 m/s** — the s 24-29 exit that is this line family's weak spot (v6 hit s 26.7 once in ~200 laps). At 0.094-0.098 the same config is 100/0 twice; 0.104 is 7 ms slower than the organisers' estimate (`race_20260919_134019`) |
| `fin_v7_bms06_slow_x64` (Xvfb **0.103 s**, Sep 19 19:45 IST) | v7 + the new opt-in `barrier_margin_slow 0.06` / `barrier_slow_period 0.100` (corridor margin 0.34 -> 0.40 only when the measured period is past 0.100 s; surrogate: inert at 0.098, +0.05 s/lap at 0.104, clean) | 115.801 + **10** (median 11.535) | 64 / **9** | — | **REJECTED — and a geometry fact.** All nine hits are at **s 25.1-26.3 on the OTHER side** (offset −0.25 to −0.43 m, 1.6-3.4 m/s), where the v7 run hit once at s 28.7 on the outside. Half-width there is ~0.57 m and car half-width + 0.40 = 0.535 m, so the symmetric hinge leaves 3 cm of freedom and the solver swerves into the inside wall; the same 0.40 is fine on wider tracks (the drill's pick). None of the nine follows a sensor stall (five load-induced gaps of 0.2-1.1 s in this log, none within 40 rows of a hit). Verdict for the slow end stays: v7 at 0.34 = 1 wide collision in 32 laps at 0.104 s, versus 100/0 twice at 0.094-0.098; a one-sided runtime margin would be the honest fix and the barrier is symmetric (`race_20260919_135810`) |
| `fin_v7_mid_x64` (Xvfb **0.103 s** p50, p99 0.113; Sep 19 20:00 IST) | v7 itself (barrier 0.34, nothing added) at the slow band again, 6 busy loops | 114.995 + **10** (median 11.448) | 64 / **6** | — | **v7 is NOT clean at 0.103 s: all six at s 25.5-27.1 running 0.25-0.33 m WIDE on the exit of the s 24-25 left-hander (right wall; the line sits 0.59-0.62 m from it there), 3.0-3.6 m/s.** So the 9/64 above was the same failure, not the margin. Clean 100/0 twice at 0.095-0.096 p50 (p99 0.104-0.106); the failure needs p50 >= 0.103. The surrogate does not reproduce it at 0.103 or 0.106 (96 laps clean each) (`race_20260919_142056`) |
| `fin_v7_noservo_slow_x64` (Xvfb **0.105 s**) | v7 with the exit servo OFF (`servo_k_p 0`) | 115.322 + 0 (median 11.528) | 64 / **1** | — | **the servo is the multiplier**: 1/64 (s 25.6, wide, 3.2 m/s) against 6/64 and 9/64 with it, same corner. The servo pushes the throttle above plan where |kappa| falls under 0.4 — exactly the exit that runs wide at the slow period — and it gains nothing at 0.098 anyway. Fix: opt-in `servo_period_max` (fade the servo out past a measured period, 4 ms ramp), byte-identical below it (`race_20260919_144344`) |
| `fin_v7_spm_slow_x64` (Xvfb **0.104 s**) | v7 + **`servo_period_max 0.100`** (the servo fades out over 0.100-0.104 s; byte-identical below) | 115.043 + 0 (median 11.507) | 64 / **1** | — | reproduces the no-servo result (1/64, s 25.9 wide, 3.15 m/s) with the servo untouched at every gated period. **PROMOTED — image v8** (v7 kept as `:iros2026-finals-v7`). The residual 1-in-64 at 0.104 s is the exit itself; the insurance for it (`apex_f42m`/`f42n` outside-margin geometries, ideal 10.44 / 10.41; or `f42k_s25` speeds halved toward safe at s 23-27, 10.36) costs 0.8-1.5 s per race at every period and is not taken (`race_20260919_150059`) |
| image v8 (Sep 19 20:44 IST) | built image: v7 + `servo_period_max 0.100` | Xvfb default **113.201**, Xvfb cmd **113.232**, display **108.333 s**, all + 0 | 12 / 0 each | — | **THE BANKED IMAGE** (v7 `-v7` 113.29/112.87/108.03; v6 `-v6`; v5 `-v5`). Same pace as v7 at every gated period, and 6x fewer collisions at the slow end of the lottery |
| `fin_v8_brk60_x100` (Xvfb 0.096 in-band, Sep 19 21:45 IST) | v8 + `a_brake 6.0` + `brake_a_full 6.0` (the literature agent's find: the controller's own `a_brake` 4.0 in `brake_limited_target` was binding under the profile's 4.6 ramp; the brake probe says the plant holds ~6.3; surrogate −0.10 s/lap here) | 113.138 + 0 (median **11.296**, best 10.992) | 100 / **1** | — | **REJECTED**: the gain is −0.04 s/lap (11.338 -> 11.296), less than half the surrogate's, and the run took one collision on the MAIN STRAIGHT at s 4.5, 0.40 m off the line at 5.0 m/s (no braking zone there; the profile accelerates 7.5 -> 8.6). Whether it is the arm's or a 1-in-100 event of the base config cannot be told from one run, and 0.4 s/race is not worth another gate. `finals_mesh_f42k_lb60{,d25}_race.csv` (the 6.0 m/s2 ramps in the line) stay unshipped (`race_20260919_154056`) |
| `mpcc_x32` (Xvfb 0.096 in-band, Sep 19 22:20 IST) | **`mpcc`** — the overhaul agent's joint speed+steering MPC (state e_lat, e_yaw, delta, v; inputs steer rate + accel; soft yaw-rate 2.71 / ay 10.5 hinges; throttle 0.0 as the solver's own input bound), shipping flags + `mpcc_brake_a 2.5`; harness said −0.43 s/lap at 0.098, 300 laps clean | 116.726 + **10** (median **11.545**) | 32 / **2** | — | **REJECTED — the sixth harness-predicted gain to fail live.** 0.2 s/lap SLOWER than mpc3 (11.34) and both collisions at the s 25 exit (0.35-0.41 m wide at 3.0-3.2 m/s), the same corner the slow-period runs hit. Solve 11-20 ms live (mpc3 ~6). The harness's plant simply does not carry the exit-wide mechanism at any period (`race_20260919_160246`) |
| `fin_v8_s25_slow_x64` (Xvfb **0.103 s**, Sep 19 22:55 IST) | v8 config on **`apex_f42k_s25_race`** = the shipped geometry with the speeds pulled halfway to the safe reference over s 23-27 (`partial_blend` alpha 0.5; v 3.34/3.42/3.83 -> 3.10/3.16/3.55 at s 24/25/26; ideal 10.364 vs 10.282) | 115.543 + 0 (median 11.530) | 64 / **0** | — | **clean at the slow band** where v7/v8 took 1-9 per 64; only 0.02 s/lap slower than v8 there (11.507). Its in-band and display gates decide whether it ships (`race_20260919_162507`) |
| `fin_v8_s25_x100` (Xvfb 0.097 in-band, Sep 19 23:30 IST) | the same `apex_f42k_s25` line at the organisers' band, Gate C | 114.235 + 0 (median **11.434**) | 100 / **2** | — | **REJECTED, and the exit is closed.** Both collisions at **s 25.7-25.8 wide** (0.25-0.27 m, 3.2-3.3 m/s) — the exit the line was meant to protect — where v7/v8 are 100/0 twice at this band; and 0.10 s/lap slower. A slower apex (3.16 instead of 3.42 m/s) raises the EXIT ACCELERATION over s 25.5-27, and running wide there tracks exit acceleration, not apex speed (the Sep 19 finding, now confirmed the other way round). The 64/0 at 0.103 above was sample noise against v8's 1/64. Do not touch this corner's speeds; the only honest fix left is an exit-acceleration cap through s 25-27 (a combined-slip forward pass), not driven (`race_20260919_163950`) |
| `fin_v8_disp100` (display 0.058, Sep 20 00:05 IST) | v8 config, Gate D | **108.141 + 0** (median 10.841, best 10.585) | **100 / 0** | — | Gate D for v8 in its own right (the servo gate is inert here). Per-2 m breakdown of this and the Xvfb century: the car is AT its ideal time everywhere except the two hairpins, which carry the whole 0.58 s/lap (display) / 1.13 s/lap (Xvfb) over ideal — full steering lock for a metre past the plan's apex (the line's R 0.62 / 0.57 needs 0.48-0.52 rad), so the real apex is a metre late and the exit 1 m/s under plan (`race_20260919_170814`) |
| `fin_yc25_x32` (Xvfb 0.096 in-band, Sep 20 00:45 IST) | **open-hairpin geometry `apex_f42i`** (hairpins at R 1.01 / 0.88 instead of 0.62 / 0.57, steering 0.31 rad instead of full lock) with hairpin speeds set by the yaw-rate law at 2.5 rad/s (`yc25`: apex 2.2 m/s, ideal **9.694** vs 10.282), barrier 0.30 (its feasible margin is 0.305) | aborted at 10 laps | 10 / **4** | — | **FAILED, and the mechanism is the cliff itself.** Hits at both hairpins (s 43.9 x3 at 0.37-0.46 m wide, s 19.6 at 0.79 m wide). Through hp1 the car arrives at 2.75 m/s (plan 2.5), goes to FULL LOCK at 1.7 m/s and still runs 0.5-0.6 m wide of an R 1.0 arc: at 2.6 m/s a lock demand asks 4.9 rad/s, far past the 2.7-3.5 rad/s breakaway, the tyre lets go and the tracker asks for more. On the tight line the same lock demand at 1.8 m/s asks 3.2 rad/s and holds. The line's radius was never the limit; the controller's steering authority is not bounded by the cliff (`race_20260919_174701`) |
| `fin_yc23cap_x32` (Xvfb 0.095 in-band, Sep 20 01:25 IST) | `apex_f42i` open hairpins with the 2.3 rad/s profile (`yc23`, ideal 9.752) + the new opt-in **`yaw_rate_cap 3.5`** (steer <= atan(3.5 L / v)), barrier 0.30 | 115.218 + 0 (median **11.553**) | 32 / 0 | — | clean but 0.2 s/lap SLOWER than v8. The cap never binds below ~2.2 m/s (atan(3.5*0.324/1.6) > lock); the car still arrives at hp1 0.6 m/s over plan (3.42 vs 2.79 at s 18.0: the open line's brake zone is shorter and the 4.6 m/s2 plan is more than it sheds), turns in late and runs 0.64 m wide at full lock at 1.56 m/s, losing 0.78 s in s 18-20 alone. Next: same line with a 3.5 m/s2 brake plan so it arrives on speed (`race_20260919_181250`) |

All three DQ arms failed at **s 18.0-18.5, 0.35-0.56 m inside the line, at
1.0-1.5 m/s true speed** — crawling, and still steered into the inside wall.
The car is not arriving hot: it is **under** plan by 0.25-0.5 m/s into the entry.

With the brake on, the live entry cut is 0.05-0.07 m **better** than the harness
predicts, so the 0.058 s harness is pessimistic here — useful for ranking, not
for gating.

---

## 3b. The period-dependent corner bias — root cause and the opt-in fix (Sep 19, agent)

**Everything mpc3 uses to compensate dead time is a constant in seconds, but
every delay in this stack is a FRAME.** Measured pose-to-rack is `period +
0.045 s`, so at 0.058 s the loop's whole transport delay is 0.10 s and at 0.098
it is 0.14. The shipping readout lead is `nd * step_dt + 0.865 * period` =
0.230 s at 0.058 and 0.261 at 0.094 — i.e. **4.0 periods of anticipation at the
display rate and 2.8 at the Xvfb rate**. That is the sign flip: at 0.058 the
turn-in is applied too early and the car runs **+0.120 m inside** the line at
s 37.5-39 (`race_20260918_162222.csv`), at 0.094-0.098 it is applied late and
runs **-0.010 p50 / -0.23 p5 (wide)** and hits the wall
(`race_20260918_173633.csv`). Live, measured on the same band.

**The bigger half of it is the THROTTLE lead, not the steering.**
`rate_adaptive`'s `thr` term scales `thr_lead_s` with the period (0.126 s at
0.058, 0.212 at 0.098), so at the organisers' rate the speed plan's brake
points are read ~0.3 m further ahead: the car brakes earlier and arrives
0.3-0.6 m/s slower at every corner. Live hairpin passage on
`finals_mesh_hyb675b_race.csv` (display `race_20260918_191411`, Xvfb
`race_20260918_185202`, both clean): s 18-21 1.340 -> 1.391 s with apex speed
1.63 -> 1.37 m/s and the brake on 9 % -> 19 % of samples; s 36-39 0.992 ->
1.036 with entry speed 3.86 -> 3.26; s 42-44 0.641 -> 0.781 with entry 3.46 ->
3.19. Same apex and exit speeds — the loss is all on entry.

**Two harness defects were found and fixed on the way** (`tools/sim_tracker.py`):

* the plant's `pose -> rack` delay is `period + EXTRA_DEAD` = `period + 0.090`,
  **0.045 s longer than the measured live law**; run with `--plant-dead 0.045`
  and the harness's absolute corner offset lands on the live one. This is why
  the harness "predicted collisions at the wrong period".
* the bookkeeping test `abs(t / DT - round(t / DT)) < 1e-6` stops firing past
  t ~ 335 s of float accumulation, so **every `--laps 36` cell silently
  measured only 25-27 laps** and then drove on uncounted. Now an integer step
  counter; cells before 335 s are byte-identical.

**The fix is opt-in, defaults byte-identical** (verified with `cmp` on
`--dump-rec`): new `lead_mode:=time`, which reads the published angle
`lead_t0 + lead_tk * period` seconds ahead (floored, as ever, at
`nd + apply_floor_steps`), plus the existing `delay_tick` rate term with
`dead_tick_c:=0.0` (the actuator model's dead time becomes the period) and a
**pinned** `thr_lead_s:=0.126` with `thr` dropped from `rate_terms` (the
throttle lead stops scaling). Harness, 36 laps/cell, corrected plant:

| line | frame | arm | laps/coll | lap median | s 37.5-39 | s 17.5-19.5 | s18-21 | s42-44 |
|---|---|---|---|---|---|---|---|---|
| hyb675 | 0.058 | shipping | 36 / 0 | 12.610 | +0.284 | +0.193 | 1.600 | 0.930 |
| hyb675 | 0.094 | shipping | 36 / 0 | 13.090 | +0.112 | +0.061 | 1.670 | 1.080 |
| hyb675 | 0.098 | shipping | 36 / **1** | 13.150 | +0.127 | +0.088 | 1.680 | 1.120 |
| hyb675 | 0.058 | **fix** | 36 / 0 | **12.370** | +0.114 | +0.114 | 1.680 | 0.900 |
| hyb675 | 0.094 | **fix** | 36 / 0 | **12.550** | +0.235 | +0.263 | 1.600 | 0.880 |
| hyb675 | 0.098 | **fix** | 36 / 0 | **12.480** | +0.218 | +0.301 | 1.510 | 0.820 |
| ay 7.5 | 0.058 | shipping | 36 / **1** | 12.260 | +0.347 | +0.190 | — | — |
| ay 7.5 | 0.094 | shipping | 36 / **1** | 12.920 | +0.214 | +0.075 | — | — |
| ay 7.5 | 0.058 | **fix** | 36 / 0 | **12.160** | +0.246 | +0.120 | 1.720 | 0.890 |
| ay 7.5 | 0.094 | **fix** | 36 / **1** (s 0.1) | **12.110** | +0.301 | +0.265 | 1.450 | 0.840 |

The period gap in lap time closes from 0.48 to 0.18 s/lap, the hairpin passage
at the organisers' rate goes 1.680 -> 1.510 (s 18-21) and 1.120 -> 0.820
(s 42-44), and the s 38 corner stops running wide at the slow rate (p5 +0.038
against +0.003). **The cost is margin**: min footprint clearance 0.112 -> 0.056
at 0.094, and the s 19 entry cut GROWS at the slow rate (+0.088 -> +0.301),
because the car now arrives at the display's entry speed. This is a
speed-gaining change and needs Gate C (100 Xvfb laps) before it can ship.

## 4. Measured and closed (do not re-open)

**Longitudinal envelope — spent.** The whole reprofile grid (vmax 8-12 × brake
4-7 × ax 6.5/7.1) spans **0.48 s** of ideal lap, and the honest ideal with a
tapered ax (5.3 − 0.273·v) is **11.5 s**, not 11.0. Harness at 0.058 (144-lap
cells): frozen profile 0 collisions; brake 5.5 −0.14 s/lap but **2 collisions at
s 37.5** (a new corner, kappa ≈ 1.0); brake 7.0 unphysical (section 2); ax 7.1 or
vmax ≥ 9 take 1-6 collisions per 36-108 laps. Any extra speed anywhere finds a
wall somewhere. One rung is still worth a live slot if one is free:
`lon_v8b55a65` (*model* −1.4 s/race at 0.058, −2.5 at 0.094, against ~+1.4 s of
expected penalty). Lines are committed under `finals_mesh_lon_*`.

**Speeds above 9 m/s — dropped.** The plant tops out at 7.9 m/s on the vmax-8
profile and 8.7 m/s when the plan asks 11.2; the straights are too short for the
~4.5 m/s² (decaying) drive ceiling. The fraction of samples where the encoder
would exceed 9.0 is ≤ 0.1 % in every harness cell, so `SPEED_MAX` and
`max_step_mps` never bind — and raising them adds a real hazard (above 9 the
estimator freezes then clamps, so the brake trigger fires late at the fastest
point). No code change.

**The Porto `cr_dk0.8` trick does not transfer.** Every `--dkappa-max ≤ 1.0` line
here has 0.500 m min half-width, and the best of them took **9 collisions in 18
laps** at 0.058 with clearance 0.000. The Sep 17 `finals_mesh_dk08` run is the
live version of the same result (4 collisions): the refinement lowered the
corner's peak kappa 1.60 → 1.40 but `|dkappa/ds|` stayed 4.77 against a requested
0.8 (25 passes gave the identical line) — the 0.53 m half-width floor leaves no
room to move.

**`w_veh` is inert here.** The 0.53 m floor binds on all 8 rungs (w_veh 0.36-0.66
give the same line).

**The "rapid rights and lefts on a straight" are not in the line.** The
"3.5-6.3 sign changes / 10 m" figure was rounding dust of a geometrically dead
straight (|kappa| ≤ 0.006 over s 2-15; 42 % of masked samples under 0.02). The
driven steering on the true straight is a 0.033 rad limit cycle plus the early
turn-in bias from s 16 (+0.096 rad where the line asks 0.03). A flatter line
(Porto's 2.4/10 m) drives **1.2 s/lap slower with MORE steering**, and the safe
line is worse still. `tools/smooth_straights.py` exists as an opt-in post-pass
and is **not** recommended. What the eye sees is the s 19 entry.

**Steering-side knobs — ruled out** (harness, 144-lap cells): `apply_lead_steps`,
`nd_delay_s`, `steer_tau` 0.02, `barrier_margin` 0.30 / 0.34 / 0.44 all leave the
cut at +0.45 and collide at 0.058; `apply_ahead_s` 0 is catastrophic; removing
the barrier entirely helps a little (12 → 2 collisions) and is not a config.
Live: `barrier_margin` 0.34 vs 0.40 indistinguishable (keep 0.40, the pick's
value, feasible cap 0.385); `steer_tau` 0.02/0.03 no gain and worse clearance
(keep 0.04).

**Cliffs to leave alone:** `e_free` 0.25 → 10 collisions/36 and `k_e` 1.0 → 5 —
the defaults 0.18 / 2.0 sit on a cliff; `k_e` 3.0 buys +0.025 m for +0.10 s/lap
and does not stack with an ay rung (1/18). `ay_budget` 6.5: no gain. `w_lat` 16
collides and 36 costs 0.08 s — keep 24.

**`thr_brake_lead_s` 0.08 is not a no-op here** (`pick_config`'s "never 0.08" is a
0.097 s artefact): harness 0.0 → 8 collisions/36 at 0.058, 0.05 → 0/36 at 0.050 m
clearance, 0.08 → 0/36 at 0.135-0.146 m for +0.37 s/lap. It was nevertheless
**rejected live** (section 3): with the brake on it buys no margin at all.

**Checkpoint / gate constraints are unnecessary on this track.** The shipping
line already passes them; imposing them costs **0.56 s of ideal lap for nothing**.

**Retrace of the Porto 89.6 → 69.2 s path, re-applied here.** Of the ~20 s gained
on Porto: ~2.3 s dead-time compensation, ~7.2 s the mesh map + LiDAR offset + the
ay ladder, ~4.6 s mpc3 + the speed-cut rescale (`e_free`/`k_e`), ~3.7 s the wide
line family, ~2 s `rate_adaptive` + the dkappa line, ~1 s the display
smoothing/barrier work. **Everything controller-generic already ships here**;
every per-track knob was re-measured above.

## 4b. LMPC-style learning from our own laps (Sep 19, agent, branch `lmpc`)

Harness only — nothing here has been driven. Two things were built, one worked.

**The papers, in five lines.** Rosolia & Borrelli's LMPC (arXiv:1609.01387,
1610.06534, 2005.00826) improves a repeated task by keeping every previously
*feasible* lap as a sampled safe set and a value function of realized
cost-to-go, and constraining the next solve to end inside it — the guarantee is
"never plan what no past lap achieved". Their own tractable version replaces the
point set with a **convex hull of nearby stored points**, i.e. an interpolation
across laps at the same progress. Kabzan et al. (RA-L 2019) learn a GP residual
on the **lateral/yaw** slice only, keep it in a small dictionary, and use its
variance to *tighten* constraints rather than to license aggression. The one
paper that maps almost 1:1 onto a fixed offline CSV profile is
arXiv:2606.10733 (FSD, 2026): per track position, measure a stability margin,
step a bounded scale factor toward a target margin, smooth it spatially, and
multiply it into `v = sqrt(scale * ay / kappa)`. **What does not transfer:** the
online terminal safe set, the terminal cost, and the non-decreasing-performance
theorem — we have no per-step solve against a safe-set constraint, a bang-bang
brake and ~10 Hz sensing. **What does:** worst-case-over-laps margin per `s`,
a bounded monotone gain, spatial smoothing, the friction-ellipse coupling, and
the standing warning that a per-segment raise changes the ENTRY state of the
next segment (so it needs a whole-lap re-check, not per-segment margin math).

### `tools/learn_profile.py` — profile learning, worth ~0.05 s/lap

Geometry untouched; only `vx`/`ax` move. Per 1 m bin of `s`, pooled over the
clean laps: worst-body-corner wall distance p2 (the safe-set boundary), driven
|offset| p95, driven/planned speed p50 (the feasibility gate — raising the plan
where the car is already under it buys nothing here, measured many times), brake
duty and driven |ay| p95. A bin is raised only if **margin > `m_lo`, the car
tracked the plan, the driven offset is small, and it is not within 3 m of a
collision in a FAILED iteration** (LMPC's discarded iterations, kept as hard
evidence: `race_20260918_173633.csv` puts a veto band over s 35.6-42.1). Raises
are Gaussian-smoothed; cuts are *eroded* first so smoothing cannot dilute the
one bin that came close to a wall. Then the usual consistency passes, plus a
**guard pass**: a raise must be sheddable at 3.2 m/s² before the next un-raised
bin, because the plan's own 4.6 m/s² backward pass is not what the closed loop
delivers, and without it the raise arrives as entry speed at the next corner.

Inputs: the four clean logs (`race_20260918_185202`, `_194320` at 0.098 s,
`_191411`, `_204004` at 0.058 s — 35 591 samples), veto from `_173633`.

| line | ideal | 0.058 laps/coll | med | 0.098 laps/coll | med | min clear 0.058 | s38 wall p2 0.098 |
|---|---|---|---|---|---|---|---|
| `hyb675b` (shipping) | 10.836 | 72 / 0 | 12.48 | 72 / 0 | 12.88 | 0.035 | 0.159-0.177 |
| `lmpc_a` α 0.08, no guard | 10.661 | 36 / **1** | 12.42 | 36 / 0 | 12.87 | 0.000 | 0.202 |
| `lmpc_b` α 0.12, no guard | 10.599 | 36 / **3** | 12.38 | 36 / 0 | 12.93 | 0.000 | 0.152 |
| **`lmpc_c` α 0.12 + guard 3.2** | **10.702** | **72 / 0** | 12.46 | **72 / 0** | **12.83** | 0.025 | 0.202 |
| `lmpc_d` α 0.10 + cuts β 0.10 | 10.879 | 36 / **3** | 12.53 | 36 / 0 | 12.97 | 0.000 | 0.135 |
| `lmpc_e` α 0.18 + guard | 10.634 | 36 / **3** | 12.44 | 36 / 0 | 12.77 | 0.000 | 0.201 |
| `lmpc_f` α 0.12 + guard, vmax 8.6 | 10.663 | 36 / **1** | 12.37 | 36 / 0 | 12.81 | 0.025 | 0.185 |

`pick_config finals_mesh --line finals_mesh_lmpc_c_race.csv` is unchanged from
`hyb675b`: checkpoint margin 0.650 m, gate separation 0.781 m, occupancy 1,
body clearance 0.388 / effective 0.376 m.

**Three things the data said, all of them limits:**

* Most of the demonstrated margin is on the **straight**, and the straight is
  `vmax`-capped at 8.0 — so most of the learned raise has nowhere to go.
  Lifting `vmax` to 8.6 (`lmpc_f`) collides at 0.058.
* Every raise that is not guarded shows up as **entry speed at s 37.5**, which
  is where `lmpc_a/b/d/e` collide. This is exactly the per-segment-optimism
  failure the literature warns about; the guard pass is the fix and it costs
  most of the ideal-lap gain (10.599 -> 10.702).
* **Cuts do not pay.** 264 clean laps at a measured 0.15-0.23 m margin in the
  s 24-26 and s 42-44 bands mean a cut there is a prior, not evidence; turned on
  (`lmpc_d`) it moves the brake points and collides. `--beta 0` is the default.

Net: `lmpc_c` is the only arm clean at both frames — **-0.05 s/lap at 0.098 and
-0.02 at 0.058**, ~0.5 s/race at the organisers' period, for 0.010 m of min
clearance at 0.058. It owes the usual two gates.

### `steer_tau_k` — the rack IS period-dependent, and it is small

`tools/steer_residual_id.py` fits `steer_fb(t) = lag(MAX_STEER*cmd(t-Td), tau)`
from a race log. On the four clean logs, plus a low-passed cross-correlation as
an independent estimator, the **publish -> rack effective lag** is

| period | Td | tau | gain | effective lag (2 estimators) |
|---|---|---|---|---|
| 0.058 s (×2 logs) | 0.050 | 0.040 | 0.958 | **0.0846 / 0.0852 s** |
| 0.098 s (×2 logs) | 0.095 | 0.005 | 0.989-0.991 | **0.1047 / 0.1051 s** |

i.e. `0.056 + 0.50*T`, reproducible to 0.001 s. The `Td`/`tau` split is
quantised by the sensor period and should not be read literally; the effective
lag is robust. mpc3 models the same chain as `cmd_dead_time` 0.135 +
`steer_tau` 0.04 = **0.175 s constant** (`delay_tick` is not in the shipping
`rate_terms`), so the model is 0.090 s too slow at 0.058 and 0.070 s too slow at
0.098 — the part `steer_tau` should carry SHRINKS by 0.020 s over the span, i.e.
**`steer_tau_k = -0.5`**.

New opt-in `steer_tau_k` / `steer_tau_ref` (defaults 0.0 / 0.058, verified
byte-identical with `cmp` on `--dump-rec` against HEAD). It touches only the
rollout's and the compensator's rack model — **not the readout index**, so it
does not re-open the late-turn-in that killed `lead_mode:=time` and the
throttle-lead pin.

Harness (`finals_mesh_hyb675b_race.csv` and `_lmpc_c_race.csv`, shipping args,
12 laps x seeds, corrected plant):

| arm | 0.058 laps/coll | med | 0.098 laps/coll | med | s 38 wall p2 at 0.098 |
|---|---|---|---|---|---|
| `steer_tau_k` 0 (shipping) | 72 / 0 | 12.48 | 72 / 0 | 12.88 | 0.159-0.177 |
| `steer_tau_k` **+0.5** | 36 / 0 | 12.44 | 36 / 0 | **13.02** | 0.152 |
| `steer_tau_k` **-0.5**, shipping line | 72 / **1** | 12.46 | 72 / 0 | **12.825** | 0.214 |
| `steer_tau_k` **-0.5**, `lmpc_c` | 72 / **2** | 12.41 | 72 / 0 | **12.745** | 0.202 |
| `steer_tau_k` **-0.25**, shipping line | **72 / 0** | 12.470 | **72 / 0** | **12.830** | 0.202 |
| `steer_tau_k` **-0.25**, `lmpc_c` | 72 / **3** | 12.47 | 72 / 0 | **12.760** | 0.183 |

The sign the measurement implies is the one that goes faster. `+0.5` (the sign
you get by wrongly assuming `cmd_dead_time` already scales with the period) is
simply slower — do not re-derive it. `-0.5` is worth 0.06-0.09 s/lap at the
organisers' period and takes 1-2 collisions per 72 laps at 0.058, so it is out.
**`-0.25` on the shipping line is clean at both frames** (72/0 each) for
-0.05 s/lap at 0.098 and the s 38 wall p2 0.177 -> 0.202 m — the same gain the
`lmpc_c` profile buys, from a different mechanism.

**They do NOT stack.** `lmpc_c` + `steer_tau_k -0.25` takes 3 collisions in 72
laps at 0.058 (and `-0.5` takes 2): both arms spend the same display margin at
the s 37.5-38 corner, and together they spend more than there is. Pick one.

### How to regenerate, and the live lines

```bash
# the shipping arm (exactly the committed finals_mesh_lmpc_c_race.csv)
.venv/bin/python tools/learn_profile.py \
    data/racelines/finals_mesh_hyb675b_race.csv --map data/maps/finals_mesh \
    --log data/logs/race_20260918_185202.csv --log data/logs/race_20260918_194320.csv \
    --log data/logs/race_20260918_191411.csv --log data/logs/race_20260918_204004.csv \
    --veto-log data/logs/race_20260918_173633.csv \
    --bin 1.0 --cte-veto 0.20 --beta 0 --m-lo 0.25 --alpha 0.12 --report \
    --out data/racelines/finals_mesh_lmpc_c_race
.venv/bin/python tools/steer_residual_id.py data/logs/race_2026091*.csv
```

Live, in expected-value order. **1 and 2 are alternatives, not a stack** —
together they collide at 0.058 in the harness.

```bash
# 1. lmpc_c profile, everything else the shipping config. Xvfb gate first
#    (Gate C, 100 laps at ~0.097 s), then 32 display laps.
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=100 \
  ./scripts/run_exp.sh lmpc_c_x100 lmpc_c_race \
  -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p steer_tau:=0.04 \
  -p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05 \
  -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6 \
  -p pub_gate_corr:=0.05 -p weave_notch_k:=0.227 -p weave_notch_q:=0.7 \
  -p straight_d:=3.0 -p straight_kappa:=0.10
# then, on the real display:
VISIBLE=1 SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 \
  LAPS=32 ./scripts/run_exp.sh lmpc_c_disp lmpc_c_race <same -p flags>

# 2. ALTERNATIVE to 1, same expected gain, different mechanism: the rack
#    schedule on the SHIPPING line (harness 72/0 at both frames). Do NOT add
#    it on top of lmpc_c_race -- that pairing collides at 0.058.
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=100 \
  ./scripts/run_exp.sh tk25_x100 hyb675b_race <same -p flags> -p steer_tau_k:=-0.25

# 3. Only after 1 or 2 has both gates: re-test the OTHER one stacked at a
#    smaller gain (-p steer_tau_k:=-0.10) on whichever line shipped. Abort on
#    the first collision at either period.
```

**The honest reading: the rack carries only 0.020 s of period dependence,
against the readout lead's 0.031 s swing — so it is a small lever, and what it
buys is spent at the same corner the profile spends.** `steer_tau_k -0.25` and
the `lmpc_c` profile are worth the same 0.05 s/lap at 0.098 and cannot be run
together. Unlike the rejected lead laws it never touches the readout index, so
it does not re-open the late turn-in; it needs the same two gates.

---

## 4c. A log-fitted surrogate of the simulator (Sep 19, agent, branch `rl-track`)

**Nothing here has been driven, and nothing here changes the image.** This
section is about the offline harness itself: why five of its predictions failed
live, what was actually wrong with it, and how far a plant identified from the
logs gets before it, too, runs out.

### Why this was started

The plan was reinforcement learning. The simulator runs at wall-clock 10-17 Hz
with no headless mode, so RL cannot be trained in it, and a policy trained on a
*non-predictive* surrogate is worthless. Section 3 records five harness-called
gains in a row that did not transfer (lead law, s-indexed schedule, BO
schedule, LMPC profile, `ay_limit 16`) and a harness that called which sensor
period fails backwards. So the first job was to find out whether a surrogate
can be made predictive at all. The answer is a qualified yes, and the
qualification is the interesting part.

### What was wrong with the harness — four measured defects

All four were identified from the two shipping 100-lap logs
(`race_20260919_044701` at 0.058 s, `race_20260919_050630` at 0.094 s) using
**ground truth only** (IPS `x,y` differentiated, the `steering` rack feedback,
the `throttle` in force). The fixes are opt-in behind `--plant-fit`; the
defaults are verified byte-identical with `cmp` on `--dump-rec`.

1. **The encoder channel — the big one.** The harness modelled the speed the
   bridge reports as a 0.193 s first-order lag on a wheel speed 3.9 % above
   ground truth. Measured, the live channel tracks truth with **one sample of
   delay** (correlation against truth peaks at lag 1-2 samples, 0.985/0.986,
   rms 0.32-0.34 m/s, mean bias +0.008 / −0.116) and a quasi-steady slip of
   **1.020 / 1.003**. The modelled lag over-reads by up to **1.43 m/s** in a
   braking zone where the live channel stays inside 0.55, and `brake_mode:=zero`
   then keeps the brake on for 4 m where the live car keeps it on for 1.5 —
   then the deeper deceleration makes the modelled lag worse still. This single
   defect was most of the harness's corner-entry pessimism.
2. **The lateral law had a knee that does not exist.** The plant clipped
   achieved lateral acceleration to 6.5 m/s² through an exponential knee at
   5.5. Measured delivery ratio (achieved path curvature over the curvature the
   rack asks for, 18 187 samples with |kappa| > 0.15 and v > 1.5, the rack
   passed through the identified 0.043 s lateral pole first):

   | asked ay | 2-4 | 4-6 | 6-8 | 8-11 |
   |---|---|---|---|---|
   | ratio | 1.019 | 0.964 | 0.933 | 0.872 |

   No knee, and **no dependence on longitudinal acceleration** below ay 8
   (0.93-0.97 flat across every ax band). The fitted law is the affine
   `r = 1.050 − 0.0180 · ay_asked`, clipped to [0.78, 1.05], with a 0.043 s
   first-order pole. The old law under-delivered 5-8 % of curvature in every
   hairpin.
3. **The rack's dead time is one constant plus the frame quantum.**
   `steer_residual_id.py`'s `0.056 + 0.50·T` is exactly what a fixed 0.056 s
   plant delay gives once the command is latched at a frame boundary, which
   already costs half a period. `--plant-dead 0.056`, and the period dependence
   falls out instead of being fitted.
4. **The brake at throttle 0.0 is 4.15 + drag, not 4.6 + drag.** Measured
   decel with the throttle at exactly 0.0: 4.99 (v 2-3), 5.07 (3-4), 5.29
   (4-5), 5.74 (5-7) m/s².

And the pose pipeline, which the harness had roughly right in shape and wrong
in detail. Measured est−truth **at the pose's own stamp**, body frame,
identical at both periods: along-track **+0.100 m**, pose age exactly one
sensor period, and a lateral error that is **heteroscedastic in curvature** —

| \|kappa\| | mean (signed toward the inside) | std |
|---|---|---|
| < 0.2 | −0.002 | 0.009 / 0.010 |
| 0.5-1.0 | −0.017 .. −0.023 | 0.021 / 0.030 |
| > 1.0 | −0.040 .. −0.059 | 0.026 / 0.057 |

i.e. almost noiseless on the straight, and in a hairpin 4-6x the scatter plus a
systematic **outward** offset. Yaw error is 0.009 rad std on the straight and
0.012-0.019 in a hairpin with a −0.017 rad outward mean; the harness had it at
zero. The sensor period jitters ±0.0015 s, and 0.50 % of frames at 0.058 s are
longer than 1.5x the median (p999 0.163 s) against 0.02 % at 0.094.

One harness gap was fixed on the way: **`MPC3Sim` never supported
`servo_head_kappa`**, so the harness could not run the shipping config at all.

### Test (a): pace. It passes.

`tools/surrogate_race.py --compare-log` puts the surrogate's per-2 m segment
times next to a live log's, median over laps. Shipping config, both held-out
100-lap logs, 300 surrogate laps each:

| | lap, live | lap, surrogate | per-bin \|error\| p50 / p95 / max |
|---|---|---|---|
| display 0.058 s | 10.815 s | **10.890** | 0.017 / 0.097 / 0.111 s |
| Xvfb 0.094 s | 11.483 s | **11.450** | 0.022 / 0.137 / 0.281 s |

Before the fit the same cells were **+1.4 s/lap** slow, all of it in the corner
entries (s 36-38 alone was +0.43 s). The cross-track distribution matches too:
|e| p95 0.221 against live's 0.227, max 0.475 against 0.459, and the per-4 m
bands agree to 0.03-0.08 m almost everywhere.

### Test (b): live verdicts. It reproduces 8-9 of 15 cells, and it is ONE-SIDED.

*(The table below is the first run, on the plant before the breakaway cliff and
the two ported parameters; 4c.2 has the final one, which is 8 / 15 with the
same zero false collides.)*

`tools/surrogate_validate.py`, 100 laps x 3 seeds per cell, every live outcome
from section 3 and none of it used in the fit. An arm "collides" if it takes at
least one collision per 100 laps, which is the live gate.

| arm | period | surrogate | live | agree |
|---|---|---|---|---|
| shipping (h3 + pub_gate + gated servo) | 0.058 / 0.097 | clean / clean | clean / clean | **YES / YES** |
| `_h2` (last hairpin at ay 9) | 0.058 / 0.097 | collide (s 42.7-43.2) / collide | collide / collide | **YES / YES** |
| `_h7` (s 36-40 released, full) | 0.097 | clean | collide (10/27) | NO |
| `_h8` (s 36-40 released, half) | 0.097 | clean | collide (4/32) | NO |
| `_h3c` + ungated servo | 0.058 / 0.097 | clean / clean | clean / collide (6/100) | **YES** / NO |
| ungated servo on h3 | 0.058 / 0.097 | clean / clean | collide (1/100) / clean | NO / **YES** |
| `ay_limit 16` | 0.058 / 0.097 | clean, **+0.38 / +0.35 s/lap** | clean but slower | **YES / YES** |
| `lead_tk 4.0` | 0.097 | collide (153/300) | collide (10/23) | **YES** |
| weave notch | 0.097 | clean | collide (5 and 8 per 100) | NO |
| `finals_mesh_sched_bo.json` | 0.097 | clean | collide (10/15) | NO |

**Confusion, cells = arm x period: 3 true collide, 6 false clean, 6 true clean,
0 false collide. Agreement 9 / 15.** The user's bar for starting RL was six of
the nine named arms; by arm this gets four fully right (shipping, `_h2`,
`ay_limit 16`, `lead_tk 4.0`) and two half. **The bar is not met, so no policy
was trained.**

The failure is not random, and that is what makes the tool usable: **the
surrogate never invents a collision.** Every cell it calls "collides" collided
live, including getting `_h2`'s at BOTH periods and `ay_limit 16`'s *lap-time*
loss at both periods right. What it cannot do is clear an arm.

### Why it cannot clear an arm — the specific, measured gap

The surrogate reproduces the median and the p95 of cross-track error and not
the rare wide excursion that actually hits the wall. At s 36-40 on the shipping
config at 0.094 s, live reaches **−0.345 m** (5 events in 100 laps); the
surrogate reaches **−0.102 m** in 300. The live events are unambiguous in the
log: at s 38 the car is on the line at 2.35 m/s, the controller commands
0.42-0.47 rad of rack against a median 0.32, the asked ay is 8-9 m/s² while the
car is accelerating at ~2.3 m/s², and the car then runs 0.32 m wide in 0.5 s —
it is not delivering anything like the curvature it is asking for.

Five candidate mechanisms were built and measured, and **none of them produces
that tail**. Recorded so nobody re-runs them:

* **Friction-ellipse coupling.** It is real but thin: at asked ay 8-10 the
  delivery ratio is 0.922 below ax 1.5 m/s² and 0.744-0.825 above it, on 82
  samples, with no effect at all below ay 8. Implemented as opt-in
  `--plant-grip-c` (default 0). Turning it on changes the s 36-40 minimum by
  0.00 m and makes the verdict table *worse* (`_h2` at 0.097 goes clean),
  because a car with less grip simply drives more conservatively.
* **Sensor-period jitter** (0.002, 0.006 s), **publish latency and jitter**
  (10/6 and 20/12 ms), **dropped frames** (one per 11.6 s and per 30 s), and
  all of them together: lap time stays within +0.02..+0.07 s and the s 36-40
  minimum stays between −0.06 and −0.14 m. None reaches −0.345.
* **A stochastic grip residual.** The fractional delivery residual has std
  0.099 / 0.159 but is essentially **white** (autocorrelation 0.35 / 0.02 at
  lag 1 and ~0 beyond), so it is mostly the 10 Hz IPS differentiation, and
  injecting white curvature noise displaces the car by millimetres.

The honest reading is the one `tools/lateral_id.py` already reaches from the
other direction: **the car has never been asked to corner hard at speed on
track** (0 quasi-steady samples above 4.5 m/s with more than 0.15 rad of
steering, 28 in the whole corpus above 0.33 rad), so the limit behaviour that
produces these excursions — transient sideslip at the front axle — is not
identifiable from racing laps, and a kinematic bicycle with a delivery ratio
cannot express it. Closing this needs the sysid staircase in the
`2026-iros-explore` image (`ros2 run pit_crew sysid_run`, then `lateral_id.py
--sysid --write`), not more racing data.

### 4c.1 What was added after the first table (Sep 19, later)

**A lateral BREAKAWAY cliff, and where the live record puts it.** The affine
delivery ratio is right to asked ay ~8.5 and badly wrong above it: measured
p50 on the two shipping logs is 0.936 (ay 6-7), 0.923 (7-8), 0.904 (8-9) and
**0.486** (9-12). Two independent signs that the quasi-static model stops
describing that regime: no first-order pole fits the ay > 8 samples at all
(rms 0.25 1/m, and FLAT in tau, against 0.063 at ay 6-8), and the `-explore`
skidpad reports the tyre holding ~0.9 of asked curvature to 9.9 m/s² and then
collapsing to 0.24-0.35 inside one 0.5 m/s step.

Where the cliff sits is settled by the live record, not by the fit. Driven in
the surrogate, 60-lap cells at both periods:

| cliff / post-ratio | shipping, 0.058 | shipping, 0.097 | h7 | h8 |
|---|---|---|---|---|
| 8.8 / 0.48 (the racing-log fit) | 11.140 (+0.33 vs live), **1 collision** | 11.450 (−0.03) | 0 | 0 |
| 9.3 / 0.55 | 10.970 (+0.16) | 11.410 (−0.07) | 0 | 0 |
| **9.6 / 0.70 (shipping)** | **10.880 (+0.065)** | **11.420 (−0.063)** | 0 | 0 |
| 10.0 / 0.55 | 10.880 (+0.065) | 11.400 (−0.08) | 0 | 0 |

The shipping config has 100 clean live laps at both periods, so a cliff that
makes it collide is too low. 9.6 is both clean and the skidpad's own number,
so the 40 racing samples above ay 9 are read as transients. **It does not
rescue the false-clean arms**: h7 and h8 take 0 collisions in 60 laps per
period at every cliff from 8.8 to 10.0, because they never ask enough ay in
the surrogate to break away. The cliff is in because it is measured and costs
nothing, not because it closed the gap.

**Two harness parameters existed only in the node.** `servo_head_kappa` (the
curvature gate on the exit servo, which the SHIPPING config sets to 0.4) and
`servo_decel_look_m` were both read into `MPC3Sim.P` and never used, so every
harness cell that set either of them silently measured the default. Both are
now ported verbatim; the shipping defaults stay byte-identical under `cmp` on
`--dump-rec`. Any earlier harness number for those two knobs is void.

### 4c.2 The verdict table on the final plant, and what it screened

Re-run after the cliff and the two ported parameters, 200 laps per cell
(2 seeds x 100), both periods:

| arm | surrogate | live | agree |
|---|---|---|---|
| shipping (h3 + pub_gate + gated servo) | clean / clean | clean / clean | **YES / YES** |
| `_h2` | collide (s 42.7-43.0) / clean | collide / collide | **YES** / NO |
| `_h7`, `_h8` | clean | collide (10/27, 4/32) | NO |
| `_h3c` + ungated servo | clean / clean | clean / collide (6/100) | **YES** / NO |
| ungated servo on h3 | clean / clean | collide (1/100) / clean | NO / **YES** |
| `ay_limit 16` | clean, **+0.385 / +0.380 s/lap** | clean but slower | **YES / YES** |
| `lead_tk 4.0` | collide (106 in 200) | collide (10/23) | **YES** |
| weave notch, `sched_bo` | clean | collide | NO |

**2 true collide, 0 false collide, 6 true clean, 7 false clean: 8 / 15.** The
one-sided reading is unchanged and is the whole value of the tool: a surrogate
collision is evidence, a surrogate clean sheet is not.

Driven as a screen on the candidates that were open on Sep 19 (200 laps per
cell, shipping flags, baseline 10.890 / 11.425):

| candidate | 0.058 | 0.097 | |
|---|---|---|---|
| `h3v86`, `cornr_mhw50`, `servo_k_p 1.0`, `servo_head_max 0.6`, `servo_decel_look_m` 1.5/3.0 | −0.025..+0.045 | +0.010..+0.020 | neutral, and the last two agree with the live "wash" |
| `steer_tau_k` −0.25 / −0.10 | −0.010 | +0.075 / +0.040 | no gain at the organisers' period, where its claim was −0.05 s/lap |
| `lmpc_c` | +0.240 | +0.425 | **slower at both**, where its claim was −0.05 s/lap at 0.098 |
| `lon_v8b55a65` | +0.410 | +0.410 | slower |
| `apex_f42a`, `apex_f42i` | +0.445, +0.685 | +0.475, +0.645 | slower |
| `hyb675b` | +0.290 | +0.535 | slower; live says +0.244 / +0.295, so the sign and roughly the size |
| `exitm _h4` | −0.040 but **3 collisions / 200 at s 24.7** | +0.190 | **VETO** — the corner the related `_ay90h4` arm hit live at s 26.8 |

and on the f42k geometry's per-zone speed releases (built with
`scripts/partial_blend.py`; the all-zeros blend reproduces the committed
`finals_mesh_apex_f42k_race.csv` to 0.014 m/s and the same 10.282 s ideal, so
the zones really are s 17.5-21.5, 36-40 and 42-2.1):

| release | ideal | 0.058 | 0.097 |
|---|---|---|---|
| base (= f42k_race) | 10.282 | +0.135 | +0.135 |
| A 0.25 / 0.50 (first hairpin) | 10.243 / 10.205 | +0.105 / +0.105 | +0.120 / +0.090 |
| **B 0.25 / 0.50 (s 36-40)** | 10.217 / **10.160** | +0.075 / **+0.020** | +0.020 / **−0.030** |
| C 0.25 / 0.50 (last hairpin) | 10.222 / 10.168 | +0.135 / +0.100 (**2 coll at s 42.6-42.8**) | +0.105 / +0.080 |

**B 0.50 is the only arm in 22 cells that beats shipping anywhere, and it does
it at both periods**; C 0.50 is the only collision, on the corner the live
record already says is capped by the steering lock. One caveat that has not
gone away: f42k_base reads 0.135 s/lap slower than shipping here while the
live Xvfb run has it 0.05 s FASTER, so on the f42k geometry the surrogate and
the live run disagree by ~0.19 s/lap — use these numbers for the DIFFERENCES
between releases on the same geometry, not for the geometry itself.

### 4c.3 The sysid staircase: breakaway is a YAW-RATE limit — and it did not fix the surrogate

`data/logs/sysid_20260919_131340.csv`, 61 cells in the `-explore` world, each
held to steady state with the IMU's own yaw rate rather than a differentiated
IPS path. `tools/sysid_breakaway.py` reproduces everything below.

**The physics finding, which is solid and worth keeping whatever happens to
the surrogate.** The car does not let go at a lateral-acceleration ceiling. It
lets go at a constant **asked yaw rate**, `v · tan(δ) / L`:

* over the 51 usable cells the attached ones reach at most **2.627 rad/s** and
  the ones that let go by yaw rate start at **2.793** — a clean separation
  with nothing in between;
* a second, independent ay ceiling near **10.5 m/s²** binds only above 5 m/s,
  where the yaw rate is small;
* together, `r_ask > 2.70 or ay_ask > 10.5` classifies **50 of 51 cells**
  (16/16 broken caught, 0 missed, 1 marginal);
* past breakaway the car holds **`2.30 · √v` m/s²** whatever the steering
  (±0.146 over all 16 broken cells) — 3.1 m/s² at 2 m/s rising to 5.2 at 5.6.

This also explains why the peak ay looked speed-dependent (4.60 at 2 m/s,
10.28 at 5): at low speed a given curvature needs a big steering angle, so the
yaw-rate ceiling bites long before the grip ceiling does.

**Why it is `--plant-breakaway`, opt-in, and NOT in `--plant-fit`.** The
staircase HOLDS each cell for about a second. On track the excursions past the
threshold last **0.19-0.29 s (p50)**, and over those the live car does not let
go at all. Measured on the display log:

| asked yaw rate | samples | delivery ratio |
|---|---|---|
| > 2.40 rad/s | 9.4 % | 0.954 |
| > 2.71 (the staircase's own threshold) | 6.3 % | **0.952** |
| > 3.00 | 3.2 % | 0.949 |
| > 3.50 | 0.13 % | **0.151** |

So 2.71 is the steady-state threshold and ~3.5 is the threshold for a
track-duration excursion; the difference is an onset time the staircase cannot
see. Reconciled that way (`R_CRIT` 3.50, `BREAK_TAU_ON` 0.12 s, band 0.15) the
law is the best-behaved setting in a 32-cell sweep of (threshold × onset ×
band) — and it still makes the surrogate **worse at both of its jobs**:

| | shipping pace vs live, 0.058 / 0.097 | verdict table |
|---|---|---|
| without the law (shipping plant) | **+0.065 / −0.063 s/lap**, 0 coll / 120 | 8 / 15, 0 false collide |
| with it, best setting | +0.375 / +0.102 s/lap, 0 coll / 120 | **8 / 15, the same six false cleans** |
| with it, at the staircase's own 2.71 | 73 collisions in 120 display laps | — |

**The six false-clean arms did not move.** `_h7`, `_h8`, `h3c` + servo at
0.097, the ungated servo at 0.058, the weave notch and the BO schedule are all
still clean in 200 surrogate laps. And the s 36-40 wide excursion the law was
supposed to produce reached −0.086 m against the live −0.345.

Where the lost time goes is diagnostic: the law fires at s 0-4 and s 14-16,
which the live histogram says are *not* the high-demand zones (live's are
s 18-20). The surrogate asks slightly more yaw rate than the live car at the
last hairpin exit, and a hard threshold turns that small pre-existing
difference into lap time. **Adding true physics to a model made its aggregate
fit worse, because the threshold amplifies every residual error rather than
correcting it.** That is worth remembering before the next "more realism will
fix it" change.

So: the constants are measured and committed, the tool reproduces them, the
law is one flag away — and the default plant is unchanged, because on the two
things that can be checked against live results it is better without it.
`--plant-breakaway 1 --plant-r-crit 2.71` reproduces the staircase's own
number for anyone who wants to see it.

### How to use it, and what it is worth

It is a **one-sided screen**: a surrogate collision is evidence, a surrogate
clean sheet is not. Used that way it would have vetoed `_h2` and `lead_tk 4.0`
before they took live slots, and it correctly called `ay_limit 16` slower at
both periods — four of the five live slots that section 3 records as wasted on
harness predictions.

```bash
# laps / collisions (with s) / per-2 m segment times, shipping flags by default
taskset -c 16-27 nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  MKL_NUM_THREADS=1 .venv/bin/python tools/surrogate_race.py \
  --line finals_mesh_exitm_m10t6_h3_race --frame 0.097 --laps 100 --seeds 3

# a candidate param is one -r, exactly as run_exp.sh takes -p
... tools/surrogate_race.py --line finals_mesh_hyb675b_race -r ay_limit=16.0

# pace against a live log, bin by bin
... tools/surrogate_race.py --line finals_mesh_exitm_m10t6_h3_race --frame 0.058 \
      --compare-log data/logs/race_20260919_044701.csv

# the whole held-out verdict table
... tools/surrogate_validate.py --laps 100 --seeds 3
```

`tests/test_plant_fit.py` pins the opt-in property and every fitted constant.

---

## 4d. The RL residual policy (Sep 19, agent, branch `rl-track`)

**Nothing here is in the image and nothing here has been driven.** Phase 2 of
the RL track, built on the surrogate of section 4c and reported whatever it
said.

### What was built

`src/pit_crew/residual_policy.py` is a pure-numpy tanh MLP loaded from an
`.npz` (the competition image has numpy only). It reads the state mpc3 already
holds and returns two numbers: an **additive steering delta** hard-bounded to
**±0.03 rad**, and a **multiplicative scale on the speed setpoint** bounded to
**0.95-1.05**. mpc3 gains one parameter, `residual_policy` (default `''`,
verified byte-identical), applied after the solve and before the publish
guard, re-clipped by the same grip cap the solver used. `tools/sim_tracker.py`
mirrors the application exactly, so the trainer and the node cannot drift.

The bounds are asymmetric on purpose. Every live failure on this track is the
car running **wide on a corner exit under acceleration**, and section 4c
measured the surrogate to be optimistic exactly there, so the policy is given
**no authority to add speed** inside the three slow zones (s 17.5-21.5, 36-40
and 42-44.7 + 0-2) or wherever `|kappa|` under the car exceeds 0.3. It may
slow the car anywhere. An `.npz` may narrow the bounds, never widen them; a
non-finite net falls back to the identity `(0.0, 1.0)`; an empty parameter
loads nothing and a missing file raises rather than silently not running.

Observation (18 numbers): cross-track error, heading error, speed, the
measured sensor period, the line's curvature at 6 preview distances
(0-4 m) and the left/right wall distance from the car at 4 of them.

`tools/rl_env.py` runs whole episodes of the REAL mpc3 with the policy plugged
in where the node plugs one in — not a gym `step()` API, so there is no second
implementation of the control law to get wrong. `tools/rl_train.py` is PPO in
numpy: no torch, so the arrays that come out of training are the arrays that
ship. The gradients are checked against finite differences and the export is
pinned to the trainer's own forward pass (`tests/test_rl_train.py`).

### Three things that had to be fixed before it trained at all

Worth recording, because each looked like "RL doesn't work here":

1. **A fixed-lap episode cannot reward speed.** The first reward was progress
   per second; summed over an episode of a fixed number of laps the total
   progress is a constant, so the return was a constant too and rewarded
   nothing. It is now `progress − w_time · dt`, a minimum-time objective, with
   `w_time` 4.0 ≈ the lap's mean speed.
2. **Domain randomisation buried the signal.** The randomised period, grip,
   pose bias and rack lag move the return by ±200 between iterations while a
   0.05 s/lap improvement is worth about 0.8. Fixed `(seed, frame)` blocks —
   common random numbers — make the batch-to-batch difference the policy.
3. **An infeasible penalty is not conservatism.** Inflating the wall by 0.10 m
   for the contact test, as the brief asked, makes contact unavoidable: the
   shipping line's own minimum footprint clearance on a zero-residual
   pessimistic rollout is 0.079 m. Trained that way the only reachable policy
   is "go slower", and that is exactly what it learned — near-wall contacts
   fell from 26 per batch to 0 and the lap time did not move. At 0.05 m the
   task is feasible and tight.

### What it learned: nothing worth driving

Two training configurations, 250 PPO iterations each (12 episodes x 4 laps per
iteration, ~3 000 surrogate laps per run), evaluated **held out** — the
evaluation seeds are not the training seeds — at 200 laps per period on the
fitted surrogate and 96 laps on the pessimistic plant.

| arm | fitted 0.058 | fitted 0.097 | pessimistic: contacts / lap / min clear |
|---|---|---|---|
| shipping mpc3 | **10.890**, 0 coll / 200 | **11.425**, 0 coll / 200 | 5 / 11.185 / 0.050 |
| `res_v1` (clear target 0.30, w 6) | 10.965 (**+0.075**), 0 / 200 | 11.470 (**+0.045**), 0 / 200 | 7 / 11.240 / 0.035 |
| `res_v2` (clear target 0.20, w 3) | 10.885 (**−0.005**), 0 / 200 | 11.430 (**+0.005**), 0 / 200 | 3 / 11.200 / 0.000 |

`res_v1` is strictly worse: it bought wall margin it did not need and paid
0.045-0.075 s/lap for it. `res_v2`, trained with the proximity penalty relaxed
so lap time could dominate the return, is **indistinguishable from the
shipping controller** — ±0.005 s/lap, which is inside the surrogate's own
noise — and on the pessimistic plant it trades two contacts for the last of
the clearance (min 0.050 → 0.000 m). Neither is worth a live gate.

**Why, and it is not a training failure.** On the fitted surrogate the
shipping controller laps 10.890 against the live 10.815, i.e. the whole
distance between the surrogate's shipping config and the fastest lap anyone
has driven on this line is **0.075 s**, which is the surrogate's own accuracy.
A residual bounded to ±0.03 rad and ±5 % of the speed setpoint, forbidden from
adding speed in the three zones and in any corner, is being asked to find a
gain smaller than the measurement error of the thing it is being trained on.
The PPO run does what it should: the return improves monotonically within each
common-random-number block, the near-wall count goes to zero and stays there,
the policy std anneals 0.100 → 0.066 — and the lap time does not move.

The surrogate's verdict on the whole exercise is the same one section 4c's
verdict table gives: **on this line, at this sensor period, the shipping
controller is at its optimum to within what can be measured offline.** Every
gain left on this track is geometry and speed profile, which is exactly what
the live record has been saying since Sep 19 07:30.

### If it is ever picked up again

* `residual_policy:=data/rl/res_v2.npz` is a live-ready parameter today
  (`tests/test_residual_policy.py` pins the bounds and the gates), and it owes
  the same two gates as anything else. It is not recommended: it buys nothing.
* The thing that would change the answer is not more PPO, it is a surrogate
  that reproduces the rare wide excursion (section 4c). That needs the sysid
  staircase in the `-explore` image, not more racing data:
  `TAG=2026-iros-explore ./scripts/sim_restart_connect.sh`, then
  `ros2 run pit_crew sysid_run`, then
  `tools/lateral_id.py --sysid data/logs/sysid_*.csv --write`.
* Do not re-run these two arms. `res_v1`'s conservatism came from an
  infeasible wall penalty and `res_v2` is a clean no-op; a third arm inside
  the same bounds has nothing left to find.

---

## 5. What is left, in order

1. **Planned ay on the shipping geometry, paired with the brake lead — they are
   ONE knob.** Ideal laps: ay 6.75 → 10.66 s, 7.5 → 10.41, 8.0 → 10.27 (shipping
   10.99), all at the same clearance and gates. Lines:
   `finals_mesh_rt_dr2_ay{675,75,80}_race.csv`. The harness cannot arbitrate
   (its plant clips at 6.5), so this is a live, ascending ladder. Harness
   (18 laps/cell at 0.058) for ranking only:

   | Arm | Result | Entry clearance |
   |---|---|---|
   | ay 6.75 + lead 0.05 | 6 collisions / 18 | — |
   | ay 6.75 + lead 0.08 | 0 / 18 | 0.050 m |
   | ay 6.75 + lead 0.12 | 0 / 18 | 0.103 m, cut 0.188 |
   | frozen line + lead 0.08 | 0 / 18 | 0.135 m, cut 0.246 |
   | ay 7.5 + lead 0.08 | 1 / 18 | — (needs lead ≥ 0.12) |
   | ay 8.0 + lead 0.05 | 10 / 18 | — |

   Display order: ay 6.75 + lead 0.12 (done: neutral), ay 6.75 + lead 0.05
   (display **110.83 s clean**, but **10 collisions in 24 Xvfb laps at s 38.6**
   — rejected as-is). **Hybrid** (ay 6.75 except the two Xvfb-marginal zones)
   passed **100 Xvfb laps / 0 / 117.72 s**; its display run is in flight. ay 7.5
   stays off the table: both marginal corners are speed-sensitive at 0.098 s.
   **Abort a rung on driven ay p99 > 9.3 or a rising cte p95.** Note on the
   harness: it called ay 6.75 6/18 at 0.058 (live: clean) and 0/18 at 0.094
   (live: 10/24). It ranks arms; it does not predict which period fails.
2. **Corner exits, not apexes (agent, Sep 18).** The `e_free`/`k_e` cross-track
   speed cut is the ONLY `_speed_plan` term that binds on this track (`ay_budget`
   and the backward brake pass contribute 0.000 m/s), and it binds on exits and
   approaches: driven exit deficits 0.5-0.9 m/s at s 19, 37.6 and 43.6 with the
   throttle servo unsaturated (u 0.12 of 0.60). Two arms, both harness-clean at
   0.058 and 0.094: **`servo_k_p 0.5 + servo_head_max 0.3`** (existing params,
   -0.133 s/lap at 0.058, -0.033 at 0.094, entry cut unchanged) and the new
   opt-in `e_cut_accel_scale 0 + e_cut_accel_kappa 0.3` (-0.10 s/lap; without
   the kappa gate 9/36 collisions at s 37.4). `e_free` 0.22 collides. Neither
   rescues the ay 6.75 line. Servo arm is on the display now.
3. **The straight swivel (agent, Sep 18): a genuine 2.9 Hz closed-loop weave**,
   period 0.35-0.37 s at both sensor periods, 3 m wavelength, +-0.034 m of real
   lateral excursion; it self-oscillates with a perfect sensor in the harness,
   so it is the lateral loop at its dead-time crossover. **Not** the barrier
   (`w_barrier 0` leaves it), not `e_free`, not the brake, not pose jitter.
   Every weight knob (`e_dead`, `w_lat`, `w_yaw`, `w_rate`, `w_jerk`,
   `w_dcmd 400`, `cmd_tau 0.30`, `apply_lead_steps`) either leaves it or costs
   laps/clearance. Remedy: new opt-in **`pub_gate_corr 0.05`** (clip the
   published angle to line-matching + 0.05 rad while no corner is within 1.5 m
   and |e| <= 0.25): actuator std -22 % at both periods, +0.01 / +0.055 s/lap,
   no clearance cost, entry cut unchanged, post-collision recovery intact
   (0.03 halves the motion again but diverges after a teleport). It does NOT
   fix the early turn-in DC bias over s 16-18; only the brake moved that. On
   the display now.

   **Sep 19 — the mechanism, and a fix (agent, harness only; not driven yet).**
   The weave is a **saturation relay against the `ay_limit` steering cap**, not
   a gain problem. Traced at the solver (`MPC3.solve`, perfect sensor,
   0.058 s): the RAW published angle alternates between +-0.055 and +-0.069
   rad, and those are exactly `atan(L * ay_limit / v^2)` at the 6.5-7.3 m/s the
   car runs there. The straight is where the speed is highest, so it is where
   the steering cap is TIGHTEST; the solver predicts the lateral error running
   out to 0.4 m if it does nothing, asks for the cap, gets it, overshoots one
   dead time later and flips. `cmd_tau` 0.10 is what turns that +-0.06 square
   wave into the +-0.035 rad seen at the actuator. This is why every gain knob
   failed: scaling `w_lat` AND `w_yaw` to **2 % on the straight, with
   `w_barrier` 0 as well**, leaves the actuator std at 0.031 rad against the
   baseline 0.032 — the loop re-equilibrates at a bigger error and the same
   command amplitude, which is the signature of a relay cycle. Also refuted
   here, do not re-run: `apply_floor_steps` 0.0; the publish lead on straights
   (`straight_lead_scale` 0.5 and 1.5); a straight-only first-order low pass
   (tau 0.30 HALVES the motion at 0.094 s and makes it WORSE at 0.058 s,
   0.0318 -> 0.0465 — a pure lag costs phase exactly where the cycle lives);
   a high-frequency shelf (`straight_hf_gain` 0.3-0.5, same story).

   What works is to stop the relay saturating AND null its frequency:
   **`pub_gate_corr` 0.05 + a second-order NOTCH on the published angle, gated
   to the straight** — new opt-in `weave_notch_k` 0.227, `weave_notch_q` 0.7,
   with the gate widened to `straight_d` 3.0 / `straight_kappa` 0.10. A notch
   has unity gain and ZERO phase shift at DC, so it costs nothing where there
   is something to track, and its centre comes from the MEASURED dead time,
   `k / (period + dead_tick_c)` = 2.20 Hz at 0.058 s and 1.63 Hz at 0.094 s,
   because the cycle is a dead-time crossover and moves with the sensor period
   — one constant in Hz cannot serve both (2.2 Hz fixed is the best arm at
   0.058 and 1.8 Hz fixed at 0.094). Harness, 6 seeds x 12 laps x both frames,
   frozen args, `--plant-realistic --pose-noise 0.03 --pose-latency 0.044
   --plant-brake-at-zero 4.6`, on `finals_mesh_hyb675b_race.csv`:

   | metric, s 2-16 | 0.058 base -> fix | 0.094 base -> fix |
   |---|---|---|
   | 2.0 / 2.5 Hz amplitude | 0.0379 / 0.0299 -> 0.0043 / 0.0021 (**-89 / -93 %**) | 0.0290 / 0.0201 -> 0.0035 / 0.0058 (**-88 / -71 %**) |
   | peak amplitude 1.5-3.5 Hz | 0.0388 -> 0.0080 (**-79 %**) | 0.0289 -> 0.0079 (**-73 %**) |
   | actuator std, mean of 2 m bins | 0.0307 -> 0.0122 (**-60 %**) | 0.0264 -> 0.0127 (-52 %) |
   | lap time | 12.358 -> 12.407 s | 13.128 -> 13.168 s |
   | collisions / 72 laps | 18 -> 12 (all s 37.5) | 0 -> 0 |

   (Pool the whole straight into one std and the reduction reads -47 %; that
   number is mostly the slow drift of the MEAN steering from bin to bin, which
   is not weave. Bin it.) On the FROZEN line `finals_mesh_race.csv` (3 seeds,
   no collisions in either arm, so the lap times are comparable) it is
   time-neutral and gains clearance: per-bin std -60 % / -48 %, lap 12.523 ->
   12.527 and 13.217 -> 13.180 s, min footprint clearance 0.025 -> 0.050 and
   0.175 -> 0.190 m, entry cut unchanged. Teleport-robust (`--teleport-every
   4`, 3 seeds): 54 -> 37 collisions at 0.058, 6 -> 8 at 0.094.

   Two traps found the hard way, both now closed in the code: the filter state
   must be DROPPED when the gate shuts (carrying it across wedged the car on
   the straight it was put down on, s 4.8, repeatedly), and the original
   1.5 m / 0.30 gate left the notch ~20 % engaged into the s 43 hairpin and
   took 186 teleport collisions against the ungated 54. And one arm that must
   NOT ship despite being the best on clean laps: **`nd_delay_s` 0.180 +
   `rate_terms=apply,thr,predict,enc,pose,delay_tick`** (the measured dead-time
   law) gives 0 collisions, per-bin std -67 % / -50 % and 0.05 s/lap FASTER,
   and then takes **247 collisions under teleports at 0.094 s**, all wedged at
   the s 19 hairpin.

   The live run this is waiting on:

   ```bash
   SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
     ./scripts/run_exp.sh weavefix hyb675b_race \
     -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p steer_tau:=0.04 \
     -p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05 \
     -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6 \
     -p pub_gate_corr:=0.05 -p weave_notch_k:=0.227 -p weave_notch_q:=0.7 \
     -p straight_d:=3.0 -p straight_kappa:=0.10
   ```
3. **`lon_v8b55a65`**, if a live slot remains after 1 and 2 (section 4).
4. **An s-indexed parameter schedule (Sep 19, agent, branch `param-sched`) —
   harness only, not driven.** Every knob in mpc3 is one global value per lap,
   and this track is three different corners. `param_schedule:=<path>` loads a
   JSON of arc-length windows (`data/racelines/finals_mesh_sched.json`) that
   override a whitelisted subset of the parameters — `thr_brake_lead_s`,
   `e_free`, `k_e`, `v_mod_min`, `barrier_margin`, the cost weights and a new
   `v_scale` on the offline profile — at full strength inside `[s0, s1]` and
   ramping linearly back to the global value over `blend_m` (1.0 m) on each
   side. This is ForzaETH's sector tuner (`speed_scaling.yaml`, one scalar per
   sector, linear ramp over ±10 waypoints) generalised from the speed to the
   tracker's own knobs. Defaults are byte-identical (`--dump-rec` `cmp`-equal),
   the file is rejected at load if two windows claim the same parameter, and a
   localization jump (or `MPC3.hold_schedule()`, which the node calls on a
   teleport) drops it back on the globals and ramps it in over 5 solves.

   **What it buys, and why one global value could not.** The tuned schedule is
   three windows, and the two that matter carry the SAME parameter with
   OPPOSITE signs 3 m apart:

   | window | s | override (global) | why |
   |---|---|---|---|
   | `hp19_entry` | 16.0-19.0 | `barrier_margin` 0.50 (0.40) | outward bias on the approach; the barrier is clipped out at the apex, so it acts 1.5-2.5 m early |
   | `c37_entry` | 34.5-37.5 | `thr_brake_lead_s` 0.09 (0.05) | brake EARLIER into the corner that runs wide at the organisers' period |
   | `hp43_entry` | 40.5-43.5 | `thr_brake_lead_s` 0.00 (0.05) | brake LATER into the s 43.6 hairpin, where the period-stretched lead costs 0.09 s |

   Harness, `finals_mesh_hyb675b_race.csv`, 8 seeds x 12 laps = **96 laps per
   cell**, corrected plant (`--plant-dead 0.045 --plant-brake-at-zero 4.6
   --pose-noise 0.03 --pose-latency 0.044`), shipping args:

   | frame | arm | laps/coll | lap median | s 18-21 | s 36-39 | s 42-44 | entry cut p50/p95 | min clearance |
   |---|---|---|---|---|---|---|---|---|
   | 0.098 | shipping | 96 / 0 | 12.910 | 1.600 | 1.310 | 1.120 | 0.053 / 0.184 | 0.075 |
   | 0.098 | **schedule** | 96 / 0 | **12.710** | 1.600 | **1.280** | **1.030** | **0.044 / 0.174** | **0.125** |
   | 0.058 | shipping | 96 / 0 | 12.460 | 1.610 | 1.380 | 0.970 | 0.144 / 0.301 | 0.025 |
   | 0.058 | **schedule** | 96 / 0 | **12.305** | 1.625 | **1.340** | **0.880** | 0.135 / 0.304 | **0.075** |

   -0.20 s/lap at the organisers' period with MORE margin everywhere — the
   s 36-39 clearance the corner actually fails on goes 0.146 -> 0.206 at
   0.098 s and 0.025 -> 0.079 at 0.058 s. `finals_mesh_sched_fast.json` adds
   `v_mod_min` 0.70 on the s 19 exit and `v_scale` 1.08 over the s 43.6 apex
   for **-0.265 s/lap** (12.645 / 12.235, still 96/0 at both periods) at
   0.034 m of hairpin clearance; it raises speeds, so it owes Gates C and D in
   its own right. Teleport recovery is unchanged (6/17 cascades against the
   baseline's 7/18).

   **Measured and closed inside this feature** (do not re-open): the same
   `thr_brake_lead_s` 0.0 on the **s 19** entry makes the cut worse (0.051 ->
   0.093) and on **s 37** costs 0.14 m of clearance; pinning `thr_lead_s`
   (0.13-0.17) on the hairpin approach — the per-segment version of the
   throttle-lead pin that was rejected live — gains 0.13 s/lap and pushes the
   entry cut p95 0.184 -> 0.245, so it is rejected for the same reason again;
   `e_free` 0.21-0.24 or `k_e` 1.4 on the hairpin exit costs 0.08-0.10 m of
   clearance (the cliff is a cliff per-sector too); `v_scale` on the s 19
   apex or approach is slower AND tighter; `barrier_margin` 0.44-0.46 on
   s 36-39 costs 0.13 s there and buys nothing.

   **The live run this is waiting on** (the schedule path is the one INSIDE the
   container, and `data/racelines/finals_mesh_sched.json` must be added to the
   Dockerfile's `COPY data/racelines/...` list before any image ships with it):

   ```bash
   SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
     ./scripts/run_exp.sh finsched hyb675b_race \
     -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p steer_tau:=0.04 \
     -p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05 \
     -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6 \
     -p pub_gate_corr:=0.05 -p weave_notch_k:=0.227 -p weave_notch_q:=0.7 \
     -p straight_d:=3.0 -p straight_kappa:=0.10 \
     -p param_schedule:=/home/data/racelines/finals_mesh_sched.json
   ```

5. **The schedule, tuned by constrained Bayesian optimisation (Sep 19, agent,
   branch `sched-bo`) — harness only, not driven.** Section 5.4's schedule was
   hand-built one window at a time. Fröhlich et al. (arXiv:2110.02710) tune an
   MPC's weights with a BO whose GP is given the *operating context*, so the
   optimum it returns is the optimum for the condition that will actually be
   raced. Here the context is the **sensor period**, which on this stack is a
   per-start lottery and is the single variable that decides which corner
   fails. With only two contexts it is cheaper to EVALUATE both than to learn
   across them, so the period enters twice: as the **objective** (median lap at
   the organisers' 0.098 s) and as a **hard constraint** (zero collisions and
   no lost margin at 0.058 as well).

   ```
   J = lap_med(0.098) + 10 * max(0, 0.10 - min clearance) + 10 * collisions
   constraints:  0 collisions at BOTH periods; min footprint clearance,
                 s 19 entry cut (p50 and p95 over s 17.5-19.5) and s 36-39
                 clearance no worse than finals_mesh_sched.json, tolerance
                 0.010 m on a clearance and 0.005 m on the cut (a lap-wide
                 MINIMUM is one sample and moves a centimetre between seeds)
   ```

   `tools/sched_bo.py` (Matérn-5/2 ARD GP fitted by marginal likelihood,
   acquisition = EI x prod P(constraint satisfied), i.e. Gardner et al.'s cEI,
   numpy/scipy only), `tools/sched_eval.py` (one cell), `tools/sched_table.py`
   (the gate table). **117 evaluations**, 3 seeds x 12 laps at each period per
   evaluation, ~40 s each, two processes. 32 of the 117 were feasible; only
   **one** ever collided, so feasibility here is a MARGIN question, not a
   crash question. Trace: `data/racelines/finals_mesh_sched_bo_trace.csv`.

   **Ten knobs**: the three committed windows' values and their leading edges,
   plus `v_scale` on the s 43.6 apex, `v_mod_min` on the s 19 exit, `v_scale`
   over the medium corners s 21.5-32, and the global ramp width `blend_m`.
   What the trace says drives the lap (correlation over all 117):
   `v_scale`(s43) -0.60, `v_scale`(med) -0.51, `blend_m` -0.48, `v_mod_min`
   -0.45, `barrier_margin`(s19) **+0.43** (a wider margin is SLOWER), `s0` of
   the s 43 window +0.41, `thr_brake_lead_s`(s43) +0.30.

   The optimum, `data/racelines/finals_mesh_sched_bo.json`, confirmed at
   **16 seeds x 12 laps = 192 laps per cell** (seeds 0-7 and the holdout 8-15
   agree; the BO only ever saw seeds 0-2):

   | arm | frame | laps / coll | lap median | s 18-21 | s 36-39 | s 42-44 | min clear | cut p50 / p95 | s 36-39 clear |
   |---|---|---|---|---|---|---|---|---|---|
   | shipping (no schedule) | 0.098 | 192 / **0** | 12.950 | 1.625 | 1.305 | 1.115 | 0.112 | 0.064 / 0.200 | 0.152 |
   | shipping (no schedule) | 0.058 | 192 / **0** | 12.470 | 1.600 | 1.370 | 0.970 | 0.025 | 0.173 / 0.333 | 0.025 |
   | `finals_mesh_sched` | 0.098 | 192 / **0** | 12.760 | 1.630 | 1.290 | 1.010 | 0.071 | 0.057 / 0.189 | 0.200 |
   | `finals_mesh_sched` | 0.058 | 192 / **0** | 12.305 | 1.615 | 1.335 | 0.865 | 0.071 | 0.155 / 0.329 | 0.100 |
   | **`finals_mesh_sched_bo`** | 0.098 | **192 / 0** | **12.460** | 1.585 | **1.210** | **0.940** | 0.079 | 0.063 / 0.199 | **0.224** |
   | **`finals_mesh_sched_bo`** | 0.058 | **192 / 0** | **12.020** | 1.540 | **1.220** | 0.860 | 0.050 | 0.157 / 0.327 | **0.158** |

   **-0.30 s/lap on the hand-tuned schedule and -0.49 s/lap on the shipping
   stack at the organisers' period** (-0.29 / -0.45 at 0.058), with the
   s 36-39 clearance the track actually fails on going 0.152 -> 0.224 m at
   0.098 and **0.025 -> 0.158 m** at 0.058. Every corner is quicker and the
   entry cut is unchanged. `finals_mesh_sched_fast.json`, measured on the same
   96-lap cells, is beaten on every count: 12.720 / 12.220 with **1 collision**
   at 0.098 and 0.000 m minimum clearance.

   **Where it is paid for, measured** (p2 of footprint clearance per 1 m bin,
   16 seeds at 0.058, against `finals_mesh_sched`): s 21 **0.135 -> 0.103 m**
   and s 43 **0.103 -> 0.075 m** — both hairpin EXITS, i.e. exactly the two
   windows that raise a speed (`v_mod_min` 0.764 and `v_scale` 1.099). The
   lap-wide minimum falls 0.071 -> 0.050 m for the same reason. A middle rung
   (`v_scale` 1.05, `v_mod_min` 0.65) was driven in the harness and is
   **dominated**: 12.630 / 12.150 for the same 0.050 m minimum. So the trade is
   a real one and it is priced: ~0.03 m of hairpin-exit margin for 0.30 s/lap
   and +0.058 m where the collisions actually happen.

   **This is a speed-gaining change and owes Gate C (100 Xvfb laps) and Gate D
   (32 display laps) in its own right.** The harness ranks arms; it does not
   predict which period fails. Note also that it was tuned WITHOUT the weave
   notch (`pub_gate_corr 0.05` is in, the notch is not) after the notch became
   suspect for the rare straight-line collisions at 0.097 s.

   The live run this is waiting on (the schedule path is the one INSIDE the
   container, and `data/racelines/finals_mesh_sched_bo.json` must be added to
   the Dockerfile's `COPY data/racelines/...` list before any image ships it):

   ```bash
   SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=32 \
     ./scripts/run_exp.sh schedbo hyb675b_race \
     -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p steer_tau:=0.04 \
     -p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05 \
     -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6 \
     -p pub_gate_corr:=0.05 \
     -p param_schedule:=/home/data/racelines/finals_mesh_sched_bo.json
   ```

   **Measured and closed inside this feature** (do not re-open): a wider
   `barrier_margin` on the s 19 entry is SLOWER, not safer — the BO moved it
   from 0.50 down to 0.444 with the entry cut unchanged to 0.003 m; the
   medium-corner `v_scale` must STOP before the s 33.7 brake zone (it is the
   per-segment-optimism failure `lmpc_a/b/d/e` died of, and the constraint on
   s 36-39 clearance is what keeps it honest); and the lap-wide minimum
   clearance is a quantised single-sample extreme (0.025 m steps of the map's
   distance field) that wanders 0.02 m between seed sets, so it ranks arms only
   when pooled over >= 16 seeds.


TUM min-time optimisation: **no** — the corridor cannot pay for it.

**Exit margin from the optimiser, not a hand shift (Sep 19, agent, branch
`exit-margin`). Offline + harness only, nothing driven.** Every ay-9.0 rung
fails the same way at the organisers' period: the car runs 0.38-0.46 m WIDE on
a LEFT-hand corner's EXIT and hits the RIGHT (outside) wall — s 19.9-20.2,
s 26.8, s 44.6 -> 0.3, and earlier s 38.6. At each of those exits the shipping
line sits **0.545-0.555 m** from the right wall and 0.62-0.79 m from the left,
i.e. ~0.42 m of outside margin after the 0.135 m car half-width, which is
exactly what it runs wide by. The one geometry fix tried so far
(`tools/fix_s39_finals.py`, 0.08 m inward over s 36-42) worked, but a hand
shift of the other three exits costs 0.3-0.9 s of ay-9.0 ideal lap because the
ramps add S-bend curvature.

New **opt-in** `--side-margin "s0:s1:side:metres,..."` on
`tools/optimize_raceline.py` (and passed through by `tools/make_track_lines.py`
for the raceline candidates), with `--side-margin-taper` (default 1.0 m,
smoothstep, ramp OUTSIDE the window; `s0 > s1` wraps through s = 0). It shrinks
the corridor on ONE side over arc-length windows of the input centerline
*before* the QP, so the optimiser itself trades exit radius against the margin.
**Where the margin has to be stated is `wall_bounds`, not the width columns**:
the shipping line sits 0.33 m INSIDE its own corridor bound at these exits, so
shrinking `w_tr_right_m` alone moved it by exactly zero (measured). The flag
therefore requires `--min-halfwidth`, and tightens that one-sided floor
(`hi -> hi - M` on the right, `lo -> lo + M` on the left) as well as the width
column. Default off; the shipping line regenerates byte-identical.

The geometry is regenerated from the drill's own reference and flags — verified
byte-identical to `finals_mesh_race.csv` without the new flag:

```bash
.venv/bin/python tools/optimize_raceline.py data/racelines/finals_mesh_centerline.csv \
    --out data/racelines/finals_mesh_exitm_m14t6 --w-veh 0.36 \
    --ay 6.0 --ax 6.5 --brake 4.0 --jerk-lat 40 --ay-tight 4.5 --kappa-tight 1.15 \
    --alpha-smooth 11 --dkappa-max 1.5 --dkappa-refine 3 --kappa-bound 1.0 \
    --min-halfwidth 0.53 --min-halfwidth-map data/maps/finals_mesh \
    --side-margin "19.8:24:right:0.14,25.8:30:right:0.14,44.3:4:right:0.14,36:42:right:0.08" \
    --side-margin-taper 0.6
```

LINE-to-wall distance, marched along the normal in `finals_mesh.pgm`
(min / median per window), against the shipping geometry:

| geometry | ideal (ay 6.0) | \|k\|max | s 20-23 R | s 26-29 R | s 44.5-2.5 R | s 36-42 R |
|---|---|---|---|---|---|---|
| `finals_mesh_race` (shipping) | 11.018 | 1.821 | 0.545 | 0.545 | 0.555 | 0.540 |
| `hyb675b` (hand shift at s 36-42) | — | 1.821 | 0.545 | 0.545 | 0.555 | 0.610 |
| `exitm_m06` (taper 1.0) | 11.081 | 1.800 | 0.600 | 0.585 | 0.610 | 0.605 |
| `exitm_m10` (taper 1.0) | 11.155 | **2.138** | 0.640 | 0.615 | 0.645 | 0.605 |
| `exitm_m10t6` (taper 0.6) | 11.152 | 1.902 | 0.650 | 0.620 | 0.635 | 0.605 |
| `exitm_m12` (taper 1.0) | 11.192 | **2.223** | 0.650 | 0.635 | 0.665 | 0.600 |
| `exitm_m14` (taper 1.0) | 11.140 | 1.829 | 0.665 | 0.665 | 0.680 | 0.605 |
| **`exitm_m14t6`** (taper 0.6) | 11.137 | 1.826 | **0.670** | **0.660** | **0.680** | 0.605 |

Two things the sweep says. **The 0.08 m at s 36-42 reproduces the hand shift's
result from inside the optimiser** (0.605 against 0.610) while giving back the
inside margin the hand shift spent (L 0.555-0.575 against hyb675b's 0.530) and
lowering the local peak curvature (1.07-1.09 against 1.121). And **the margin
does not buy curvature monotonically**: at taper 1.0 the ramp reaches back into
the s 18.9 and s 43.6 hairpin APEXES, and m10 / m12 land on a 2.14 / 2.22 1/m
apex (radius 0.47 m, inside the 0.561 m steering lock) while m06 and m14 do
not. Taper 0.6 keeps the ramp off the apex and fixes it (m10: 2.138 -> 1.902).

Cost is small where the hand shift's was not: **+0.06 to +0.12 s of ideal lap
at ay 6.0**, and at ay 9.0 the new geometries are all FASTER than the
hand-shifted one (`hyb675b` 10.097 s; m06 9.881, m10t6 9.939, m14t6 10.032),
because the hand shift's S-bend is gone.

`pick_config finals_mesh --line <X> --no-write` on the blended lines — every
one **better than the shipping geometry on every metric**: min half-width
**0.520 m** (hyb675b 0.500), body clearance **0.400 m** (0.388), checkpoint
margin 0.648-0.651 m (0.650), gate separation 0.766-0.776 m (0.781),
occupancy 1, and all three clear `barrier_margin` 0.40 on their own feasible
margin (0.385) where `hyb675b` only clears 0.34.

**Harness** (`tools/sim_tracker.py`, shipping `-r` args + `pub_gate_corr 0.05`,
NO weave notch, `--plant-realistic --pose-noise 0.03 --pose-latency 0.044
--plant-brake-at-zero 4.6 --plant-dead 0.045`, 3 seeds x 12 laps = 36 laps per
cell at each frame). `h1` = ay 9.0 blended back to the `hyb675b` speeds over
s 36-40; `h2` = the same plus the s 17.5-21.5 hairpin. Wall column = the CAR's
minimum footprint clearance in that window, worst over the three seeds.

| line | ideal | 0.058 coll / med | 0.098 coll / med | hp1 exit s19.5-21 | s 26-28 | hp3 exit s44-1 | s 38-41 |
|---|---|---|---|---|---|---|---|
| `hyb675b` (shipping hybrid) | 10.836 | **0** / 12.430 | **0** / 13.010 | 0.112 / 0.190 | 0.280 / 0.250 | 0.090 / 0.150 | 0.025 / 0.175 |
| `hyb675b` + ay 9.0, h1 (control) | 10.284 | **2** / 12.020 | **1** / 12.670 | 0.025 / 0.035 | 0.035 / 0.177 | 0.071 / 0.100 | 0.225 / 0.251 |
| `hyb675b` + ay 9.0, h2 (control) | 10.447 | **1** / 12.010 | 0 / 12.560 | 0.146 / 0.202 | 0.056 / 0.236 | 0.035 / 0.125 | 0.250 / 0.275 |
| `exitm_m06` h1 | 10.103 | **1** / 12.010 | 0 / 12.610 | 0.071 / 0.056 | 0.106 / 0.224 | 0.071 / 0.071 | 0.050 / 0.168 |
| `exitm_m06` h2 | 10.234 | **1** / 12.020 | 0 / 12.650 | 0.112 / 0.168 | 0.190 / 0.180 | 0.071 / 0.071 | 0.075 / 0.177 |
| `exitm_m10t6` h1 | 10.160 | **0** / 12.040 | **0** / 12.720 | 0.056 / 0.125 | 0.177 / 0.266 | 0.056 / 0.025 | 0.025 / 0.182 |
| **`exitm_m10t6` h2** | **10.267** | **0** / **11.980** | **0** / **12.660** | 0.090 / 0.158 | 0.236 / 0.266 | 0.071 / 0.025 | 0.050 / 0.158 |
| `exitm_m14t6` h1 | 10.249 | **0** / 12.330 | **0** / 12.840 | 0.079 / 0.125 | 0.280 / 0.354 | 0.071 / 0.090 | 0.035 / 0.251 |
| **`exitm_m14t6` h2** | 10.384 | **0** / 12.390 | **0** / 12.870 | 0.112 / 0.257 | 0.292 / 0.320 | 0.071 / 0.100 | 0.025 / 0.175 |

**The harness reproduces the live failure and the margin fixes it.** The ay-9.0
control collides at both frames (2/36 and 1/36 on `h1`, 1/36 on `h2`) with the
car's own clearance down to 0.000-0.035 m at the very exits the live runs hit;
every `m10t6` and `m14t6` arm is **72/72 laps clean**. m06 is not enough
(1 collision at 0.058 on both blends, matching its +0.04..0.055 m of margin —
under the +0.10 m bar). Read the lap times only as a ranking, never as a gain:
**four harness-predicted gains failed to transfer live on Sep 19**, and the
lines that are clean here still owe Gate C and Gate D.

Ranking (fastest ideal lap whose exits carry >= +0.10 m of outside margin and
which is clean in the harness at both frames): **`exitm_m10t6_h2` first**
(ideal 10.267, +0.095/+0.075/+0.090 m, 72/0, harness median 11.980/12.660 vs
the shipping hybrid's 12.430/13.010), **`exitm_m14t6_h2` second** (10.384,
+0.125/+0.115/+0.125 m, 72/0, and by far the best car clearance at s 26-28 and
the hairpin exits), then their `h1` variants. `m06` is out.

Live, in order — `LOC_ARGS` on every arm, Gate C (100 Xvfb laps) then Gate D
(32 display laps), abort on the first collision at either period:

```bash
LOC_ARGS='-p max_step_mps:=9.0 -p publish_on_reloc:=false' \
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=100 \
  ./scripts/run_exp.sh exitm_m10t6h2_x100 exitm_m10t6_h2_race \
  -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p steer_tau:=0.04 \
  -p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05 \
  -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6 \
  -p pub_gate_corr:=0.05
# then the display, same flags:
LOC_ARGS='-p max_step_mps:=9.0 -p publish_on_reloc:=false' \
VISIBLE=1 SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 \
  LAPS=32 ./scripts/run_exp.sh exitm_m10t6h2_disp exitm_m10t6_h2_race <same -p flags>
# fallback rung, more margin for ~0.12 s of ideal lap:
#   ... exitm_m14t6h2_x100 exitm_m14t6_h2_race <same -p flags>
```

**The LAST hairpin: a later apex, not more exit margin (Sep 19, agent, branch
`exit-margin-2`). Offline + harness only, nothing driven.** The one corner the
shipping geometry still has to take at the safe 1.57 m/s apex is the last
hairpin (apex s 43.6): at full ay 9.0 (`_h2`) it collided 9/32 Xvfb and 1/32
display, every hit at s 44.1-44.3 running 0.22-0.37 m WIDE of the line.

Three families were built on top of `m10t6`'s spec (`19.8:24:right:0.10,
25.8:30:right:0.10,36:42:right:0.08`, taper 0.6; the baseline regenerates
byte-identical bar one `-0.0000`):

* `e14/e18/e22` — the third window moved to `43.9:3:right:M` at M = 0.14/0.18/0.22;
* `n10/n16` — the m10t6 spec **plus** `41.5:43.3:left:M` on the hairpin ENTRY;
* the products `eXXnYY`.

**The one-sided EXIT margin alone makes the apex tighter, not safer.** Pushed
right on its own, the optimiser pays for the exit by closing the apex radius:
e14 0.546 m, e18 0.527, e22 0.488 — all under the 0.561 m steering lock, the
opposite of what the corner needs. The LEFT margin on the entry alone
(`n10`, `n16`) does nothing for the exit (right wall 0.64 vs 0.62). **Only the
two together work**, and they work better than either sum suggests:

| geometry | ideal ay 6.0 | hp3 apex \|k\| (R) | s17-22 \|k\| (R) | R wall s44.1/44.3/0.5/1.5 | s0-5 \|k\| |
|---|---|---|---|---|---|
| `exitm_m10t6` (shipping) | 11.157 | 1.740 (0.575) | 1.902 (0.53) | 0.92 / 0.78 / 0.64 / 0.62 | 0.199 |
| `exitm2_e14` | 11.094 | 1.831 (0.546) | 1.633 (0.61) | 0.96 / 0.82 / 0.70 / 0.66 | 0.087 |
| `exitm2_n10` | 11.187 | 1.905 (0.525) | 1.815 (0.55) | 1.04 / 0.84 / 0.68 / 0.64 | 0.078 |
| `exitm2_e14n10` | 11.087 | 1.620 (0.617) | 1.541 (0.65) | 1.06 / 0.86 / 0.70 / 0.68 | 0.064 |
| **`exitm2_e18n10`** | **11.076** | **1.569 (0.637)** | 1.622 (0.62) | **1.08 / 0.86 / 0.74 / 0.72** | 0.067 |
| `exitm2_e22n10` | 11.091 | 1.617 (0.618) | 1.732 (0.58) | 0.96 / 0.84 / **0.76 / 0.76** | 0.072 |

`e18n10` is the fastest ay-6.0 geometry of the whole exit-margin family, opens
the last hairpin from R 0.575 to **0.637 m** (clear of the 0.561 lock, so
`apex_deviation` 0.000 and effective clearance 0.400 against h3's 0.365),
carries **+0.10 m** more right-wall margin exactly through the impact band, and
straightens the exit onto the start line (|k| over s 0-5: 0.067 vs 0.199). It
also incidentally relaxes the s 19 hairpin (1.902 -> 1.622). `pick_config
finals_mesh --line <X> --no-write` on every `n10` product: min half-width
0.520, body clearance 0.400, checkpoint margin **0.652 m** (h3 0.651), gate
separation **0.792 m** at 19->20 (h3 0.776), occupancy 1, barrier 0.40.

Speed rungs on each geometry (`tools/reprofile.py --ay 9.0 --ax 6.5
--brake 4.6 --vmax 8.0`, then `scripts/partial_blend.py <ay90> <safe_ref>
<out> "17.5:21.5:0,36.0:40.0:0,42.0:44.7:A,0.0:2.0:A"`; the `safe_ref` is the
`hyb675b` speeds mapped onto the new geometry by FRACTIONAL s, which
reproduces the shipped `_safe_ref` to 5e-05 m/s and `_h3`/`_h2` to 0.037 m/s):

| line | ideal | apex v | v@44.4 | v@1.5 |
|---|---|---|---|---|
| `exitm_m10t6_h3` (shipping) | 10.481 | 1.58 | 2.50 | 4.93 |
| `exitm_m10t6_h2` (collided live) | 10.267 | 2.05 | 2.82 | 5.19 |
| `exitm2_e18n10_a0` | 10.495 | 1.57 | 2.37 | 4.93 |
| `exitm2_e18n10_a05` | 10.358 | 1.98 | 2.55 | 5.13 |
| `exitm2_e18n10_a075` | 10.300 | 2.19 | 2.64 | 5.23 |
| `exitm2_e18n10_a10` | 10.248 | 2.40 | 2.73 | 5.33 |
| `exitm2_e22n10_a10` | 10.244 | 2.35 | 2.82 | 5.34 |

**The harness has NO signal at this corner and must not be read as one.**
3 seeds x 12 laps at 0.058 and 0.098 (`--plant-realistic --pose-noise 0.03
--pose-latency 0.044 --plant-brake-at-zero 4.6 --plant-dead 0.045`, shipping
`-r` args + `pub_gate_corr 0.05`, no notch): **every collision in all 24 cells
is at s 37.2-37.9**, none at the hairpin — including 2/36 on the shipping `h3`
line that is 100/0 live at both periods, and 0/72 on `h2`, which is the line
that actually crashed live. What the harness does give is the CAR's minimum
clearance at s 42-44.8 (worst of three seeds, 0.058 / 0.098):

| line | coll (all at s 37) | car clr s42-44.8 | car clr s0-3 | med lap 0.058 / 0.098 |
|---|---|---|---|---|
| `m10t6_h3` (live clean) | 2 / 0 | 0.050 / 0.125 | 0.150 / 0.125 | 12.310 / 12.750 |
| `m10t6_h2` (live 9 coll) | 0 / 0 | **0.035 / 0.050** | 0.100 / 0.050 | 12.010 / 12.690 |
| `e14_a10` | 0 / 0 | **0.025 / 0.056** | 0.150 / 0.100 | 12.180 / 12.550 |
| `n10_a10` | 2 / 0 | 0.090 / 0.100 | 0.225 / 0.250 | 12.110 / 12.500 |
| `e14n10_a10` | 2 / 0 | 0.075 / 0.075 | 0.150 / 0.125 | 12.280 / 12.560 |
| `e18n10_a05` | 2 / 0 | 0.100 / 0.103 | 0.175 / 0.175 | 12.320 / 12.730 |
| `e18n10_a075` | 4 / 0 | 0.075 / 0.125 | 0.175 / 0.175 | 12.280 / 12.630 |
| **`e18n10_a10`** | 1 / 0 | **0.100 / 0.100** | 0.150 / 0.125 | 12.260 / 12.650 |
| `e22n10_a10` | 0 / 0 | 0.075 / 0.100 | 0.150 / 0.150 | 12.220 / 12.620 |

Read only the clearance column: at the FULL ay-9.0 apex speed the `e18n10`
geometry keeps 0.100 m of body clearance through the hairpin where `h2` kept
0.035, and `e14_a10` (exit margin without the later apex) keeps 0.025 — the
same place `h2` failed live. That is the offline case for the pairing and the
case against the exit margin on its own.

Ladder to drive, `LOC_ARGS` on every arm, Gate C then Gate D, abort on the
first collision at either period:

```bash
LOC_ARGS='-p max_step_mps:=9.0 -p publish_on_reloc:=false' \
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=100 \
  ./scripts/run_exp.sh exm2_e18n10a075_x100 exitm2_e18n10_a075_race \
  -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p steer_tau:=0.04 \
  -p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05 \
  -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6 \
  -p pub_gate_corr:=0.05
# rungs, in order: _a075 (ideal 10.300) -> _a10 (10.248) -> e22n10_a10 (10.244);
# safety-only fallback _a0 (10.495, h3 speeds on the better geometry).
```

**Win-stack lines (Sep 19, Opus agent, branch `win-stack-lines`).** Ideal
<= 9.3 s IS reachable on this corridor (min-half-width 0.42-0.46 geometries at
ay 9.0: 8.90-9.27 s) but only by breaking the checkpoint/gate rules, cutting
effective clearance 0.377 -> 0.300, and taking **25-88 collisions per 36 harness
laps at both periods** — every one at s 37.0-37.5, the corner the flat
geometries move curvature INTO. The jerk limiter's split-budget fix is already
in; bypassing it (`--jerk-lat 0`) buys 0.2-0.35 s of ideal for 20-35
collisions/36. `w_veh` is inert. **The honest gain is the profile, not the
geometry: ay 9.0 on the shipping geometry with s 36-40 blended back to the
shipping speeds** (`A_ay90h`, ideal 10.06 vs 10.82, 60/0 harness laps at both
periods; ay 8.0 and 7.5 rungs 10.26 / 10.39). Rebuilt here on the hyb675b
geometry as `finals_mesh_win_b_ay90h1_race.csv` (ideal 10.10, s 36-40 window)
and `_ay90h_race.csv` (10.51, also s 23.5-28.5). Abort per rung: driven ay p99
> 9.3, cte p95 rising, any collision at s 36.5-38.5, wall p2 at s 38-41 < 0.20.
Addendum (same agent): TUM iterative call converges at 2 iterations (5 and 8
byte-identical); corner opening drops the reference's wall distance 0.559 ->
0.461 and every line off it is "corridor exhausted" (this track's 1.34-1.36 m is
all width, no slack, unlike Porto); the fully constrained TUM line is slower AND
tighter than shipping (10.80 s, 0.236 m). **No understeer on this simulator**:
achieved/kinematic curvature 0.96-1.01 to full lock on real display logs (the
`vehicle.py` table says 0.40 at lock; CLAUDE.md note marked superseded). Driven
ay p99 on those logs is 6.50 / 6.81, not 9.6. **Rule: a non-zero collision
count at either period disqualifies a rung** — failures here are clusters (10
in 11-24 laps), not a rate, which is why Gate C is 100 laps at zero.

**Localization audit (Sep 19, Opus agent, merged from `audit-localization`).**
The pose error is PERIOD-INDEPENDENT in every segment (|lat| p95 0.095/0.094 at
the s 19 hairpin, 0.165/0.127 at s 43, straight 0.013/0.012; display/Xvfb), so
the 0.45 s/lap hairpin overhead cannot be bought back in localization. New
facts: the IPS truth is the REAR AXLE, so the published pose sits +0.08..0.10 m
ahead everywhere (the map yaml's `lidar_offset` 0.1933 should be 0.2733 against
it); a curvature-proportional lateral bias toward the OUTSIDE of corners reaches
−0.09..−0.105 m at the s 43 apex and reproduces on a closed replay (likelihood
field, tunable: `beam_temp 4.0`, `n_beams 240`, `motion_noise_theta 0.01`, none
driven yet); **the wheels spin at every launch and the estimate runs +5 m ahead
for the first 3 s** and after any straight collision (opt-in `dr_accel_max 4.0`).
Bugs fixed, normal path byte-identical: a clock reset made the node reject every
later scan for the rest of the run; one future-stamped encoder sample froze the
speed estimate forever; int-truncated grid index below the map origin; NaN pose
publish; zero `sigma_hit`/`beam_temp`. The log's `speed` column is the LOGGER's
left-wheel filter, not localization's estimator — the "enc_tau lags p2 −1.1 m/s"
note was about the logger. Always pass `LOC_ARGS` on live arms (done tonight).

**Pose reference (Sep 19, Opus agent, branch `pose-offset`). Offline only,
nothing driven. The along-track fix is real and CHEAP; the lever-arm variant is
CLOSED; the fix is a MARGIN lever that COSTS pace.** Two opt-in knobs in
`localization.py`, both 0 by default and the shipped pose stream byte-identical
(the audit's replay against `a7dad98` still matches sample for sample):
`pose_along_offset_m` shifts the PUBLISHED pose back along its heading (0.08 =
the rear axle) and `lidar_offset_override` sets the lever arm the FILTER casts
from, which the map YAML otherwise wins outright.

Measured by `tests/pose_offset_replay.py` — a closed replay of the real node
against `finals_mesh` with the truth as the REAR AXLE and the scans cast from
`truth + 0.2733`, 6 laps of each of the two clean 100-lap logs
(`race_20260919_044701` 0.058 s, `race_20260919_050630` 0.093 s):

| segment | along p50, base D/X | shift 0.08 | lever 0.2733 | lat p50 base | lat p50 shift / lever |
|---|---|---|---|---|---|
| straight s 2-15 | −0.472 / −0.446 | −0.552 / −0.526 | −0.469 / −0.525 | +0.000 / −0.001 | +0.000 / +0.001 |
| hairpin s 18-21 | +0.144 / +0.133 | +0.064 / +0.053 | +0.067 / +0.052 | −0.017 / −0.017 | −0.017 / −0.016 |
| apex s 24-25 | +0.099 / +0.101 | +0.019 / +0.021 | +0.019 / +0.023 | −0.016 / −0.018 | −0.017 / −0.018 |
| corner s 36-40 | +0.165 / +0.168 | +0.085 / +0.088 | +0.086 / +0.091 | −0.006 / −0.006 | −0.005 / −0.004 |
| **apex s 43.2-44.2** | +0.114 / +0.099 | +0.034 / +0.019 | +0.036 / +0.018 | **−0.149 / −0.086** | **−0.150 / −0.085** |

* both knobs remove **exactly 0.080 m** of along-track lead in every observable
  segment, at both periods, and agree to within 0.006 m — so the cheaper one
  wins: the output shift needs no map rebuild, no `PIT_CREW_LIDAR_OFFSET`
  change and no preflight change (preflight checks the map's stamp against
  `LIDAR_OFFSET` and knows nothing about the override, which is itself a
  reason not to use the override live);
* **neither touches the corner-outward lateral bias.** A control that casts
  from 0.1933 — i.e. perfect geometry — still shows −0.028 / −0.032 m at the
  s 43 apex, so that bias is the likelihood field and the map discretisation,
  not the lever arm. `lidar_offset_override` is therefore CLOSED as a fix for
  it; it stays as the only way to change the lever arm without a map;
* the straight is unobservable in `s` and its along-track error is a random
  walk in both variants — do not read the straight row as a bias;
* **the MAP is not the problem.** `finals_mesh` is `source: mesh`:
  `tools/map_from_mesh.py` rasterises the Unity track mesh and uses
  `LIDAR_FROM_POSE` for **nothing but the YAML stamp**. The grid is true
  geometry and does not depend on the offset, so a "consistent rebuild" would
  be a one-line stamp change with a byte-identical PGM, raced with
  `LIDAR_OFFSET=0.2733`. No `finals_mesh_ra` was built and none is needed.

What it would buy and cost, on the fitted surrogate
(`finals_mesh_exitm_m10t6_h3v86_race`, shipping flags, 96 laps per arm, **0
collisions in every arm**), modelled as `--pose-lon-bias` 0.100 → 0.020. Every
cell is the pair of values from two disjoint seed triples (seeds 0-2 and 3-5),
which is also the error bar:

| frame | quantity | as shipped (0.100) | corrected (0.020) |
|---|---|---|---|
| 0.058 | lap median | 10.880, 10.880 | **11.080, 11.060** |
| 0.058 | cut, hairpin s 19 | +0.243, +0.245 | +0.170, +0.175 |
| 0.058 | cut, s 24 (tightest) | +0.232, +0.219 | +0.203, +0.204 |
| 0.058 | cut, s 38 | +0.210, +0.214 | +0.204, +0.201 |
| 0.058 | cut, hairpin s 43 | +0.269, +0.272 | +0.201, +0.196 |
| 0.097 | lap median | 11.420, 11.420 | **11.580, 11.570** |
| 0.097 | cut, hairpin s 19 | +0.161, +0.141 | +0.049, +0.054 |
| 0.097 | cut, s 24 (tightest) | +0.193, +0.195 | +0.176, +0.173 |
| 0.097 | cut, s 38 | +0.122, +0.122 | +0.125, +0.116 |
| 0.097 | cut, hairpin s 43 | +0.191, +0.181 | +0.126, +0.124 |

i.e. the correction removes **0.07-0.11 m of the cut at the two hairpins** and
moves turn-in ~0.1 m later there, but costs **+0.15-0.20 s/lap** (1.5-2.0 s of
race). The +0.10 m lead is currently acting as a free, speed-dependent
corner-entry lead and apex-cutter. It does **not** help at s 24.4, the tightest
point of the lap (−0.02 m), so it does not buy the margin that would let the
line be trimmed THERE; at the two hairpins it does, and the honest use of it is
paired with a line or profile that spends the recovered 0.07-0.11 m, not on its
own. Surrogate rule applies: the clean sheets are worth nothing, the pace is
real.

Live arms, each owing Gate C then Gate D (shipping `LOC_ARGS` plus one knob):

```bash
# A -- the recommended variant: published pose becomes the rear axle
LOC_ARGS='-p max_step_mps:=9.0 -p publish_on_reloc:=false -p pose_along_offset_m:=0.08' \
SIM_QUALITY=ultra CONTROLLER=mpc3 MAP=finals_mesh LIDAR_OFFSET=0.1933 LAPS=100 \
  ./scripts/run_exp.sh pose_shift08_x100 exitm_m10t6_h3v86_race \
  -p rate_adaptive:=true -p w_dcmd:=50.0 -p cmd_tau:=0.10 -p steer_tau:=0.04 \
  -p barrier_margin:=0.40 -p thr_brake_lead_s:=0.05 \
  -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3 -p brake_a_full:=4.6 \
  -p pub_gate_corr:=0.05
# B -- the filter's own lever arm (same LIDAR_OFFSET: preflight sees the map,
#      not the override). Offline says it is A plus nothing; drive it only to
#      confirm that live.
LOC_ARGS='-p max_step_mps:=9.0 -p publish_on_reloc:=false -p lidar_offset_override:=0.2733' \
  ... same line and CTRL flags ...
```

**Where the gated h3 line still loses time (Sep 19 08:45, 100-lap logs at each
period, 2 m segments).** It is AT its ideal almost everywhere at both periods.
The one period-dependent loss is the FIRST hairpin: s 18-20 ideal 0.90 / display
0.91 / **Xvfb 1.11 s** (+0.2 s/lap = 2 s/race). At 0.098 s the car turns in later
(|steer| 0.06-0.10 at s 17.5-18 vs 0.12-0.17), runs wider at the apex (−0.18 vs
−0.10 at s 19.5) and carries 1.63 vs 1.85 m/s through it. Cause: the steering
readout lead is 4.0 sensor periods on the display and 2.8 on Xvfb. The Sep 19
lead-law test equalised them DOWN to 2.8 (slower at both); `lead_tk 4.0`
equalises UP and leaves the display unchanged by construction — in flight.

**Why the servo gains nothing at 0.098 s (Sep 19 13:00, 100-lap logs).** With
the servo on, the first hairpin s 18-20 goes 1.11 -> **1.24 s** on Xvfb (ideal
0.90): the approach s 16-18 is 0.04 s quicker and the corner 0.13 s slower. The
servo pushes the car up to plan speed right INTO the braking zone; at the slow
period it arrives hot, runs wide, and the speed cut slows the apex. Opt-in fix:
`servo_decel_look_m` — no over-command when the profile brakes (a_prof < −0.5)
within the next N metres. In flight at 1.5 and 3.0 m.

**Other teams' public stacks (Sep 19 ~17:00 IST, Opus agent; read, never run).**
**MonacoF1 / Ekumen (ICRA 2026 finals winner, 110.27 s) ships its full source in
a public image, `ealban/monaco_f1tenth:2026-iros-compete`** (pushed Sep 19 — their
entry for this final). From their shipped config: Nav2 regulated pure pursuit at
80 Hz (their MAP controller and Stanley are commented out); pose extrapolated by
MEASURED message latency + one control step; speed looked up from a rasterised
VELOCITY MAP by map cell, not by arc length; closed-loop throttle = PI (p 5, i 1)
with a second-order prefilter and the integrator clamped to `max_traction_slip *
|v|` (0.15) on top of the same 0.0408/(m/s) feedforward we measured — the
ingredient our bang-bang `throttle_kp` attempts lacked; genetic raceline (100
offset genes, pop 200 x 300 gen, seeded from min-curvature) with a wall
`safety_margin` of **0.17 m** on this track (we hold ~0.30); encoder odometry
that models wheel slip; an impact detector that hard-stops, reseeds AMCL and
soft-starts; AMCL 250 beams / 1750-3000 particles + an EKF. VAUL's RACE stack is
not public (only a wall-follower, their GPU particle filter, the TUM optimiser,
`On-Track-SysID`); NTU DeepSpeed (49.83 s qualifier) has nothing public. Three
teams independently use ~0.040 throttle per m/s and none reverse-brakes; Khalifa
documents the exact-zero brake. **Our own `bongracers/roboracer:iros2026-qualification`
is public with full source and racelines.** Policy: learn techniques, never copy
code or lines. Source extract is in the session scratchpad, not in the repo.

**Literature review (Sep 19, Opus agent).** Ranked for our limits: (1) velocity
replanning from driven logs / a spatial grip map in Frenet cells (GripMap,
arXiv 2504.12115: +5.2 % lap time) — the principled form of our hand-held slow
zones; (2) exit-acceleration-limited (combined-slip) forward pass, matching the
finding that running wide tracks exit acceleration; (3) empirical feedforward
steering with a curvature preview sized to the 3.2 rad/s rate limit (learned
versions lose closed loop: arXiv 2605.21111); (4) a buffered delay-compensating
EKF rather than a fast open inner loop (explains why `control_mode timer` lost);
(5) tuned pure pursuit as a comparison arm (AROLA: PP 10.10 s vs MPC 10.24 s on
hardware); (6) residual RL inside the sim (3-6 %, ~33 h of wall clock, post-finals).
Drifting: inapplicable to this tyre. To download: arXiv 2509.18506 "Spatial
Envelope MPC"; Velenis & Tsiotras ISIE 2005.

**Braking audit (Sep 19 ~17:30 IST, 100-lap logs).** Driven minus planned speed
through each braking zone: into the FIRST hairpin (s 12-18.8, plan 8.0 -> 1.85
at the brake's 4.6 m/s2 limit) the car is +0.26 / +0.30 m/s over plan mid-zone
at both periods and **still +0.25 m/s hot at the END on Xvfb** (on plan on the
display) — that is the first hairpin's 0.2 s/lap loss at 0.098. Into s 37
(s 31.5-37.2) the display is on plan while Xvfb is 0.2-0.3 m/s UNDER (braking
starts a metre early: 33.7 vs 34.7). Last hairpin: on plan / slightly under.
**METHOD RULE from now on: Xvfb A/B runs use `RATE_MIN_DT=0.094
RATE_MAX_DT=0.100`** — the per-start period lottery (0.088-0.106) confounded two
verdicts today.

**The swivel, resolved (Sep 19 ~15:00 IST).** It is a display-period phenomenon:
straight rack std 0.015 rad at 0.058 s vs 0.006-0.010 at 0.098 s. The cure is a
GENTLER law where there is nothing to track: `straight_pp_k 0.5` blends a
pure-pursuit steer (Ld = 0.5 v, 1.5-5 m, dead-time-projected error) through the
existing straight gate — live on the display rack std 0.0148 -> 0.0068 rad
(-54 %), cmd std -69 %, time-neutral (107.23 vs 107.40 s). At 0.098 s the same
law wanders 0.35 m (2 collisions / 32, then +0.19 s/lap when firmed), so
`straight_pp_period_max 0.075` fades it out above a 0.065-0.075 s measured
period: identical to the baseline on the organisers' box, calm on a fast one.
Rejected on the way: a slew limit (adds lag: 2-15 harness collisions in 6 laps),
the notch (5 and 8 straight-line collisions per 100 Xvfb laps).

**Fitted surrogate (Sep 19, `rl-track` branch, Phase 1).** `tools/surrogate_race.py`
(on that branch) predicts the shipping config's lap to 0.03-0.08 s at both
periods (was 1.4 s off): the harness had modelled a 0.193 s encoder lag (live:
one sample) and a 6.5 m/s2 grip knee (live: affine 1.050 - 0.0180*ay). It
reproduces 9 of 15 known live verdicts and NEVER invents a collision, but
misses the rare wide excursions at the limit (6 false "clean"). Use: a
surrogate collision is a veto, a clean sheet is nothing, its pace is real.
Closing the blind spot needs the sysid staircase in the `-explore` image.

**Surrogate screening of the open candidates (Sep 19 23:00, 200 laps/cell, both
periods).** VETO: exitm `_h4` (3 collisions at s 24.7 — the live-failing corner
family). Not worth a slot (slower at both periods): `lmpc_c` +0.24/+0.43,
`lon_v8b55a65` +0.41, `apex_f42a` +0.45, `apex_f42i` +0.65-0.69, `steer_tau_k`
arms +0.04-0.08 at 0.097. Washes confirmed: servo gain 1.0, headroom 0.6,
`cornr_mhw50`. DISAGREEMENT: `apex_f42k` live -0.05 s/lap at 0.098 vs surrogate
+0.125 — outside its demonstrated +-0.06 accuracy; trust the live run there.
Harness correction: `servo_head_kappa` and `servo_decel_look_m` were never
ported to the harness's MPC3Sim until Sep 19 23:00 — every earlier HARNESS cell
that set them measured the ungated servo (all LIVE results stand).

**THE GRIP MAP (Sep 19 18:45 IST, `sysid_run` in the `-explore` image, 61
cells, `data/logs/sysid_20260919_131340.csv`).** Delivered/kinematic yaw-rate
ratio 0.93-0.95 up to the peak, then a ONE-STEP collapse to 0.25-0.50; past
breakaway the car holds only 3.2-5.3 m/s2 whatever the steering. Peak ay and
the steering at which the ratio first drops below 0.85:
2.0 m/s 4.60 @0.40 (break 0.45) | 2.5 5.57 @0.30 (0.35) | 3.0 6.62 @0.25 (0.30)
| 3.5 7.21 @0.20 (0.25) | 4.0 9.34 @0.20 (0.25) | 4.5 8.96 @0.15 (0.20) | 5.0
**10.28** @0.15 (0.15) | 5.5 8.94 @0.10 (0.15). So: at hairpin speed full lock
is PAST breakaway (why the hairpins cannot be driven faster); the tyre delivers
the winners' 9-10 m/s2 at 4-5 m/s but only inside 0.15-0.20 rad of steering.
Handed to the surrogate for the limit refit.

**Brake probe (Sep 19 19:00 IST, `brake_probe` in the `-explore` image,
`data/logs/brake_20260919_132335.csv`).** From 7.1 m/s: throttle EXACTLY 0.0
gives 6.39 m/s2 with the wheels LOCKED (encoder/IPS slope ratio 0.30, slip
+0.90); every negative command also locks the wheels and the best, −0.15, gives
6.52 m/s2 — not a proportional brake, and a locked wheel steers nothing. The
measured held deceleration from high speed is ~6.4 m/s2 incl. drag, above the
profile's 4.6 plan; the earlier "4.55 held ceiling" is the low-speed figure.

**Closed by agents, Sep 19 ~16:30 IST.** (a) Unused-width corners: three of four
(s 22.2, s 40.9, s 27.6) cannot be given speed — longitudinal REACHABILITY between
the capped zones, not wall width, binds them; only s 30-33 has room (−0.02 s/lap,
below the surrogate's resolution). (b) Pose offset: the +0.08 m along-track lead
is the rear-axle/IPS mismatch (map needs no rebuild; an output shift equals the
lever-arm fix to 0.006 m), but removing it COSTS 0.15-0.20 s/lap — it is a free
corner-entry lead; it only pays paired with a re-optimised hairpin margin. Opt-in
`pose_along_offset_m` / `lidar_offset_override` merged, not driven. (c) Image v6's
first as-shipped test: 1 collision at the s 26.7 exit (1 per ~200 laps on this
family) — insurance variant `h3v86c` (s 24-28 at half raise, ideal 10.50) built.
(d) Closed-loop PI throttle (`thr_pi_kp 3 ki 5 prefilter 0.15 slip 0.15`, opt-in,
merged): surrogate −0.21 / −0.13 s/lap, live gates pending.

**In flight at 22:30 IST:** v6 gates (v86 + period-gated PP); Opus agents on
the unused-width corner speeds (`unused-width`), a MonacoF1-style closed-loop
throttle (`cl-throttle`), and RL Phase 2 on a pessimistic plant (`rl-track`).

**Web research (Sep 19, Opus agent, ~35 min).** No RoboRacer sim winner has
published a stack. Facts worth keeping: **ICRA 2026 finals gold was 110.27 s /
11.00 s best lap** (silver 12.10, bronze 12.53 s laps) on an unseen track — the
finals are far softer than the qualifiers, so collision risk outweighs a second
of lap time. VAUL (6.46 s Porto qualifier) ships the TUM optimiser verbatim with
a **flat 12 m/s² ggv**; their edge is corner-radius opening. ForzaETH's MAP
controller (arXiv 2209.04346) plans steering by inverting a velocity × steer →
achieved-ay table (0.055 m lateral error at 11 m/s, 40 Hz+), the published form
of putting `achieved_curvature()` in the planner; a speed-scheduled lookahead
Ld = m + q·v is their latency fix. The AutoDRIVE paper says actuator delays are
modelled (value unpublished); another 2026 entrant logs 0.017-0.050 s command
age, consistent with `steer_tau` 0.04. No public source for the WheelCollider
curve, timestep, checkpoint or collision logic — our decompiled findings stand.

**Corner-by-corner audit of the shipping line (Sep 19, agent, branch
`corner-audit`). Offline + harness only, nothing driven.** The question was
whether the racing line is under-optimised AT THE CORNERS. Method: march the
normal in `finals_mesh.pgm` for the LINE's wall distance, and bin the two clean
100-lap logs (`race_20260919_044701` display 0.058 s, `race_20260919_050630`
Xvfb 0.096 s) by 0.5 m of `s` for the CAR CENTRE's nearest-wall distance, split
by side of travel, plus the signed offset from the line. Full table committed as
`data/racelines/finals_mesh_cornr_audit.csv`.

**The single structural finding: `--min-halfwidth 0.53` against a corridor of
half-width 0.57-0.70 m leaves the QP 0.04-0.17 m of lateral freedom, so the
shipping line does not apex any corner — it runs mid-corridor everywhere.** The
optimiser says so itself ("finished line holds 0.534 m at s 25.0"), and the
audit confirms the line's nearest wall is 0.53-0.55 m at every apex. What apexes
is the CAR: the harness measures it +0.161..+0.183 m inside the line in corners
at 0.058 s and +0.072..+0.080 at 0.098 s.

Live minimum wall distance (car centre; body clearance is this minus 0.135 m
perpendicular, up to 0.25 m at an angle), display / Xvfb, with the class:

| corner | phase | live min wall D / X | side | class |
|---|---|---|---|---|
| hairpin s 18.9 (L) | entry / apex / exit | 0.426 / 0.553 · 0.424 / 0.553 · 0.424 / 0.503 | L inside, R on exit | **UNUSED WIDTH** — never within 0.42 m of anything; speed-capped by the safe blend, not by width |
| s 22.2 (R) | all | 0.559 / 0.530 | both | **UNUSED WIDTH** |
| s 24.4 (L) | entry | 0.318 / 0.372 | LEFT (inside) | AT THE LIMIT |
| s 24.4 (L) | **apex** | **0.225 / 0.280** | LEFT (inside) | **AT THE LIMIT / the tightest point of the lap**; the car sits +0.24..+0.25 m inside a line that is already on its 0.534 m floor |
| s 24.4 (L) | exit | 0.266 / 0.348 | LEFT | AT THE LIMIT |
| s 27.6 (R) | entry s 26.5-27.5 | **0.266 / 0.372** | RIGHT (outside of the s 24 corner) | **AT THE LIMIT on the display** — the wide exit of the s 24 corner, not its own corner. Corrects the "s 26-30 = 0.46-0.57 m" reading, which is the Xvfb number only |
| s 27.6 (R) | apex / exit | 0.336 / 0.445 · 0.459 / 0.515 | R | UNUSED WIDTH |
| s 30-34 kinks (R) | s 33.5-35.5 | 0.347 / 0.412 | RIGHT (inside) | borderline; left side never nearest (> 0.70 m) — **UNUSED WIDTH on the outside** |
| s 30-33 | — | 0.575 / 0.625 | L | **UNUSED WIDTH** |
| s 37.6 (L) | entry / apex | 0.412 / 0.470 | L | UNUSED WIDTH |
| s 37.6 (L) | **exit s 38.5-39.5** | 0.530 / **0.283** | RIGHT | **AT THE LIMIT at the organisers' period only** — the known period-dependent corner, already capped |
| s 40.9 (R) | all | 0.503 / 0.442 | both | **UNUSED WIDTH** (widest part of the lap: line 0.65/0.67, car 0.65 on the display) |
| hairpin s 43.6 (L) | **entry s 42.5-43.0** | **0.251 / 0.355** | LEFT (inside) | **AT THE LIMIT** |
| hairpin s 43.6 (L) | apex / exit | 0.369 / 0.467 · 0.400 / 0.450 | L / R | at the limit only in the steering-lock sense (closed) |
| straight s 2-15 | entry / body | 0.475 / 0.375 | both | UNUSED WIDTH, but it is a straight — no lap time in it |

Nothing is class (iii) OVER. Two corners are genuinely at the limit and neither
is a corner the previous work had named: **s 24-27 is one early-apex/wide-exit
failure** (inside 0.225 m at s 24.5, then outside 0.266 m at s 26.5-27), and the
**last hairpin's ENTRY** (0.251 m at s 43.0), not its exit.

**Independent optimum.** Re-solved with a plain symmetric margin, profiled
identically (`reprofile --ay 9.0 --ax 6.5 --brake 4.6 --vmax 8.6`, then the
three safe zones blended back with `partial_blend`), against the shipping line
rebuilt the same way (`finals_mesh_exitm_m10t6_h3v86_race.csv`, ideal 10.425 s):

| geometry | ideal | \|k\|max (R) | pick_config min half-width / barrier |
|---|---|---|---|
| `--min-halfwidth 0.45`, no side margin | 10.214 | 1.76 (0.57) | — |
| `--min-halfwidth 0.50`, no side margin | **10.329** | **1.57 (0.64)** | 0.480 / 0.34 |
| `--min-halfwidth 0.53`, no side margin (= `finals_mesh_race`) | 10.405 | 1.82 (0.55) | 0.520 / 0.40 |
| **shipping** `exitm_m10t6_h3v86` | 10.425 | 1.90 (0.53) | 0.520 / 0.40 |

**The whole geometry lever is 0.21 s of ideal lap between a 0.45 m and a 0.53 m
floor, and the exit margins cost only 0.020 s.** TUM min-time was not re-run:
the win-stack agent already measured the fully constrained TUM line slower AND
tighter than shipping (10.80 s, 0.236 m), and the corridor has not changed.

**Re-optimised family** (`tools/optimize_raceline.py --side-margin`, taper 0.6,
same build chain; `finals_mesh_cornr_*`). Hard constraints: `pick_config` checkpoint
margin >= 0.60, gate separation >= 0.60, occupancy 1, min half-width >= 0.50,
and |kappa| at s 18.9 / s 43.6 no tighter than 1.90 / 1.74.

| candidate | floor | change | ideal (Δ) | \|k\| hp1 / hp3 | verdict |
|---|---|---|---|---|---|
| shipping | 0.53 | — | 10.425 | 1.90 / 1.74 | baseline, harness **72 / 0** |
| **`cornr_j`** | 0.50 | shipping right windows + `18.0:20.2:left:0.03,42.3:44.3:left:0.03` (hold the hairpin apexes off the lower floor) + **`23.2:25.6:left:0.10`** | 10.498 (+0.073) | **1.62 / 1.67** | **the only candidate that fixes s 24-27**: line-to-inside-wall at s 24-26 **0.534 -> 0.600 m**, both hairpin radii 0.53/0.57 -> **0.60 m** (clear of the 0.561 lock, `apex_deviation` 0.000). Harness **72 / 0** at both frames, corner cut +0.171 -> +0.131 m. **Fails `min half-width >= 0.50` by 0.02 m (0.480) and forces `barrier_margin` 0.34.** |
| `cornr_g2` | 0.50 | as above without the s 24 window, + `13:17:right:0.05` | 10.398 (−0.027) | 1.90 / — | constraint-clean on kappa, same 0.480 / 0.34 problem; buys nothing at s 24 |
| `cornr_h` | 0.53 | give-back only: windows shortened to `19.8:22.5`, `25.8:28.5`, `36:40.5` | 10.487 (+0.062) | 1.87 / 1.55 | **passes every hard constraint** (0.520 / 0.40, checkpoint 0.652, gate 0.773, effective clearance 0.375 vs 0.365) but moves the tight corners by 0.016 m — not worth a live slot |
| `cornr_mhw50` | 0.50 | no side margins at all | 10.329 (−0.096) | 1.56 / 1.57 | **REJECTED**: gives the exit margins back — right wall at s 38-40 0.619 -> 0.530 m, the corner that DQ'd every ay-9 rung at 0.098 s |
| `cornr_a` / `_b` / `_f` / `_m` / `_n` / `_q` | 0.53 | the s 24 left margin at the shipping floor | +0.16 to +0.33 | up to 2.05 | **REJECTED**: at floor 0.53 the same fix costs 3-5x as much and tightens a hairpin past 1.90 |
| `cornr_c` / `_e` / `_p` | 0.50-0.52 | shipping windows at a lower floor | −0.09 / −0.07 / +0.21 | 2.00 / 2.07 / 1.80 | **REJECTED** on the hairpin-kappa constraint |
| `cornr_k` | 0.50 | every margin raised (0.12 / 0.10) | +0.23 | **2.36 (R 0.42)** | **REJECTED** — reconfirms "margin does not buy curvature monotonically" |

**Harness veto only** (`tools/sim_tracker.py --plant-realistic --pose-noise 0.03
--pose-latency 0.044 --plant-brake-at-zero 4.6 --plant-dead 0.045`, shipping `-r`
args + `pub_gate_corr 0.05 servo_k_p 0.5 servo_head_max 0.3 servo_head_kappa
0.4`, 3 seeds x 12 laps at 0.058 and 0.098). **Read the collision column only —
five of five harness-predicted lap-time gains failed live on Sep 19.**

| line | 0.058 laps/coll, med | 0.098 laps/coll, med | min footprint clearance 0.058 / 0.098 |
|---|---|---|---|
| shipping `h3v86` | 36 / **0**, 12.36 | 36 / **0**, 12.77 | 0.050-0.071 / 0.050-0.100 |
| `cornr_j` | 36 / **0**, 12.35 | 36 / **0**, 12.78 | 0.025-0.100 / 0.075-0.100 |
| `cornr_h` | 36 / **0**, 12.43 | 36 / **0**, 12.75 | 0.050-0.075 / **0.125** |
| `cornr_mhw50` | 36 / **0**, 12.16 | 36 / **0**, 12.54 | **0.025-0.050** / 0.025-0.050 |

**The floor value 0.50 is a different QP basin, not a smooth knob.** At 0.50 the
solver finds a line whose hairpin radii are 0.60-0.64 m; at 0.51, 0.52 and 0.53
it lands on the tight-apex basin (R 0.48-0.55) and is both slower and tighter.
Do not interpolate between them.

**Honest conclusion: under the stated hard constraints the shipping geometry is
already at its constrained optimum.** Nothing here is both faster and legal. The
one real improvement available is `cornr_j`, which is a SAFETY trade — it costs
0.073 s of ideal lap and buys 0.066 m of inside clearance at the tightest point
of the lap plus two hairpin radii clear of the steering lock — and it can only be
had by accepting `min half-width` 0.480 and `barrier_margin` 0.34. That is a
judgement call for the user, not a change to make unilaterally this close to
Gate D.

**The APEXING family (Sep 19, agent, branch `corner-audit`). Offline + harness
only, nothing driven.** The audit above said the 0.53 m floor forbids apexing.
This is what happens when it is lowered. Relaxed constraints (the coordinator's
call, on the evidence that MonacoF1 ships a 0.17 m wall margin on this track):
`pick_config` checkpoint margin >= 0.60, gate separation >= 0.60, **gate
occupancy 1**, lap-wide |kappa| <= 1.90, and line-to-wall >= 0.40 m everywhere.
`min half-width >= 0.50` and `barrier_margin 0.40` are NOT required; the barrier
each line needs is reported instead. Same build chain as everything above.

**Four facts the sweep established, in order of usefulness.**

1. **The floor is worth far more than any margin.** Plain min-curvature at
   `--min-halfwidth` 0.42 gives ideal **10.147 s** (−0.278) with a lap-wide
   |kappa| of **1.05 (R 0.956 m)** — the first line here that genuinely apexes.
   The harness confirms the mechanism: on the shipping line the car runs
   **+0.171 m inside** it in corners; on the 0.42 line it runs **+0.004 / −0.07 m**,
   i.e. the line finally goes where the car was always going.
2. **Gates 7 and 8 sit at the s 18.9 hairpin apex**, 1.03 m apart
   (`finals_mesh_checkpoints.csv`), so **opening that hairpin shortens the path
   between them and puts the body in two trigger boxes at once** — a dropped
   lap. Measured: hp19 apex |kappa| 1.90 (shipping) -> sep > 0.776, occupancy 1;
   1.42 -> 0.720, occupancy 1; **1.13 -> 0.692, occupancy 2**; 0.99 -> 0.683,
   occupancy 2. **The first hairpin cannot be opened past |kappa| ~ 1.4.** This
   is a new hard limit and it is geometric, not dynamic.
3. **Big one-sided margins at a low floor destroy the apex** (the `cornr_k`
   result again, now quantified): margins of 0.18 right / 0.11 left on 8 windows
   at floor 0.42 give |kappa| **2.28 (R 0.44)**; 0.13-0.15 on 6-9 windows give
   1.65-1.81. Small margins on a low floor beat a high floor.
4. **Putting the checkpoint/gate constraint inside the optimiser is a dead end
   here** — `--checkpoints --gate-separation 0.62 --checkpoint-margin 0.32` at
   floor 0.42 stretches the line from 44.3 to **46.3 m** and gives ideal
   **11.03 s**, and the checkpoint margin it achieves (0.527) is *worse* than
   leaving the flag off. Use side margins to place the line instead.

| line | floor | ideal (Δ) | \|k\| hp19 / s37.6 / hp43 | ckpt | gate sep | occ | min line-wall | barrier / lead | harness 3x12 both frames |
|---|---|---|---|---|---|---|---|---|---|
| shipping `h3v86` | 0.53 | 10.425 | 1.90 / 1.08 / 1.74 | 0.651 | 0.776 | 1 | 0.520 | **0.40** / 0.05 | 72 / **0**, clr 0.050-0.100 |
| **`apex_f42k`** | 0.42 | **10.282 (−0.143)** | **1.62** / 1.10 / 1.76 | 0.640 | 0.732 | **1** | 0.460 | **0.34** / 0.05 | **72 / 0**, clr **0.071-0.127** |
| `apex_f42j` | 0.42 | 10.301 (−0.124) | 1.81 / — / 1.76 | 0.640 | 0.733 | 1 | 0.460 | 0.34 / 0.05 | 71 / **1** (s 37.8, the harness's chronic false positive) |
| `cornr_mhw50` | 0.50 | 10.329 (−0.096) | 1.56 / 1.00 / 1.57 | 0.601 | 0.757 | 1 | 0.480 | 0.34 / 0.05 | 72 / 0, clr 0.025-0.050 |
| `apex_f42i` | 0.42 | 10.248 (−0.177) | 0.99 / 1.13 / 1.13 | 0.639 | 0.692 | **2** | 0.440 | 0.34 / 0.00 | 72 / 0, clr 0.056-0.150 |
| `apex_f42a` | 0.42 | **10.147 (−0.278)** | 0.98 / 0.98 / 1.05 | **0.532** | 0.683 | **2** | 0.420 | 0.34 / 0.00 | 72 / 0, clr 0.050-0.125 |
| `apex_f45c` | 0.45 | 10.279 (−0.146) | — / — / 1.73 | **0.552** | 0.737 | 1 | 0.440 | 0.34 / 0.05 | not run |
| `apex_f42d` | 0.42 | 10.264 | 1.42 / 1.06 / 1.65 | **0.515** | 0.720 | 1 | 0.400 | 0.34 / 0.05 | 70 / **2** (s 30.0, s 32.0) |
| `apex_f42e` / `_f42g` | 0.42 | 10.265 / 10.253 | 1.15 / 1.17 | 0.564 | 0.693 | **2** | 0.420 | 0.34 / 0.00 | f42e 72 / 0 |
| `apex_f42c` | 0.42 | 10.252 | 1.71 | **0.516** | 0.724 | 1 | 0.400 | 0.34 / 0.05 | not run |
| `apex_f47a` | 0.47 | 10.533 (+0.108) | 1.18 / — / 1.31 | — | — | — | 0.475 | — | not run |
| `apex_f42b` / `_f45b` / `_f45d` | 0.42/0.45 | 10.406 / 10.397 / 10.274 | **2.28** / **1.97** / 1.84 | — | — | — | — | — | **kappa veto** |
| `apex_f42f` / `_f45e` (optimiser checkpoint constraint) | 0.42/0.45 | **11.033 / 11.260** | — | 0.527 | 1.05 | — | — | — | **rejected, see fact 4** |

**`apex_f42k` is the pick.** Floor 0.42 with small live-evidence margins —
```
--min-halfwidth 0.42 --side-margin-taper 0.6 --side-margin
  "19.8:22.5:right:0.15,25.8:28.0:right:0.13,44.3:4.5:right:0.15,
   36:41.5:right:0.15,8:17:right:0.12,23.2:25.6:left:0.13,
   42.0:43.4:left:0.15,29:34:left:0.10,18.3:19.8:left:0.06"
```
Per corner phase, line-to-wall against shipping: it **keeps** the three places
the live car is at the limit — s 37.6 exit R **0.619** (shipping 0.619), hp43
entry L **0.546** (0.555), straight R **0.550** (0.550) — **adds** where the car
is tightest — s 24 apex L **0.576** (0.534, +0.042) and s 24 exit L **0.690**
(0.621, +0.069) — and **spends** 0.03-0.09 m only where the audit found unused
width (s 22, s 24 entry, s 27, s 34 kink, s 40.9, the s 19 hairpin). It opens
the first hairpin to |kappa| 1.62 (R 0.62), the most the gate-7/8 limit allows,
and `apex_deviation` is 0.000 (shipping 0.035). In the harness it is clean at
both frames with **more** footprint clearance than shipping, the corner cut
falls +0.171 -> +0.102 at 0.058 and +0.075 -> +0.037 at 0.098, and it has the
best lap medians of the whole family (12.14 / 12.59 against 12.36 / 12.77).

**Two honest caveats.** (a) It needs `barrier_margin 0.34`, not 0.40 — feasible
margin 0.325. (b) **The flatter a line is, the worse mpc3 holds it**: |e_lat|
p95 goes 0.346 m (`f42k`) -> 0.41-0.47 (`f42a`/`f42i`/`f42e`), and the harness
lap medians go the wrong way (12.50-13.06 on the flat lines against 12.14-12.64
on `f42k`) even though their ideal laps are 0.03-0.14 s quicker. The tracker was
tuned around a line it cuts; a line it cannot cut is a different control problem.
`f42a` remains the honest upper bound of the geometry lever (−0.278 s ideal,
harness clean, best tracking error of the flat family) and is committed for that
reason, but it fails gate occupancy and the checkpoint margin and must not be
driven as-is.

---

## 6. Submitting

Build, test, push, submit. Nothing in this sequence is optional and **nothing
changes after Gate D**.

```bash
cd /home/ankit/code/data/simracing
TAG=2026-iros-compete

# 0. assets and entrypoint are already correct: FINALS_SLUG defaults to
#    finals_mesh in submission/Dockerfile and PIT_CREW_MAP defaults to
#    /home/data/maps/finals_mesh in submission/autodrive_devkit.sh.
FINALS_SLUG=finals_mesh .venv/bin/python -m pytest -q \
    tests/test_submission_assets.py -k FinalsSlug
.venv/bin/python -m pytest -q tests

# 1. build FROM the finals api image
docker build --platform linux/amd64 --build-arg TAG=$TAG \
  -f submission/Dockerfile -t bongracers/roboracer:iros2026-finals .

# 2. race the image as shipped, both documented entry paths, on Xvfb
./scripts/test_submission_image.sh bongracers/roboracer:iros2026-finals 12 $TAG
ENTRY_MODE=cmd ./scripts/test_submission_image.sh bongracers/roboracer:iros2026-finals 12 $TAG

# 3. and on the real display, the organisers' own setup
VISIBLE=1 ./scripts/test_submission_image.sh bongracers/roboracer:iros2026-finals 12 $TAG

# 4. read every run with its period before reading its collisions
.venv/bin/python tools/lap_report.py data/logs/submission_<ts>.csv \
    data/racelines/finals_mesh_race.csv data/maps/finals_mesh
.venv/bin/python tools/score_race.py data/logs/submission_<ts>.csv

# 5. publish and record the digest
docker push bongracers/roboracer:iros2026-finals
docker image inspect bongracers/roboracer:iros2026-finals \
  --format '{{json .RepoDigests}}'
```

Then: paste the DockerHub overview from `submission/README.md` into the
repository description (it names the finals track and gives the organisers' run
commands); if the repo is private, add DockerHub user `autodriveecosystem` as a
collaborator; **submit the exact `bongracers/roboracer:iros2026-finals` on the
Phase 2 form linked from the event page** — a push is not a submission, both must
be done before **Sep 19 AoE = Sep 20, 17:30 IST**.

**Gate D is the line in the sand.** Gate C = 100 Xvfb laps at ~0.097 s, 0
collisions; Gate D = 32 real-display laps at ~0.058 s, 0 collisions, on the exact
config being shipped. After Gate D passes, the digest is recorded and **nothing
in the image changes** — any further experiment goes on a separate branch. The
image that exists today (114.04 / 113.98 s display, 119.5 Xvfb, 119.57 as an
image) is the floor under the weekend; a faster arm replaces it only by passing
the same two gates.

---

## How we got here (Sep 17-18)

Kept for provenance; superseded by everything above.

**Sep 17, first run on the released track** (`race_20260917_134454.csv`, Xvfb,
dt_p50 0.083 s): 32 laps, **3 collisions**, median 11.862 s, best 11.390 s,
OFFICIAL 118.094 s. Cross-track p50 0.095 / p95 0.336 m (Porto 0.21), car 0.26
m/s under plan, driven median 7.6 % over the 11.02 s ideal (Porto: 0.6 %).
The first collision attribution was **wrong**: xy (0.72, −15.57) is *checkpoint
7*, and the row where `collision_count` steps is the respawn, not the impact.
With the c-2 rule (`tools/racelog.py`) every collision is at **s 18.07-18.57**.

**Sep 17, `finals_mesh_dk08`** (`--dkappa-max 0.8`, `race_20260917_135943`): 4
collisions, OFFICIAL 119.07 s + 20 s, same entry, same side — see section 4.

**Sep 18, this host (RTX 5090).** Xvfb baseline `race_20260918_154740.csv`: 32
laps, 0 collisions, OFFICIAL 117.13 s at dt_p50 0.094 s — but the cut was
unchanged (per-lap most-inside p50 **+0.444 m**, worst +0.496). The clean run was
just the tail sitting under the wall at that period. The real display then
returned 11 laps / 10 collisions, and the safe line and barrier 0.34 did the same
(section 3): a DQ on the organisers' setup, with the s 19 entry as the single
blocking problem.

**The finding that changed the game** was reading the simulator source: the brake
at throttle exactly 0.0 (section 2). It closed the entry cut from +0.35..+0.49 to
+0.19..+0.25 and turned three DQs into 32/0. Everything since has been an attempt
to spend that margin on lap time.

**Checkpoint numbering.** This file used to be a checkpoint list (0 baseline,
1 speed cap, 2 longitudinal, 3 grip, 4 line, 5 controller, 6 validation,
7 image). Checkpoint 0 passed with the brake; 1 was dropped; 2, 3, 4 and 5 are
folded into sections 4 and 5; 6 and 7 are section 6.
