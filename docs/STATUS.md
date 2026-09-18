# Status — measured, not assumed

## NEXT SESSION — START HERE (September 18: the finals config is frozen and clean on the display)

**`docs/FINALS_TUNING.md` is the finals state page** — frozen config, the
mechanism, every arm driven, what is closed, what is left, and the submission
commands. This block is the one-screen version.

**State:** the finals image for the SRL 2026 IROS track (`2026-iros-compete`)
is built, tested and ready to push as `bongracers/roboracer:iros2026-finals`;
submission due **Sep 19 AoE = Sep 20, 17:30 IST**. **Candidate: the same config
on the ay-6.75 profile of the same geometry (`finals_mesh_rt_dr2_ay675_race.csv`)
drove 32 / 0 / OFFICIAL 110.829 s on the display** (`race_20260918_172451.csv`,
best lap 10.937 s, driven ay p99 6.62, entry cut +0.22 p50). **It failed Gate C: 10
collisions in 24 Xvfb laps (0.098 s), all running wide at s 38.6**
(`race_20260918_173633.csv`), so it does not ship. A hybrid profile with the
frozen speeds at s 24-28 and s 35-41 (`finals_mesh_hyb675_race.csv`, ideal
10.82 s) **passed the 100-lap Xvfb gate: 100 / 0 / 117.72 s** (frozen: 119.51 s on
32 Xvfb laps; `race_20260918_174633.csv`); display **32 / 0 / 111.82 s**; it is the
entrypoint's finals line and the built image raced 117.26 s (cmd entry, clean),
118.40 s + 1 collision (default entry, at s 38.7, 0.093 s) and 112.09 s (display),
all 12 laps. That corner is marginal at the organisers' period on the frozen
speeds too, so `finals_mesh_hyb675b_race.csv` (0.08 m inward there, exit capped
at 3.0 m/s) replaced it: **100 Xvfb laps / 0 / 118.07 s with 0.566 m margin
at s 39-41, display 32 / 0 / 111.59 s**, and the rebuilt image raced 117.13
(Xvfb), 117.80 (Xvfb, cmd entry) and 111.74 s (display), all 12 / 0. That image
is `bongracers/roboracer:iros2026-finals` as of Sep 19 01:40 IST. Behind it,
two single-knob arms on the frozen line behind it (`servo_k_p 0.5 +
servo_head_max 0.3` for the corner exits; `pub_gate_corr 0.05` for the straight
weave — both opt-in, committed, defaults off, see FINALS_TUNING section 5). Note the frozen profile's own wall margin at s 39 is 0.25 m at
0.098 s (32 clean laps); its 100-lap Xvfb gate is still owed. ay 6.75 + lead 0.12 was neutral (114.10 s).

```
map / line   finals_mesh  ->  finals_mesh_race.csv   (mpc3)
CTRL_ARGS    -p map_prefix:=$MAP -p rate_adaptive:=true -p w_dcmd:=50.0
             -p cmd_tau:=0.10 -p steer_tau:=0.04 -p barrier_margin:=0.40
             -p thr_brake_lead_s:=0.05
             -p brake_mode:=zero -p brake_a_min:=1.5 -p brake_dv:=0.3
             -p brake_a_full:=4.6
LOC_ARGS     -p max_step_mps:=9.0 -p publish_on_reloc:=false
env          OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=1
```

Driven: **real display (0.058 s) 32 laps / 0 collisions / OFFICIAL 114.042 s**
(`race_20260918_162222.csv`) and 113.981 s on a second sample
(`race_20260918_165621.csv`); **Xvfb 32 / 0 / 119.508 s**
(`race_20260918_163645.csv`); the built image raced the organisers' way,
**12 laps / 0 collisions / 119.57 s**. Without the brake the same line, the safe
line and barrier 0.34 each took **10 collisions in 11-14 display laps** — a DQ.

### Corrections to the Sep 12 brief below (from reading the simulator source)

- **The brake is throttle exactly 0.0, not negative throttle.** Negative
  throttle is reverse drive; `VehicleController.cs` applies `BrakeTorque =
  MotorTorque` (428 N·m) only at `throttle == 0.0f`, and this stack had never
  published it (0.4 % of samples). `brake_mode:=zero` is what publishes it.
- **The tyre is linear to its extremum** (slip 0.01, 9.81 m/s², `TireFriction.cs`).
  The "sustained limit 4.6-6.6 m/s²" quoted in earlier briefs was chatter across
  the knee, not a ceiling; driven ay p99 is already 9.6. Planned ay is a real
  lever again, gated on driven ay p99 < 9.3.
- **The held brake ceiling is 4.55 m/s²** plus drag (constant torque locks the
  wheel in one physics step, so what is held is the longitudinal asymptote).
  Plan brake ≤ 5.5; **"`--brake` 7.0 is ready to ship" was never achievable** —
  it is unphysical, not merely unvalidated.
- Also from the source: drive torque is linear in throttle to 1.0 (11 m/s ≈ 0.44)
  but constant torque means acceleration decays with speed; steering slews at
  3.2 rad/s in physics time with no lag, independent of graphics quality (Ultra
  does not change the physics rate); there is no special startup mode, so the
  warm-up lap is free.

The Sep 12 brief below is otherwise still current: it is the controller-generic
stack the finals config is built on.

## September 12 brief — the stack the finals config is built on (still current except where the Sep 18 block corrects it)

One day, ~25 experiments. There is now a single shipping config that has driven
clean on the **real display (0.058 s)** and on **Xvfb (0.09-0.10 s)**, on all
four tracks. Superseded narratives are at the end under "How we got here
(Sept 12)" — read them only if a number surprises you.

### 1. What ships now

Everything below is an uncommented default in `submission/autodrive_devkit.sh`.

```
line     Porto slug -> data/racelines/porto_mesh_cr_dk0.8.csv
         any other slug (mpc3) -> <slug>_race.csv ; legacy mpc -> <slug>_safe.csv
CTRL_ARGS (mpc3)  -p map_prefix:=$MAP -p rate_adaptive:=true
                  -p w_dcmd:=50.0 -p cmd_tau:=0.10
                  -p steer_tau:=0.04 -p barrier_margin:=0.34
                  case *cr_dk0.8.csv -> -p barrier_margin:=0.40   (last -p wins)
LOC_ARGS          -p max_step_mps:=9.0 -p publish_on_reloc:=false
env               OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=1
```

Why each is in: **`rate_adaptive`** scales `apply_ahead_s` (a constant 0.090 s,
0.87 of the 0.104 s period mpc3 was tuned at) with the measured period — 7
collisions/32 at a 0.090 s instance became 2, 100 Xvfb laps clean at 0.091-0.098 s.
**`w_dcmd` 50 + `cmd_tau` 0.10** damp the solver's steering limit cycle (display:
feedback sign flips 5.5 -> 2.4/s, command step p90 0.14 -> 0.041 rad).
**`steer_tau` 0.04** models the rack's first-order lag in the rollout and buys back
the tracking `cmd_tau` costs on tight circuits (CDC-TF 116.7 -> 112.0 s, ICRA 2026
150.4 -> 148.3 s); keep it at 0.04, 0.06+ is a mismatch that collides.
**`barrier_margin`** is a corner-ENTRY outward bias: at the hairpin apex 99 % of
solves already sit at the `ay_limit` cap (0.161 rad) and the barrier is clipped
out, so it acts 1.5-2.5 m earlier (+0.008 rad at 0.34, always outward).
**`max_step_mps` 9.0** removes the motion model's 5.77 m/s cap (0.6 m per update at
`rate_nominal` 0.104), which was a DQ on SRL 2025 ICRA; **`publish_on_reloc:=false`**
suppresses the one pose after a global reinit (measured 13 m from the car twice).
**Single-threaded BLAS**: mpc3's numpy otherwise runs at 1030 % CPU and starves the
simulator, changing the period.

**Real display, Ultra, 0.058-0.059 s** (32 laps each, OFFICIAL race = sim lap
timer + 10 s/collision):

| Run | Config | Result | Log |
|---|---|---|---|
| `disp_bm40` | **exactly the shipping Porto config** (bm 0.40 + steer_tau) | 32 laps, **0 collisions, 70.131 s**, median 7.000; hairpin most-inside p50 -0.261 / worst -0.296, **0/32 laps below -0.30** | `race_20260912_102205.csv` |
| `disp_bm34_tau` | shipping with bm 0.34 | 32, **0**, **69.357 s**, median 6.948; most-inside p50 -0.297 / worst -0.338 (13/31 below -0.30) | `race_20260912_083832.csv` |
| `disp_bm34` | bm 0.34, no steer_tau | 32, **0**, 69.444 s — the first ever clean display run | `race_20260912_080935.csv` |
| `disp_ndpin` | + opt-in `nd_delay_s` | 32, **0**, **69.244 s** (best race of the project); margin NOT improved (16/32 below -0.30) | `race_20260912_101101.csv` |
| `thr_bl05` | + opt-in `thr_brake_lead_s:=0.05` | 32, **0**, 69.943 s; most-inside p50 -0.290 / worst -0.310 (8/32 below -0.30) | `race_20260912_102857.csv` |

The wall sits at about -0.35 m. 0.40 buys +0.04 m of real margin for +0.8 s of
race; on a pass/fail qualifier and an unknown final that trade is right.

**Xvfb, Ultra, 0.09-0.10 s** (32 laps unless stated):

| Track | Line | Result | Log |
|---|---|---|---|
| Porto | `cr_dk0.8`, bm 0.40 + steer_tau | **100 laps, 0 collisions, 71.910 s**, median 7.221 | `race_20260912_103418.csv` |
| Porto | `cr_dk0.8`, bm 0.34 | **100 laps, 0, 71.496 s** | `race_20260912_091004.csv` |
| CDC-TF 2025 | `safe`, bm 0.34 | 32, 0, 117.265 s | `race_20260912_084350.csv` |
| CDC-TF 2025 | `fast2_m50`, bm 0.34 + tau | 32, **2 collisions**, 101.873 s (both at s 39.18) | `race_20260912_093952.csv` |
| CDC-TF 2025 | `fast3_m53`, bm 0.34 + tau | 32, 0 collisions, but **one missed checkpoint** -> 112.089 s | `race_20260912_104735.csv` |
| SRL 2025 ICRA | `safe`, bm 0.34 | 32, 0, 149.769 s | `race_20260912_085120.csv` |
| SRL 2025 ICRA | `fast2_m50`, bm 0.34 + tau | 32, **0, 140.422 s** (-9.4 s/race vs safe) | `race_20260912_094731.csv` |
| ICRA 2026 | `safe`, bm 0.34 + tau | 32, 0, **148.321 s** (150.434 without tau) | `race_20260912_095621.csv` |
| Porto | legacy `mpc` on safe, 11 Hz and Ultra | 32, 0, 78.212 / 78.395 s — the rate-proven fallback | `race_20260912_022719.csv`, `..._023408.csv` |

Baseline for scale: main branch 89.6 s; the published winning Porto lap is
**6.46 s** (VAUL) for a ~65 s race.

### 2. Finals day

`docs/FINALS_RUNBOOK.md` is the procedure — the Sept 16-19 checklist at the top,
then steps 0-11, with Gates A/A2/B/C/D/E attached to the runs. Track drops
**Sep 16**, submission due **Sep 19 AoE**. Line generation is one command:

```bash
./scripts/finals_drill_mesh.sh finals $TAG "<track name>"
```

Stage 5/5 runs `tools/pick_config.py`, which measures the chosen line and prints
the `PIT_CREW_CONTROLLER_ARGS` to race with (`barrier_margin` 0.40 only where the
line's feasible cap — min half-width minus 0.135 m — clears it; `thr_brake_lead_s`
0.05 on a tight circuit, never 0.08). Pass = 0 collisions over 100 Xvfb laps
(Gate C) **and** 0 over 32 real-display laps at ~0.058 s (Gate D).

**CHECKPOINT (Sept 12, end of session; the user asked to stop cleanly):**
qualification is secure (pass/fail on collisions; the built image raced
the organisers' way on the real display at 70.06 s clean), the finals
work resumes on Sep 16 with the released track. Steer_tau 0.02 vs 0.04:
display 69.88 s with the best hairpin margin; circuit regression at
0.02 clean on CDC-TF (113.4) and ICRA 2025 (142.3, +2 s), ICRA 2026
150.1 (+2 s), Porto Xvfb 32 laps with 1 collision (71.6 s) where 0.04
had 100/0. **Decision: steer_tau 0.04 stays shipped**; 0.02 is the
display-preferred alternative (PIT_CREW_CONTROLLER_ARGS="-p
steer_tau:=0.02") if the organisers' period turns out to be ~0.058. Open items for the next session, ranked:
1. The line generator fails on a track with a corner tighter than the
   steering lock (SRL 2024 IROS: 24/24 candidates rejected, safe line
   0.265 m effective clearance). A partial attempt is saved as
   docs/wip/2026-09-12_partial_tight_track_and_ftg.patch (NOT applied).
2. follow_the_gap fallback audit (offline LiDAR harness, direction
   lock, corridor-adaptive bubble): partial work in the same patch.
3. Corridor sign fix in optimize_raceline.py (committed after
   check_pipeline): one-sided bounds now land on the right wall; the
   "0.53 asked, 0.443 held" claim did NOT reproduce (pre-fix held 0.522,
   post-fix 0.527; the residual is the post-QP smoothing). With the
   corrected bound icra2026's width-using candidate no longer survives
   the gates (it ships safe anyway).
4. The organisers' telemetry period is still unknown; the shipping
   config is clean at 0.058 (real display) and 0.09-0.10 (Xvfb).
5. Ask the organisers (Discord/email) whether the Sep 19 form accepts a
   different image from the qualification one (the rules only say the
   phases are "handled separately").

**Late Sept 12 additions (after the brief was written):**
- The CDC-TF missed lap was NOT a skipped gate: checkpoint triggers are
  width x 0.254 x 0.100 m boxes (scene), the car passed every one inside
  its extent (min margin 0.459 m, same as counted laps), and the sim's
  own lap_time ran through the crossing -- a sim-side trigger fault, 1 in
  32, unique among 61 logs. A checkpoint constraint now exists anyway
  (`--checkpoints`, pick_config reports the margin and exits 2 below
  0.35 m); fast4_cp == fast3_m53 byte for byte.
- MECHANISM of the missed lap (sim source, Tinker-Twins/AutoDRIVE
  LapTimer.cs, matches our build): a lap counts only if CheckpointCount
  reached 20, which needs cp20's ENTER accepted while
  PreviousCheckpoint == 19; CheckpointFlag is ONE global latch cleared by
  ANY trigger exit, and PreviousCheckpoint is overwritten on every exit.
  The car (0.50 m, ~15 colliders each firing its own enter/exit) must be
  inside at most one gate box at a time. At the CDC-TF hairpin gates
  17-20 are 0.26 m apart along fast3_m53 (safe line: 0.39-0.41), so the
  car straddles 2-3 boxes every lap (0.13 s/lap of triple occupancy);
  one lap in 32 the cp17 exit landed after the cp19 enter and cp20 was
  dropped. Rule for lines: consecutive gate planes >= 0.60 m apart along
  the line (car length + box), zero triple occupancy, never re-touch a
  gate after passing it, especially the last three before the finish.
  Gate separation collapses with |lateral| (the gates fan out), so the
  lever is the line's lateral position at converging gates, not speed.
  An uncounted lap costs ~one lap time (10 s): weight it like a
  collision. Agent implementing the occupancy metric + constraint.
- Gate-separation constraint built (committed): measured off the line
  geometry, fast3_m53 has 0.388 m between gates 19 and 20 and straddles
  two boxes 271 ms/lap; the safe line 0.558 m / 97 ms; Porto 0.96 m,
  ICRA 2026 1.61, ICRA 2025 fast2 1.05 (all occupancy 1). On CDC-TF every
  width-using line that satisfies the constraint fails the kappa gate,
  so **CDC-TF ships its safe line** (the fast line's 9 s/race vs a 3%/lap
  chance of a ~10 s uncounted lap: the drill now refuses it). Floor set
  to 0.55 m (0.60 would condemn the safe line). Two tool bugs found:
  corridor_of is mirrored vs tph's alpha sign (one-sided bounds broke;
  --min-halfwidth's QP restatement is affected, explains "0.53 asked,
  0.443 held"), and a 0.05 m keep-floor discarded half of one-sided
  bounds. Open: the mirror fix.
- steer_tau x barrier sweep (7488 harness laps): tau 0.02 keeps 88% of
  the tight-track gain and 0.03 keeps 95% while Porto's worst clearance
  goes 0.000 (tau 0.04, bm 0.34) -> 0.075 (0.03, bm 0.40) -> 0.103 (0.02,
  bm 0.40); tau 0 is dangerous on CDC-TF (26 collisions/144 on the pole
  plant); barrier 0.40 is free-or-better wherever the cap is >= 0.385
  (safe lines, fast3_m53) and worse at cap 0.345 (icra2025 fast2_m50).
  pick_config now takes 0.40 at cap >= 0.385. **Display A/B: Porto
  steer_tau 0.02 + 0.40 = 32/0, OFFICIAL 69.88 s, hairpin most-inside
  p50 -0.257 / worst -0.275 (0/32 below -0.30)** vs 0.04 + 0.40 = 70.13 /
  70.01 s, worst -0.296 / -0.313 (0 and 2 laps below -0.30): faster AND
  more margin, as the sweep predicted. Circuit regression at 0.02
  running (chain9); if clean, steer_tau 0.02 replaces 0.04 in the image.
- Stall-test control (display, bm 0.40 + steer_tau 0.04, no stalls):
  32 laps, 1 collision on lap 32 at s 19.07 (offset -0.23 m at 4.35
  m/s, a different corner from the hairpin), 70.01 s. With the earlier
  0/32 that is 1/64 display laps for this config; s 19 is inside the
  17.5% of Porto stations where 0.40 exceeds the line's own clearance.
- Stall arms: data/logs had become root-owned (a container write), so
  stall_inject.sh could not open its log and injected NOTHING during
  the `stall_base` arm (32/0, 70.19 s, zero telemetry gaps: it is a
  second clean control, not a stall test). Ownership fixed; the gap
  and both arms of chain7 log stalls; the baseline-with-stalls arm is
  re-queued.
- stall_gap arm (8 sim pauses of 0.4-0.63 s delivered for 0.33 asked,
  gap_predict_s 0.6, no localization gap params): 32 laps, 0
  collisions, but one 21.2 s lap: 7 s after the second stall the car
  STOPPED at s 27.1 for 13 s (throttle 0, steering held, no collision)
  and resumed by luck -- the coast watchdog after localization declared
  itself lost post-stall and, with publish_on_reloc false, published no
  pose. Exactly the predicted localization failure; the `both` arm (with
  gap_dt / gap_noise_mult / lost_patience_min / scan_dt_max / enc_gap_max)
  is the real test.
- **stall_both arm (gap_predict_s 0.6 + the localization gap set; 8
  sim pauses of 0.4-0.6 s delivered): 32 laps, 0 collisions, longest lap
  7.54 s, OFFICIAL 70.18 s.** The package rides out stalls that stop the
  car for 13 s without it. Candidate for the finals image after a
  no-stall Xvfb regression (Porto + one circuit, queued in chain8), plus
  a stalls-with-defaults arm for the comparison (the earlier one had no
  stalls injected).
- Finals-day rehearsal, offline, fresh slug (SRL 2024 IROS Track): the
  drill runs in 55 s and pick_config/entrypoint derivation work, but
  (a) stage 4 had been crashing silently on every track (fixed), (b)
  pick_config reported raw clearance 0.526 where the line's effective
  clearance was 0.265 (fixed: both printed, warning below 0.28), (c) the
  finals slug's racelines now ship via a FINALS_SLUG build ARG, and (d)
  **the line generator failed on that track: 24/24 raceline candidates
  rejected (direction reversals / corridor exhausted) and the safe line
  sits at 0.265 m effective clearance, below the floor** -- on a track
  like that the drill would ship an unproven line. Open; agent on it.
- Qualification context (ICRA 2026 standings, same Porto track, 28
  teams, all qualified -- there is no time cutoff, only the collision
  tolerance): 70 s would have placed 2nd behind VAUL (64.80) and ahead
  of DEWO SPEED (74.67). The final was won by the team ranked 9th in
  qualification on a slower unseen track (110 s race, 11 s laps); the
  qualification winner was outside the final's top ten.
- **stall_base2 (shipping defaults, 8 real stalls of 0.4-1.0 s): 32
  laps, 0 collisions, longest lap 7.52 s, 70.27 s.** A sim pause freezes
  physics and telemetry together, so the defaults' 0.35 s coast is
  harmless live; the harness's dropped-frame collisions do not transfer
  to this stall shape. gap_predict_s ALONE caused the 13 s stop; the
  full package was clean; the defaults were clean. **Decision: the gap
  package stays opt-in (prepared GAP_BLOCK, type-checked test); not
  shipped** -- no live evidence it helps, one live case it hurt. No-stall
  Xvfb regression with the package: Porto 32/0/72.08 s, ICRA 2025 fast2_m50
  32/0/140.2 s (clean, no cost either way).
- **Image test on the real display (the organisers' path: built image,
  --entrypoint /bin/bash, no mounts, helper clicks; period 0.058):
  12 laps, 0 collisions, OFFICIAL 70.06 s**, /localization and /mpc3 up,
  the extra-shell check passed. First image-as-shipped run since today's
  config; log data/logs/submission_20260912_170708.csv. (The
  submission_..._074832 log is this morning's test at the older config.)
- **Turn-in probe, both rates (display 0.0583 s vs Xvfb 0.0898, 20
  pulses each):** the total pose-to-yaw-onset delay is RATE-INVARIANT
  once the tick ZOH is subtracted: 0.099 vs 0.096 s (d -0.003 s = -0.09
  x the period change); publish-to-rack moves +0.013 s (+0.42 x the
  period change, i.e. the tick term), rack-to-yaw compensates. So the
  plant's transport is "one period + ~0.10 s" and the early turn-in at
  0.058 is NOT a transport-delay difference; the delay_tick law is
  confirmed at the plant level.
- Turn-in probe, first valid run (Xvfb, 0.090 s period, 20 pulses,
  data/logs/turnin_20260912_111057.csv): publish-to-rack dead time
  0.094 s (= period + 0.094 raw 0.184), rack-to-yaw lag 0.0004 s, total
  pose-to-yaw-onset 0.096 s corrected / 0.186 raw = 2.05 periods. The
  display run is queued for the rate comparison (the robust output).

### 3. Open, ranked by finals risk

1. **Missed checkpoint on the CDC-TF `fast3_m53` line.** 32 laps, 0 collisions,
   laps 10.1-10.4 s — but one 20.4 s "lap": two start-line crossings with no
   `lap_count` increment. The rules stop the lap timer only when every
   checkpoint is passed, so OFFICIAL was 112.089 s instead of ~102. No other
   Sept 12 log (40 checked) shows the signature, so it is this line at one
   checkpoint. **Finals hazard of the first order**: the line generator must
   respect the checkpoint trigger extents, and `pick_config.py` must report the
   minimum checkpoint margin and fail loudly. *An agent is in flight on this.*
2. **The stall test has never been run live.** `scripts/stall_inject.sh` (docker
   pause of the sim — the organisers' risk shape, ~0.07 s overhead) is ready;
   the four-arm test is control / stalls with defaults / + `gap_predict_s:=0.6` /
   + the localization gap args. Harness evidence only so far (section 5).
3. **The image has not been rebuilt or raced organiser-style since today's
   config landed.** The last image test was 14 laps / 0 collisions / 70.82 s on
   the *old* `mc0.60_ay6.5` line and the pre-Sept-12 controller. Runbook steps
   6-9. A 14-lap sample scatters ~1.5 s; do not read it as a validation.
4. **Turn-in timing probe: unrun.** `ros2 run pit_crew turnin_probe` /
   `scripts/turnin_probe.sh` measures pose-stamp-to-yaw-rate delay inside the
   race loop; the rate comparison (`tools/turnin_fit.py --compare display.csv
   xvfb.csv`) is the robust output. It was attempted on the display and produced
   **0 pulses, and that log was lost.** Re-run `VISIBLE=1 ./scripts/turnin_probe.sh`
   then the Xvfb one. Lowest risk of the four: nothing ships on its result.

### 4. Rejected today (one line each, with the number that killed it)

| Knob | Killed by |
|---|---|
| `apply_floor_steps:=0.0` + `rate_terms:=apply,thr,delay,predict,enc,pose` | Display 15 laps, **11 collisions**, same hairpin — it fixed the slew (0.078 -> 0.003) which was a symptom. |
| `lead_mode:=spatial` | Display **11 collisions in 10 laps** (with steer_tau, 10/10). |
| `steer_tau:=0.04` alone (no barrier) | Display 10 collisions in 30 laps — no better than smoothing alone. |
| `w_dcmd:=100` alone, no `cmd_tau` | Fastest anywhere (Xvfb 69.393 s) and **10 collisions in 12 display laps**. |
| `nd_mode:=period` | +0.5..+2.2 s/lap on the tight circuits, **61/432** harness collisions on ICRA 2025 at four corners. |
| `control_mode:=timer` (fixed 20 Hz solve) | +0.25 s/lap and 4-6 collisions in 33 laps at *every* period. |
| ILC (`ilc_*`) | Clean 32/0 at 0.092 s, but **7 collisions in 100 laps** — 5 of them in the 20 laps after the host drifted 0.092 -> 0.108 s. A table learned at one period goes stale when the period moves. |
| CMA-ES "rate-invariant" parameter set (`rateinv`) | Driven 32/0/70.899 s, but the margin claim did not transfer: \|e\| p95 **0.226 vs 0.182** m, min clearance 0.127 vs 0.179 m. |
| `lon_ax45_b40` (honest ax 4.5 profile) | Display 32 laps, **1 collision**, 70.619 s, median 7.063 vs the candidate's 6.95 clean. |
| `thr_lead_mode:=min` + `thr_lead_s:=0.45` (`lead45`) | Display 32/0 but **72.869 s** (+0.33 s/lap) for +0.02 m of margin. |
| `rate_slew` | The period-range data does not support widening the `rate_lo`/`rate_hi` clamp. |
| `grip_watchdog` | Costs more race time than it saves in every configuration. |

### 5. Opt-in, validated, unshipped

Each is one `-e PIT_CREW_CONTROLLER_ARGS` / `PIT_CREW_LOCALIZATION_ARGS` away
(they are appended last and win; `tests/test_entrypoint_args.py` asserts it).

| Knob | Flag | Evidence |
|---|---|---|
| Telemetry-gap ride-out (controller) | `-p gap_predict_s:=0.6` | Harness, stall every 5 s, 72 laps/cell: 3-frame gaps 6 -> 1 (slew) / 19 -> 8 (pole); 5-frame 18 -> 3 / 46 -> 18. No-gap reports byte-identical. **Never driven live.** |
| Telemetry-gap handling (filter) | `-p gap_dt:=0.15 -p gap_noise_mult:=3 -p lost_patience_min:=3 -p scan_dt_max:=1.2 -p enc_gap_max:=1.5` | Two 0.5 s constants coincide by default, so above a 0.5 s gap the filter dead-reckons at the stale speed: 0.9 m of phantom travel for a stalled car, 2.25 m error + 9 relocalizations for a 1 s gap. Fixed: worst post-gap error **0.46 m**. |
| Teleport detector | `-p teleport_scan_jump:=0.35 -p teleport_imu_g:=0 -p teleport_hold_n:=7 -p teleport_reseed:=false` | On the real collision bag: **9/9 resets detected within one frame, 0 false positives** for any threshold 0.20-0.525 (clean max 0.194). IMU branch unusable (1/9). Peak post-reset error 0.65 -> 0.30-0.38 m; reseed helped on 0/9 and costs 0.06 m. |
| Brake-phase throttle lead | `-p thr_brake_lead_s:=0.05` | Removes 93-100 % of the commanded entry overspeed, Porto -0.018 s/lap, \|e\| p95 -24..-38 % (288 harness laps, 0 collisions); display 32/0 69.943 s. But **+0.30 s/lap on CDC-TF, +0.23 on ICRA 2025** — not free off Porto. |
| Dead-time pin | `-p rate_terms:=apply,thr,predict,enc,pose,delay_tick -p nd_delay_s:=0.180` | Display 32/0 **69.244 s**, the best race measured — but the hairpin margin is not improved (16/32 laps below -0.30 m). |
| `apply_lead_steps` | `-p apply_lead_steps:=1.2` | Lap-neutral, removes 10-15 % of the hairpin cut; 1.0 removes most of it at +0.12..+0.31 s/lap with pole-plant collisions. Not run live. |
| Faster localization cap | `-p max_step_mps:=12.0` | Only if Gate B says the finals profile asks for more than 9 m/s. Zero lap cost at any cap >= 6.4 in the harness. |
| SRL 2025 ICRA fast line | `LINE_ARGS='... --min-halfwidth-ladder 0.50'` | `fast2_m50` drove 32/0/140.422 s against the ladder's 0.53 rung; 0.53 buys 0.041 m of clearance for 0.40 s/lap. |

### 6. Standing rules learned today

* **Queue rules.** A scored run needs a quiet host: `run_exp.sh` refuses above
  load 12, and a 150-lap re-run of a 0/100 config took **9 collisions** while
  three build agents computed. Any run taken under load is contaminated — re-run
  it. Never run an offline search during a live run.
* **Every run is evidence only together with its `dt_p50`.** The sensor period
  is a per-start lottery of the simulator on this host (0.088-0.106 s on Xvfb,
  flat within a run, independent of graphics quality); `RATE_MAX_DT`/`RATE_MIN_DT`
  on `run_exp.sh`/`run_stanley.sh` restart the sim until the period is in the
  wanted band. Read it with `tools/lap_report.py`. A run at one period says
  nothing about another.
* **Log column semantics.** `x,y` are the IPS **truth**; `px,py` are the
  **estimate** (a swap produced a wrong root cause today). `steering` is the
  actuator **feedback**, `cmd` is the published command. The logger writes a
  **one-frame-stale pose**, so de-stale before comparing rates; its speed column
  is an alpha-0.30 filter and lags differently at each period. On a collision the
  logger writes one already-teleported row *before* `collision_count`
  increments, so the pre-impact sample is **row c-2**. `tools/racelog.py` is the
  one loader that gets all of this right. Score only with
  `tools/score_race.py`, quoting the OFFICIAL line.
* **Harness limits.** It **cannot reproduce the display collisions** (0 at 0.058 s
  with shipping params) — the display is the gate, never tune against the harness
  for that. It ranks lines and rate terms usefully but does not predict absolute
  tracking error (~10x optimistic on collisions at 0.088), so do not tune cost
  weights on it. Its **seeds do nothing without pose/yaw noise**. And
  `--pose-latency` equal to `--frame` lands the pose on the tick boundary and
  **fakes collisions** (the 0.045 default poisons frame 0.045).
* **Graphics quality does not set the sensor rate** on this host; the display
  does (Ultra on the real display is still 0.058 s). Validate at Ultra anyway —
  quality moves the Xvfb period by ~4 %.
* **Rules of the race** (verified from the 2026 rules page and the organisers'
  repo): Porto qualification is a pass/fail **gate** only, no carry-over; ranking
  is total race time on one unseen track, +10 s per collision, **11+ = DQ**,
  warm-up lap not counted, sim lap timer, tie-break on best lap. Organisers run
  Ultra, GUI, windowed on a real display (OBS recording), i9 14900K / RTX 4090,
  a human clicks Connection, and they start the image with no environment — so
  **anything that must ship, ships as an uncommented line in the script.**

### How we got here (Sept 12)

Kept for provenance; each of these is superseded by the sections above.

* **The rate failure.** mpc3 was validated at ~9.6 Hz (logger dt 0.104-0.106) and
  collided at ~11 Hz (dt 0.088-0.091): 3 collisions in 10 laps with BLAS pinned,
  7/32 at a 0.090 s instance (`ctl_fast3`), 2/100 at Ultra. Two wrong causes were
  chased and retracted: a "1.45 m speed lag" (it was the logger's own speed
  filter; ground truth lags 0.22 m) and a "0.15 m forward pose offset" (logger
  staleness; the consumed pose is rate-invariant to 0.007 m median). The real
  term was the fixed `apply_ahead_s` 0.090 s — 0.87 of the period at 0.104, 1.02
  at 0.088 — which `rate_adaptive` scales. With it plus the `cr_dk0.8` line:
  32/0/70.748 s, then **100/0/70.515 s**.
* **Four failed display runs, one cause.** The shipping config at 0.058 s took 10
  collisions/23 laps, 11/15, 11/20, 10/9 — always the same hairpin entry
  (-1.6,-6.5), cte -0.34..-0.37 m at 2.5-3.0 m/s, localization innocent (de-staled
  \|err\| p50 0.037 m). The `apply_floor_steps` fix removed the command-slew
  saturation and changed nothing. The approach table (display 0.058 vs Xvfb 0.094)
  showed identical true speed and identical published steering but lateral offset
  diverging from s 8.5 (-0.08/-0.13/-0.19/-0.24/-0.30 m vs
  0.00/-0.03/-0.06/-0.12/-0.18) — which looked like a rate-dependent plant.
* **It was not a plant.** Direct measurement: the rack moves exactly 3.2 x period
  per tick at 0.058 *and* 0.100 s (a wall-clock slew limiter), dead time
  0.07-0.10 s at both, actuator-to-yaw-rate gain 0.93-0.95, throttle servo K
  12-16 with tau 0. The wiggle was a **solver limit cycle**: the first free plan
  step after the `nd` committed steps has no jerk cost, so a one-step flick at the
  slew limit (exactly `steer_rate*step_dt` = 0.192 rad) is the cheapest answer to
  any small error and is never executed before the next solve flicks back. The
  peer-stack research (`docs/research/steering_smoothness_peer_stacks.md`) says
  the same: nobody else re-optimises per raw localization update. `w_dcmd` +
  `cmd_tau` fixed the flutter (display 27 laps, 10 collisions — still the
  hairpin), and `barrier_margin` 0.34 fixed the hairpin.
* **Dead-time law.** Pose-stamp-to-rack delay = **period + 0.052 s** (RMS 0.006 s
  over six logs); the devkit bridge emits the command as the last statement of
  the telemetry callback, so a command answers the next tick, and a publish off
  the tick grid is dropped. Per-pose solving is information-optimal on this bridge.
* **The line is converged.** 76 Porto candidates (w_veh 0.50-0.80 x ay 6.75-7.5 x
  dkappa 0.5/0.6/0.8), 342 harness laps per finalist, 0 collisions on all 33
  gate-passing lines; field spans 0.10 s/lap; ay 7.25+ turns back. Driven,
  `dk06_w063_ay70` 32/0/**69.916 s**. Under the identified actuator `cr_dk0.8`
  wins on clearance (0.090 m vs 0.025-0.056). The remaining ~0.5 s to the winners
  is not in the line: of the plan-side gap to 6.46 s, ~90 % is the lateral cap
  (ay 9.0 alone takes ideal 7.036 -> 6.352) and ~10 % longitudinal.
* **Lap-time budget** (display log `race_20260912_083832.csv`, 32 laps, median
  6.944): the car beats its own ideal lap (7.036) by 0.096 s by driving 0.857 m
  less track (hairpin -0.45 m of path) while losing +0.161 s longitudinally. The
  drive ceiling is the sim's speed servo saturating (K 15-17, a_max 4.0-5.0,
  a_min -6.5) at 4.2-4.7 m/s2 against the profile's ax 6.5 — the profile's extra
  ideal time is time the car never had.
* **Tight circuits.** The drill's absolute `MESH_RACE_KAPPA_MAX` 1.0 gate rejected
  every raceline on both 2025 circuits while shipping centerlines that are tighter
  still (cdctf safe kappa max 2.10, icra2025 1.63). A relative gate plus
  `--dkappa-max 1.5 --dkappa-refine 3 --kappa-bound 1.0` and
  `--min-halfwidth` gave the first fast lines on which `barrier_margin` 0.34 is
  feasible (cdctf `fast2_m50` ideal 10.03 / clearance 0.364 / min half-width
  0.493; icra2025 13.84 / 0.348 / 0.481). Lesson from the failed m45 family (25
  collisions at one corner): easing a corner's curvature lets the profile spend it
  all on speed the tracker cannot deliver — **geometry and speed must be capped
  together**. Porto and ICRA-2026 outputs stayed byte-identical (52 CSVs);
  `check_pipeline.sh` passed.
* **The SRL 2025 ICRA DQ.** 4 laps / 14 collisions with cross-track reading 0.03 m.
  The first diagnosis blamed the encoder and was wrong (the log's columns were read
  swapped); the cause was the localization estimate saturating at 5.1 m/s on a
  13.9 m stretch above the cap. Fixed by `max_step_mps:=9.0`; driven 32/0/148.761 s.
* **Collision-reset recovery audit** (100 live collisions, 21 runs): the reset is
  the last numbered checkpoint, 0.51 m p50 off the line, backwards on 50 % of
  resets (up to 4.5 m); \|est-truth\| < 0.15 m after 1.20 s p50, back to plan speed
  after 0.94 s, +0.5 s p50 beyond the 10 s penalty, **9 % cascade within 3 s**.
  The old `SpeedEstimator` teleport detector fired on 0 of 97 teleports, so its
  "stand still and spread" branch and mpc3's `consume_jump` are dead code.
* **Period-range audit** (7128 harness laps): the candidate is collision-free on
  all six track/line cells and both plants over 0.045-0.104 s, and through 0.125
  once pose noise is on; it degrades above 0.125 and collides on four of six cells
  at 0.150. Mid-run drift (step, ramp, sawtooth): 0 collisions in 432 laps vs 17
  for the pre-barrier config. **Three consecutive dropped frames (0.40 s) exceed
  `max_predict`/`stale_pose` 0.35 s** and put the car on the coast watchdog:
  35-40 collisions / 36 laps when repeated; one dropped frame is fine.
  Regression attributed: `steer_tau` 0.04 costs Porto clearance at 0.045-0.075
  (0.250 -> 0.050 m) and doubles \|e\| p95 on the icra2025 safe line at 0.097-0.104
  (0.138 -> 0.216) for 0.08-0.10 s/lap — live evidence keeps it, and the barrier
  is the counterweight.
* **The hairpin cut is the publish lead.** From a zero-error state the solver plans
  no cut at all (e_min -0.003 m at every entry, unchanged by `w_lat_T`, horizon
  30/40, `e_free` 0, `yaw_gain` 1.0, `ay_deliver`, `w_barrier` 0); the published
  angle is read nd + apply steps into a ramping-curvature plan and exceeds the
  line-matching angle by 0.04-0.33 rad. `apply_ahead` 0 removes it (-0.179 ->
  -0.006 m) for +0.39 s/lap; `e_free` is refuted on the logs and the understeer map
  is right to 5 %. The corner-local lever (inside half-width) is the right one.
* **Superseded shipping lines**, kept so the history below reads correctly:
  `mc0.63_ay6.75` (100 laps, 0 collisions, 72.16 s) and `mc0.60_ay6.5` (71.63 /
  72.49 s) were both shipping earlier; `cr_dk0.8` replaces them. w_veh 0.63 at
  ay 7.0 gave best lap 6.71 s but 4 collisions/32 — ay 6.75 is the limit for this
  tracker on this line family.

## September 13: image as shipped 70.8 s, 100 laps clean twice

**Shipping: mpc3 defaults on `porto_mesh_mc0.60_ay6.5.csv`, mesh map,
single-threaded BLAS. 100 laps clean at 71.63 s and again at 72.49 s
(sim timer 72.02); the image raced as the organisers run it: 14 laps, 0
collisions, 70.82 s on the simulator timer, best 6.98 s.** Baseline main
89.6 s; winners 65 s. Rejected since: the ay 6.75 line (2-3/100 at the
s 11.2 hairpin entry) and the CMA "smooth" parameter set on the shipping
line (5/32, same corner). That entry is the next limit; single-parameter
isolations of the search's knobs: ay_budget 6.57 alone -> 2/32; horizon 30 +
step_dt 0.040 alone -> 2/32 but median 7.06 (fast); both collide at the s
11.3 hairpin entry arriving at ~5 m/s one second before a 2.9 m/s corner,
i.e. a late/weak braking point, not a steering problem. Testing horizon 30
with earlier braking: a_brake 3.5 -> 9/32, thr_lead_s 0.30 -> 7/32, w_rate
1.9 alone -> 0/32 at 72.29 (neutral). Horizon 30 is dead live. The shipping
defaults are the local optimum on this line; next is the line's own braking
profile (planner --brake 3.5) into that hairpin: 32 laps, 0 collisions,
71.90 s, median 7.18 -- neutral. Porto has converged at ~71-72 s clean with
this line and tracker. Wider line w_veh 0.63 at ay 6.75 (0.309 m clearance):
32 laps, 0 collisions, 71.30 s, median 7.17, best 6.95 -- same time with
more room; 100-lap validation queued. ICRA-2026 with the pinned mpc3 on its
centerline: 30 laps, 0 collisions, 148.8 s (90 clean laps total on that
config across three runs).

## September 13: 71.6 s / 100 laps clean

**Shipping config on this branch: mpc3 on `porto_mesh_mc0.60_ay6.5.csv`
(mesh map, lidar_offset 0.1933). 100 laps on an idle host, 0 collisions,
first ten 71.63 s (simulator timer 71.18 s), median 7.28 s, best 6.98 s.**
Baseline main 89.6 s. ay 6.0 on the same line: 0/100 at 73.46 s (the
fallback). The image with the ay 6.0 line raced as shipped at 73.57 s (sim
timer), 14 laps clean; re-run `scripts/test_submission_image.sh` after the
ay 6.5 promotion (first attempt under an offline search load: 14 laps, 0
collisions, 75.21 s sim timer with three stalled 8 s laps; re-run on a quiet
host pending). The mesh pipeline's automatic profile stays ay 6.0 for
unseen tracks; Porto ships the explicitly validated ay 6.5 file.

Image with the ay 6.5 line, quiet host, as shipped: 14 laps, 0 collisions,
**72.94 s sim timer**, PASS.
Offline CMA (seeds 1-3, validated on 4-6) on the ay 6.75 wide line
(`porto_mesh_cand2_w0.60_ay6.75_race.csv`, same geometry as the shipping
line): e_free 0.193, k_e 0.654, barrier_margin 0.211, w_lat 8.07, w_rate
1.906, ay_budget 6.566, horizon 30, step_dt 0.0402 -> live 32 laps, 0
collisions, **70.59 s, median 7.09, best 6.98**; the same line at default
params took 1 collision (72.69 s), so the smoother steering (w_rate ~2,
ay_budget at the measured 6.57 limit) is what buys it. 100-lap validation
running.

100 laps of that candidate: 2 collisions (s 11.2 hairpin entry, laps 40 and
86), median 7.126, first ten 70.92 (sim timer 70.32), best 6.89. At 2%/lap
its expected race is no better than the clean 71.63 s config, so it is NOT
promoted. Found meanwhile: mpc3's numpy solves ran OpenBLAS at ~1030% CPU
inside the container in every mpc3 run so far; the entrypoint and
run_stanley.sh now pin OMP/OPENBLAS/MKL threads to 1. Both configs are
being re-run for 100 laps with the pin. With the pin the ay 6.75 candidate
is faster (median 7.049, first ten 70.24) but still 3 collisions/100 at the
s 11.2 hairpin entry: dropped. Next: its smoother-steering parameters on the
clean ay 6.5 line. **Shipping config (mpc3 defaults, mc0.60_ay6.5) with the
BLAS pin: 100 laps, 0 collisions, 72.49 s (sim timer 72.02), median 7.21** --
the second clean century for this config (71.63 s before the pin).

## September 13 (earlier): mpc3 + wide line PROMOTED, 73.5 s / 100 laps clean

**Shipping config on this branch: mpc3 on `porto_mesh_mc0.60_ay6.0.csv`
(mesh map, lidar_offset 0.1933). 100 laps, 0 collisions, first ten 73.46 s,
median 7.35 s, best 7.10 s.** Baseline main 89.6 s; previous promoted
(legacy mpc, centerline) 79.1 s. The submission entrypoint now starts mpc3
with the wide line (PIT_CREW_CONTROLLER=mpc selects the legacy tracker with
the centerline). Pending: image raced as shipped, ICRA with mpc3 on the
mesh lines (running), 32-lap probes of the wide line at ay 6.5.

ICRA-2026 with mpc3 on the mesh lines: centerline (safe) 30 laps, 0
collisions, 149.4 s; the wide line built at the provisional 0.22 m floor
(0.230 m clearance, 12.45 s ideal) took 10 collisions in 18 laps at the
kappa 1.42 corner. **MESH_MIN_CLEARANCE is now 0.28 m** (0.230 collided,
0.292 clean on Porto); the ICRA race line rebuilt under it has 0.301 m
clearance and 12.53 s ideal, untested yet.

mpc3 on the wide line at ay 6.5 (`porto_mesh_mc0.60_ay6.5.csv`): 32 laps,
0 collisions, **72.92 s, median 7.30, best 7.05**. 100-lap validation
pending before it replaces the ay 6.0 line in the entrypoint.
Robustness fixes (teleport restart, stuck gate, callback guards,
supervised entrypoint, preflight cross-checks) committed with 78 tests;
they change nothing on a normal lap but the promoted config must be
re-validated once with them.

100-lap mpc3 on mc0.60 at ay 6.5: 1 collision (lap 35, a late-braking
hairpin entry, during an interval with 5x sensor jitter from an offline
search), median 7.263, first ten 73.61. Re-validate on an idle host before
choosing it over ay 6.0 (0/100).
ICRA rebuilt race line (0.301 m, kappa max 1.36): 9 collisions in 30 laps
with mpc3, all at the kappa 1.36 corner and its entry; the centerline
(kappa max 2.21 but 0.5-0.6 m of room) is 0/30. **MESH_RACE_KAPPA_MAX 1.0:**
a mesh race line tighter than Porto's (0.83) is rejected and the centerline
ships. ICRA now ships `icra2026_mesh_safe` (149.4 s with mpc3, 145.7 s with
the legacy tracker).

**Branch image (mpc3 + wide line + all robustness fixes) raced as the
organisers run it, no mounts: 14 laps, 0 collisions, 74.22 s sample clock,
73.57 s simulator timer, best 7.15 s. PASS.** (`scripts/test_submission_image.sh`)

## September 12, night: mpc3 on the wide line, 74.4 s

**Best measured so far: mpc3 (new tracker) on the wide min-curvature line
`porto_mesh_mc0.60_ay6.0` (mesh map, lidar_offset 0.1933), defaults after
the speed-cut rescale (e_free 0.18, k_e 2.0, barrier_margin 0.24, w_lat 24):
32 laps, 0 collisions, first ten 74.36 s, median 7.35 s, best 7.12 s.**
Promoted config (legacy mpc, centerline) is 79.09 s. 100-lap validation
queued; then promote (submission entrypoint must switch to mpc3 + the wide
line) and re-run ICRA with the mesh line pipeline.
Launch: `CONTROLLER=mpc3 MAP=porto_mesh LIDAR_OFFSET=0.1933 ./scripts/run_exp.sh <name> mc0.60_ay6.0`

**Tyre envelope settled (staircase on the -explore floor, 51 held cells,
4103 rows, `data/logs/sysid_20260911_220017.csv`): sustained lateral
acceleration p99 over a 0.27 s hold is 6.57 m/s2, at every speed; the
asymptote is not identifiable (6.2-8.5 fit equally) because nothing holds
more.** The skidpad's 9.9 was a transient. So ay ~6.5 is the real sustained
limit for this car, matching the driven cliff, and the winners' 6.5 s comes
from a shorter, smoother line tracked to a few centimetres, not from grip.
Calibration in data/calibration/{steer_map,ggv}.csv (tools/lateral_id.py).

## September 12: promoted config = 79.7 s clean over 100 laps

**Branch defaults now ARE the best validated config:** planner `PROFILE` ay
6.0 (ax 6.5, brake 4, jerk 40), mpc `cmd_delay_s` 0.18, `ay_limit` 7.8,
`pose_blend` 1.0, localization on the exact mesh map `data/maps/porto_mesh`
with `lidar_offset` 0.1933 (submission entrypoint + Dockerfile updated).
Measured: 100 laps, 0 collisions, first ten 79.68 s (sim timer 79.10),
median 7.95 s (`ay60_mesh_100`); with `throttle_a_gain` 0.02 (now default):
**100 laps, 0 collisions, 79.09 s (sim timer 78.64), median 7.89 s**. Baseline main was 89.6 s.
`data/racelines/track_v2_safe.csv` is byte-identical to the validated
`track_v2_safe_ay6.0.csv`. `icra2026_safe.csv` was regenerated at ay 6.0 and
is UNDRIVEN — drive it on `2026-icra-compete` before trusting the unseen-track
pipeline at this profile.

Everything measured and rejected today (all on the mesh map unless noted):
legacy ay 6.25 2/100, ay 6.5 3/100 and 7/100, ay 7.0 5/32; mpc2 cut 0.14 at
ay 6.5 0/100 but 80.36 s; mpc2 search-tuned params (cut 0.22, dense horizon)
4/32 at ay 7.0 and 10/12 at ay 6.5 -- the offline optimum does not
transfer. Hairpin-only bias lines, tight caps, gyro/understeer delay models,
Frenet progress, measured-speed dead reckoning, brake_gain 0: all rejected
with numbers in the sections below.

**ICRA-2026 (compete image), exact mesh map `icra2026_mesh` + lidar_offset
0.1933, pipeline line at ay 6.0 with the tight-corner cap (ay 4.5 above
kappa 1.15): 20 laps, 0 collisions, 148.0 s, median 14.77; then 40 more laps, 0 collisions, 148.3 s; with throttle_a_gain 0.02: 30 laps, 0 collisions, 145.7 s.** Old config:
165.3 s; Aug 30 drill: 199.5 s with 5 collisions. Without the cap the ay 6.0
line took 10 collisions in 10 laps at the kappa 1.44-1.71 corner. For an
unseen track: build the mesh map from the compete image with
`tools/map_from_mesh.py --tag <tag> "<track name>" data/maps/<slug>_mesh`
(the track list comes from `--list`), then `make_track_lines.py` as usual.

Throttle over-command (`throttle_a_gain`, throttle per m/s2 of planned
acceleration; the servo only pulls hard when the target is above the current
speed) on the promoted config, 32 laps each: 0.02 -> 78.80 s (median 7.89),
0.04 -> 78.68 s (median 7.88), both 0 collisions over 32 laps -- but 0.04
over 100 laps: 8+ collisions (s 13.6 and 17.6): arriving at corners at plan
speed removes the margin the under-plan car had. NOT promoted. **0.02 over
100 laps: 0 collisions, 79.09 s (sim timer 78.64), median 7.89 -> promoted
as the default.** Lesson: 32 laps cannot resolve a 1-3% per-lap rate.

**Submission image built from this branch, raced as the organisers run it
(`scripts/test_submission_image.sh`, no mounts, official flags): 14 laps,
0 collisions, 79.62 s by sample clock, 78.02 s by the simulator lap timer,
best lap 7.66 s.** Nodes: bridge, localization, mpc only.

Min-curvature lines with the legacy tracker on the mesh map (probes, Sept 12
night): w_veh 0.42 ay 7.0 -> 10 collisions in 3 laps; 0.36 ay 6.5 -> 11 in
4; 0.48 ay 7.0 -> 10 in 8. Collisions are wherever the line hugs a wall
(kappa 0.35 corners as much as the hairpins): tracking p95 0.235 m against
0.206 m clearance. The wide line needs a tracker with ~0.08 m p95 first
(mpc3, in development).

CAUTION: a 150-lap re-run of the promoted config while three build agents
were computing (niced, single-threaded, but with bag exports inside the
devkit container) took **9 collisions** (s 14.8/17.6/12.6, IMU jitter 4x
normal) against 0/100 on the idle host. Treat as contaminated, but re-validate
the promoted config on an idle host before submission; its true rate may be
1-3%/lap rather than 0.

**Wide min-curvature lines on the mesh map (trajectory-anchored reference,
alpha-smooth 11), legacy tracker, ay 6.0, 32 laps each:** w_veh 0.60
(0.292 m mesh clearance, hairpin R 1.20 m) -> median **7.14 s, best 6.97 s,
first ten 71.7 s, 8 collisions** (hairpin exits s 13.3/26.8, s 17.4);
w_veh 0.66 (0.325 m) -> median 7.28, 5 collisions. The line is worth
0.75 s/lap; the legacy tracker still cuts 0.125 m inside on it. mpc3 (new
tracker: horizon optimisation with speed as state, servo throttle) is being
driven on safe_ay6.0, safe_ay7.0 and mc0.60.

**mpc3 live (32 laps each, mesh map):** safe_ay6.0 -> 0 collisions but
86.7 s (0.50 m/s under plan); safe_ay7.0 -> 5 collisions (4 at s 29.2 on
lap 0, right after the standing start); **mc0.60_ay6.0 -> 0 collisions,
78.94 s, median 7.86, best 7.63, e_lat p50/p95 0.072/0.179 m**, still 0.41
m/s under plan. mpc3 is the tracker for the wide line; the remaining gap is
longitudinal (under investigation). 100-lap validation of mpc3 on mc0.60
queued.

mpc3 with the speed-plan/standing-start fixes, 100 laps on mc0.60_ay6.0:
1 collision (s 13.4, lap 75), median 7.853, first ten 78.41 s, still 0.38
m/s under plan -- the fix did not move the live speed. Staircase sysid log
recorded (data/logs/sysid_20260911_212223.csv, 103 rows, likely stopped
early); being fitted.

mpc3 speed deficit found: the lateral-error speed cut's free band (0.05 m)
sat below the tracker's own median error, so it braked ~5% everywhere; the
servo tracks its setpoint to 4 mm/s. Defaults now e_free 0.18, k_e 2.0,
barrier_margin 0.24, w_lat 24 (offline 7.31 s on mc0.60 at the 0.146 m
floor). Live so far: thr_lead 0.15 -> 2 collisions, 7.97 (rejected);
**mc0.60_ay6.5 -> median 7.52, best 7.24, first ten 77.34 s, 1 collision
(s 11.3 hairpin entry, lap 4)**. Explicit P1 A/B on mc0.60_ay6.0 queued.

Qualification deadline is Sept 12 AoE: the qualification image is `main`
(untouched); only the DockerHub push and the Phase 1 form remain.

## September 11, late: 89.6 -> 79.5 s, branch optimizations-ankit

Branch `optimizations-ankit`, PR #1. Every row: Porto, 32 laps, same host,
same session, `scripts/run_exp.sh`, scored by `tools/score_race.py` (sample
clock; the simulator's own timer reads ~0.5 s lower over ten laps).

| run | 10-lap race | median | best | collisions/32 | where |
|---|---:|---:|---:|---:|---|
| baseline main (ay 4.5, ay_limit 4.5) | 89.61 | 8.99 | 8.87 | 0 | |
| cmd_delay 0.09 | 88.32 | 8.83 | 8.75 | 0 | |
| cmd_delay 0.135 | 88.03 | 8.79 | 8.62 | 0 | |
| **cmd_delay 0.18** (now default) | **87.32** | 8.73 | 8.66 | 0 | |
| cmd_delay 0.24 | 88.00 | 8.80 | 8.70 | 0 | |
| d0.18 + ay_limit 5.85 | 87.14 | 8.72 | 8.59 | 0 | |
| d0.18 + use_measured_speed | 91.54 | 9.12 | 8.99 | 0 | rejected |
| d0.18, ay 5.0, ay_limit 6.5 | 84.83 | 8.42 | 8.23 | 1 | s 13.6 hairpin exit |
| d0.18, ay 5.5, ay_limit 7.15 | 82.06+10 | 8.17 | 8.04 | 3 | all s 13.6 |
| **d0.18, ay 6.0, ay_limit 7.8** | **79.49** | **7.94** | **7.87** | **0** | needs long validation |
| d0.18, ay 6.5, ay_limit 8.45 | 77.79+10 | 7.72 | 7.60 | 2 | s 13.6, s 27.7 |
| d0.18, ay 7.0, ay_limit 9.1 | 77.96+30 | 7.58 | 7.38 | 7 | s 12.6/13.6 x5, s 16.2 x2 |

What this says: the dead-time compensation is worth 2.3 s and is in. Above
ay 5 every collision is at a hairpin exit (kappa 0.65-0.78), where the
steering angle is large; the S-bend that used to take every hit only
reappears at ay 7. The car is at or under plan speed at every collision, so
these are lateral, not braking. Lines `data/racelines/track_v2_safe_ay*.csv`
are regenerable (git-ignored) with `optimize_raceline.py --centerline-only
--ay X --ax 6.5 --brake 4.0 --jerk-lat 40`.

### Later on Sept 11: the hairpin collisions are an INSIDE cut, not a wide run

`tools/signed_offset.py` (+ = left of travel) against the centerline through
the right-hand hairpin, s 10-14:

| run | offset s 10-11 | s 12-13 | s 13-14 |
|---|---:|---:|---:|
| baseline (no delay comp, ay 4.5) | -0.078 | -0.107 | -0.016 |
| cmd_delay 0.18, ay 4.5 | -0.151 | -0.148 | -0.209 |
| cmd_delay 0.18, ay 6.0 | -0.168 | -0.150 | -0.228 |
| + reference shifted 0.10 m INSIDE | -0.260 | -0.251 | -0.301 (collision every lap) |

Negative is right = inside. The car cuts the hairpin and every s 13.6
collision is with the inside wall at the exit. The dead-time compensation
doubles the cut. Everything that "failed" that afternoon is the same
mechanism: the gyro and understeer delay models predict less rotation so the
controller turns harder (10 collisions in <= 11 laps each); ay_limit 9.5
allows more lock (5 collisions); the inside bias moved the line into the
wall. An OUTSIDE bias (`--inside-bias -0.12`) and a shorter delay are the
two fixes under test. Also measured: brake 6.0 with the honest drag/servo
profile is worse (6 collisions), and the servo only accelerates hard when
the target is well above the current speed, so the straight-line deficit
needs throttle over-command (`throttle_a_gain`), not a different profile.

Simulator facts from the asset audit (see the commit "Exact maps from the
simulator mesh"): throttle is a speed servo (target 24.6 x throttle, ~6
m/s2 authority both ways, zero throttle brakes at ~5.8 m/s2); rigidbody
drag 0.273/s; physics at 1000 Hz; telemetry every 90 ms regardless of
graphics or load; LiDAR is a CPU raycaster (the "GPU raytraced" note in
CLAUDE.md is wrong, though Xvfb still works); sideways tyre peak mu 1.0 at
slip 0.01, asymptote 0.5 at 0.1; all six circuits' meshes ship in the
practice image and `tools/map_from_mesh.py` turns them into exact maps.

### Later still: the hairpin cut is a LOCALIZATION bias

`race_logger.py` now records `/pit_crew/pose` next to IPS truth. On the
ay 6.5 run, projecting both onto the centerline (+ = left):

| s | truth lateral | estimate lateral | estimate - truth |
|---|---:|---:|---:|
| 0-10 (straight, easy corners) | -0.090 | -0.097 | -0.007 |
| 10-12 hairpin entry | -0.140 | -0.257 | **-0.117** |
| 12-14 hairpin exit | -0.165 | -0.331 | **-0.166** |
| 25-29 second hairpin | -0.129 | -0.257 | **-0.128** |

The filter is unbiased on the straights and 0.12-0.17 m to the inside in
every tight corner. That is what ignoring the LiDAR's 0.1933 m forward
offset does in map AND filter: it cancels while the heading is constant and
shifts the estimate toward the centre of a tight turn. It also explains why
every controller-side fix failed (Frenet progress: 6 collisions, same cut;
brake_gain 0: 3 collisions, median 7.81; ay 6.5 Frenet: 3 collisions) and
why the offline tracker sim never reproduces the cut. Under test:
localization on the exact mesh map with `lidar_offset` 0.1933
(`MAP=porto_mesh LIDAR_OFFSET=0.1933`), then mpc2.

**Mesh map + lidar_offset 0.1933 (`MAP=porto_mesh LIDAR_OFFSET=0.1933`),
ay 6.0: 32 laps, 0 collisions, 80.12 s (sim timer 79.62), median 8.02.**
Estimate-truth lateral difference is now within 0.02 m everywhere,
hairpins included. The truth still cuts the hairpins by 0.15 m, so that part
is the controller; the mesh map is the localization base from here on.
`data/racelines/porto_mesh_*.csv` are symlinks to the track_v2 lines and
`data/maps/porto_mesh_traj.csv` to the recon trajectory (for the init pose).

Mesh-map ladder (`MAP=porto_mesh LIDAR_OFFSET=0.1933`, cmd_delay 0.18,
ay_limit = 1.3 x ay, 32 laps each):

| ay | race | median | best | collisions |
|---:|---:|---:|---:|---:|
| 6.0 | 80.12 | 8.02 | 7.92 | 0 |
| **6.5** | **78.18** | **7.83** | **7.73** | **0** |
| 7.0 | 76.53 (+50 pen.) | 7.65 | 7.52 | 5 (s 27.7 x3, 12.6, 29.2) |

The controller's own cte at its predicted point is -0.04 in the hairpins
while truth is -0.15: the delay prediction under-rotates at hairpin exits.
Gyro sign/gain verified correct (corr 0.98, slope 0.96 vs IPS heading rate).

100-lap validation, mesh map, ay 6.5: **3 collisions / 100 laps** (s 13.6 x2,
s 27.7 x1), median 7.834, first ten 78.33 s. A 3% per-lap rate is ~3 s of
expected penalty per race; not shippable without the hairpin fixed.
mpc2 on the mesh map: ay 6.0 84.84 s / ay 6.5 83.12 s, both clean, tracking
p50 0.065 m, but 1.17 m longer path per lap (no corner cut); at ay 7.0/7.5
it collided because its steering cap sat below the line's own feedforward
(fixed: cap derived from the line with 15% headroom, under test).

Offline identification against the bag (`tools/delay_model_id.py`): the
shipping prediction (gyro to now, then bicycle through the command history)
is unbiased at the hairpins (median 3 mm at H 0.18); the car rotates 4% LESS
than the bicycle, not more; understeer scaling is the worst-biased model.
So the 0.11 m between the controller's cte and truth is in the pose it
starts from: on the mesh map the filter output matches truth, leaving the
controller's `pose_blend` 0.55 first-order lag (under test: 1.0 and 0.8).

mpc2 with `cut_offset_m` 0.14 on the ay 6.5 line, mesh map: **32 laps, 0
collisions, 80.55 s, median 8.06** (legacy 78.18 but 3 collisions/100).
mpc2 at ay 7.5 with the line-derived cap: 5 collisions, no headroom there.
Hairpin-only outside bias on the mesh map: 82.3 s and 1 collision; dropped.

More mesh-map runs at ay 6.5 (32 laps each): pose_blend 1.0 -> 77.59 s,
median 7.753, 1 collision (s 17.6), hairpin cut unchanged at -0.165;
tight-corner cap 5.5 + pose_blend 1.0 -> 78.22 s, 1 collision (s 27.7);
mpc2 cut 0.18 -> 80.87 s, 1 collision. So the legacy tracker sits at ~1-3
hairpin collisions per 100 laps at ay 6.5 whatever the cap, bias, blend or
prediction model, and mpc2 cut 0.14 (80.55 s clean) is the safe candidate.
100-lap validations of mpc2 cut 0.14 (ay 6.5) and legacy ay 6.0 running.

**100-lap validation: mpc2, cut_offset_m 0.14, ay 6.5 line, mesh map with
lidar_offset 0.1933: 100 laps, 0 collisions, first ten 80.36 s, median
8.068 s.** This is the first configuration under 81 s with a measured zero
collision rate over 100 laps. Launch:
`CONTROLLER=mpc2 MAP=porto_mesh LIDAR_OFFSET=0.1933 ./scripts/run_exp.sh <name> safe_ay6.5 -p cut_offset_m:=0.14`

**100-lap validation: legacy mpc, ay 6.0 line, mesh map, cmd_delay 0.18,
ay_limit 7.8, pose_blend 1.0: 100 laps, 0 collisions, first ten 79.68 s
(sim timer 79.10), median 7.951.** Best validated config so far. Launch:
`MAP=porto_mesh LIDAR_OFFSET=0.1933 ./scripts/run_exp.sh <name> safe_ay6.0 -p cmd_delay_s:=0.18 -p ay_limit:=7.8 -p pose_blend:=1.0`

INVALID runs (host load average 49/32 cores from an offline search running
concurrently): ay625_pb1 (1 collision, 78.26 s) and ay65_pb1_100 (10
collisions in 32 laps, 5 of them at s 17.6 which never collided before).
Re-run both once the host is quiet. Rule: never drive a scored run while
anything else heavy runs on the host; check `uptime` first.

Re-runs on a quiet host: ay 6.25 + pose_blend 1.0 -> 32 laps, 0
collisions, 79.33 s, median 7.92. ay 6.5 + pose_blend 1.0, 100 laps -> 7
collisions (s 17.6 x4, 16.2 x2, 27.7), median 7.79; the first third of that
run overlapped the tail of the offline search (sim at 9.9 Hz), but 4 of the
7 came after it stopped, so ay 6.5 is not clean for the legacy tracker.
Offline search (tools/search_config.py; sim matches mpc2 live to 0.1 s, not
the legacy): mpc2 on the ay 7.0-7.5 lines with cut_offset_m ~0.27, step_dt
0.035-0.05, horizon 23-29, w_rate ~1.2, apply_ahead_s 0.12 predicts
7.58-7.63 s/lap at the 0.10 m clearance floor. Live test of a moderated
version (cut 0.22, ay 7.0) queued. Candidate lines in
data/racelines/track_v2_cand_*.csv (mc0.42_ay7.0: 7.12 s ideal, 0.206 m
mesh clearance) are built but not driven.

Ready to test next, in order:
1. `--ay-tight 5.0` lines (`track_v2_safe_ay6.0t/6.5t/7.0t.csv` exist):
   keep the hairpins at ay 5 and let the rest run at 6-7. Ideal laps 7.9 /
   7.7 / 7.5 s.
2. Long validation (100 laps) of whichever is clean, then ICRA-2026, then
   promote: `PROFILE` in make_track_lines.py and `ay_limit` default = 1.3x.
3. Corrected-LiDAR map `data/maps/track_v2_c` is built (from the ay 5.0
   bag, git-ignored) with `*_c_*` line symlinks; run with
   `MAP=track_v2_c LIDAR_OFFSET=0.1933 ./scripts/run_exp.sh ...`. Its
   clearance on the safe line is 0.300 m vs 0.325 on the old map.
4. Review knobs still untested (all default off): blend_gate_m,
   rollout_profile_speed, kappa_span_m 0.2, windowed_index, pose_qos_depth
   1, keep_prior_weights, midpoint_heading, publish_on_reloc false.
5. Straights: the car sits 0.9-1.2 m/s under plan wherever ax > 3 (a
   placed team measured the drivetrain at ~3.5 m/s2 at 6-8 m/s). Worth a
   speed-dependent ax ceiling in the profile and a re-test of
   throttle_a_gain 0.02 now that corner entry is predicted.

Organisers' procedure (verified from the official repo): Ultra graphics,
GUI, i9-14900K / RTX 4090, `docker run ... --entrypoint /bin/bash -it`
then the race starts when they click Connection; they sum the sim's own
lap timer; +10 s per collision; 12 laps (warm-up, 10, cool-down). Validate
the final config once with `VISIBLE=1` at Ultra before submitting.

## September 11, evening: where the 2.4 s is

Everything below was measured today on a new host (RTX 5090; the docker images
had to be pulled and `.venv` rebuilt with uv — `quadprog` needs
`--no-binary quadprog` or `trajectory_planning_helpers` fails to import).
Baseline reproduced first, so the numbers are comparable to September 10:

| configuration | laps | collisions | median | best | first ten scored |
|---|---:|---:|---:|---:|---:|
| shipping main, `run_stanley.sh 1.0 track_v2 safe`, Xvfb, default graphics | 33 | 0 | 8.993 s | 8.874 s | **89.606 s** |

### The target, verified from the official results pages

Porto qualification best laps: **6.49 s** VAUL and 6.53 s Mamba (65.41 s race)
at CDC-TF 2025; 6.77 s AGV (67.86 s) at ICRA 2025. Sources:
[CDC-TF 2025](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-cdc-tf-2025/),
[ICRA 2025](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-icra-2025/).
We are at 8.87 best / 8.99 median: **1.37x**.

### Offline: the gap is the grip envelope, spread over every corner

Ideal lap on our own safe centerline (`optimize_raceline.py --centerline-only`,
ax 6.5 / brake 4 / jerk 40 unless noted):

| ay | ideal lap |
|---:|---:|
| 4.5 (ships) | 8.611 s |
| 6.0 | 7.794 s |
| 7.5 | 7.215 s |
| 9.0 | 6.847 s |
| 9.0, ax 7.1, brake 7 | 6.730 s |
| 9.0, ax 7.1, brake 7, no jerk limit | 6.552 s |

So the winners' 6.5 s is exactly our own line at ay 9. The 1.88 s between the
ay 4.5 and ay 9 profiles is spread: 0.15-0.24 s in each of the six corners,
0.03-0.06 s on each straight. No single corner fix reaches it; only the
lateral envelope does. Each +1 m/s2 of usable ay is worth roughly 0.4-0.5 s of
lap at the current level.

### The "4.64 m/s2 sustained limit" is our own cap, not the tyre

`grip_curve.py` on today's baseline log: the achieved/asked curvature ratio is
0.93-0.97 in EVERY steering bin from 0.05 to 0.40 rad, and achieved ay tops out
at p99 4.68 m/s2. The tyres deliver what they are asked; they are simply never
asked for more, because the profile (ay 4.5) and `ay_limit` (4.5) both stop at
4.5. A race log cannot measure the tyre limit while the controller caps demand
below it. The only uncapped measurement we have is the `-explore` skidpad:
~0.9 of asked curvature up to **9.87 m/s2**, then an abrupt collapse. The
winners' onboard telemetry (8.6-9.5) agrees. Stop quoting 4.64 as a tyre
property; it is where our collisions start, which is a control question.

### Measured today: 0.135 s of dead time the controller does not know about

`src/tools/latency_probe.py` (new) steps steering at standstill and times the
`/steering` feedback: sensors at **11.36 Hz** on this host; command to first
feedback movement **median 0.135 s, range 0.090-0.180 s**; then the 3.2 rad/s
slew. Cause, from reading the devkit bridge: it is request/response — the sim
sends a frame, the bridge publishes sensors and returns *whatever command was
last published*. Our 50 Hz loop is therefore sampled at ~11 Hz, and the applied
command is up to one frame old before the sim even sees it.

What that costs at the corner that produces every collision (s 14.6-15.7, kappa
-0.52 -> +0.38 in 1.1 m at 2.7-3.4 m/s): 0.135 s of dead time is 0.36-0.47 m of
travel, **40% of the ramp**; the full observe-to-act chain (frame age +
localization + dead time, roughly 0.2 s — estimate) is ~0.7 m, most of it. The
MPC applies `delta0` at the current pose as though it took effect now, and its
pose prediction covers only scan-to-now. Cross-track p95 of 0.31 m through
that entry (Aug 29) is what an unmodelled 0.135 s looks like at 3.4 m/s.

### Pointers, ranked by evidence (owner decides which to pursue)

1. **Dead-time compensation in the MPC.** Keep the last ~0.2 s of published
   steering commands, propagate the predicted state through them (bicycle +
   3.2 rad/s slew) to the moment the new command will actually bite, then
   optimise from there. Standard latency-compensated MPC. Evidence: today's
   0.135 s measurement, the bridge design, and the fact that all collisions
   sit at the one kappa ramp shorter than the dead-time travel. Cheap to test:
   drive the shipping profile, then ay 5.0 / 5.5 / 6.0 with `ay_limit` raised
   in step, 36 laps each. Success = the ay 5.5 run that took 8 collisions on
   Aug 29 goes clean.
2. **Corrected-LiDAR map (0.04 m lateral p95) is validated and unshipped.**
   Its gate was "beat 9.81 s and pass ICRA" (Aug 23), both since overtaken.
   Re-run it with the current ay 4.5 / brake 0.05 stack, then as the base for
   pointer 1. `MIN_CLEARANCE` 0.30 m was set with 0.17 m of localization
   error; with 0.04 m the min-curvature line's 0.283 m may be enough, which is
   0.31 s of ideal lap. Test order: corrected map + safe line first.
3. **Only then raise ay**, one step at a time, profile and `ay_limit` together.
   Skidpad says the cliff is at ~9.9; the winners sit at 8.6-9.5. Expect the
   clean ceiling to move with tracking accuracy, not with the tyre.
4. **Longitudinal loop with the dead time modelled.** `throttle_kp` bang-bang at
   10 Hz is the textbook signature of a proportional loop with 0.135+ s of
   delay at 0.09 s sampling. A Smith-predictor style loop (predict speed
   through the known delay with the measured 0.040 throttle/(m/s) plant, then
   correct) is the honest version of the closed loop that failed. Lower
   priority: STATUS shows the speed deficit is currently load-bearing margin at
   corner entry, so do it after 1.
5. **Jerk limiter at 40 costs 0.18 s at ay 9** (6.73 vs 6.55) and nothing at
   4.5. Revisit only once the envelope is above ~7.
6. **Not worth time** (re-confirmed today): line geometry on Porto, steering
   resolution, the grip watchdog, the arbiter, and raising ay without 1 or 2.

Public winner code: VAUL's repos hold only a wall-follower entrypoint and a
2023 TUM optimizer fork with an f1tenth-gym ggv (ay 12 flat) — not tuned for
AutoDRIVE and not evidence of what they raced.

Docs fixed today: README shipping config (was ay 5.0 / brake_gain 0.3 / 91.4 s),
AGENTS.md test-suite line, CLAUDE.md Aug 28 shipping paragraph marked
historical. `upstream/` now also holds the VAUL clones (git-ignored).

## September 10: qualification audit (was START HERE)

Qualification closes September 12 AoE (September 13, 17:30 IST). Use the
`2026-iros-practice` base, not `compete`. Packaging/run instructions are in
`submission/README.md`. DockerHub publication and the Phase 1 form are separate
from GitHub; the DockerHub repository name is still needed from the team.

Fresh measurements on September 10:

| configuration | first ten scored laps | scored collisions | fastest sampled lap |
|---|---:|---:|---:|
| existing main, development runner, default graphics | 90.965 s | 0 | 8.965 s |
| audited standalone image, Ultra graphics, no mounts | 89.547 s | 0 | 8.907 s |

The standalone run reached 20 counted laps with zero collisions before explicit
failure injection. LiDAR: 20/20 distinct scans at 10.36 Hz. These lap times use
host timestamps at simulator lap-counter transitions (about one sensor-update
timing uncertainty), not official high-resolution lap timing. The environments
differed, so their difference is NOT an isolated performance improvement.

**Scoring correction:** older sections claiming `10 * median` plus proportional
collisions was the competition scoring are wrong. `tools/score_race.py` now uses
the actual ten-lap window after warm-up and the collision-counter difference
inside that window. It rejects late recordings, missing transitions and resets.
It cannot declare a backwards run successful based on geometric gate crossings.
Historical race-time estimates remain historical; re-score their logs before
comparing them with current results.

Audit changes: supervised child processes and process-group shutdown; preserved
bridge while publishing final zeros; preflight asset checks; no stale/NaN pose
refresh; invalid LiDAR and stale IMU rejection; reactive stale-scan watchdog;
complete encoder reset; line-buffered evaluation logger; consistent pinned
development SciPy; reduced baked map assets. Runtime subscriptions were inspected
on the live image: no restricted inputs in MPC or localization. Additional shell
starts no duplicate nodes. Missing raceline exits before starting the bridge.
Suspended localization caused zero throttle; killing MPC caused zero throttle
and container exit 1. The original upstream package is not edited.

Validation: six pure-Python tests, three real ROS-node watchdog tests, and
`scripts/check_pipeline.sh` pass. Both shipping track artifacts are unchanged.
The simulator initially crashed without the documented privileged GPU flags;
the normal official flags worked. The dev runner previously installed NumPy 2
via unpinned SciPy; the standalone image retains NumPy 1.22.2 / SciPy 1.10.1.

The unfinished local edits found on arrival were preserved in branch
`experiments/pre-submission-20260910` at `64322d1`. They are not part of the
qualification image. Substantial performance changes stay on separate branches.

Matched Ultra-graphics performance experiment completed: correcting the MPC
pose-blend timestamp produced 89.835 s / zero collisions / 8.900 s best, against
main's 89.547 s / zero / 8.907 s. The best-lap difference is below the logger's
roughly 0.1 s sample period and total time was 0.288 s worse, so the change was
rejected and removed. No performance experiment is merged into qualification.

Remaining: DockerHub push/access check and Phase 1 submission form. The final
track pipeline still needs its own race validation after the September 16 release.

### September 11 race-time isolation tests

Main remained unchanged while three previously proposed performance changes were
tested or screened independently:

- The gated turn-in profile was rejected offline. It changed 109 of 299 speed
  samples and increased ideal lap time from 8.637 s to 8.861 s while the current
  profile is already collision-free, so it had no credible race-time benefit.
- `throttle_a_gain=0.01` (one quarter of the previously failed acceleration
  feedforward) produced 4 collisions by lap 8 and was stopped. The car's normal
  speed deficit is load-bearing corner-entry margin, not free straight-line time.
- `rate_limited_rollout=true`, isolated from the old five-change MPC bundle,
  produced 1 collision by lap 6 and was stopped. The more physically faithful
  future actuator model does not improve the competition objective here.

One startup attempt accidentally left a delayed helper alive and briefly created
duplicate controller processes. That run was detected from the process table,
stopped, and excluded from every result above. Both valid experiments used one
localization, one MPC and one logger with rendered Xvfb LiDAR. All rejected files
and the temporary experiment branch were removed; qualification `main` remains
at the verified 89.547 s / zero-collision baseline.

## Target: what winning actually looks like

Published results, **same Porto practice track we develop on**:

| Event | Qualification (10 laps) | Best lap | Finals best lap |
|---|---|---|---|
| CDC-TF 2025 | Mamba 65.41 s | **6.53 s** | VAUL 7.74 s |
| ICRA 2025 | AGV 67.86 s | 6.77 s | VAUL 11.28 s |

Qualification uses the **Porto practice track — the same one we develop on**,
so those best-lap figures are directly comparable to ours. The finals figures
are on each event's own unseen track and are not comparable. Podium teams post
**zero collisions**, so cleanliness is table stakes, not an edge.

Our best clean lap is **9.81 s** — about **1.50x off the winning pace** (it was
32.43 s when this section was first written; see "Speed: 32.43 s -> 11.21 s").
The old conclusion that the remaining gap was simply the plan is superseded by
the Aug 23 winner-video and localization isolation work below.

The benchmark has since been verified asset-by-asset: the 2025 and 2026 Porto
builds have the same track mesh, vehicle and physics. Onboard winner telemetry
also directly shows roughly 8.6-9.5 m/s2 of lateral acceleration and a path
close to the optimized TUM geometry. The gap is real; the previous inference
that this grip was unusable came from our controller's behaviour, not from the
winning car's demonstrated envelope.

Sources: [ICRA 2025 results](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-icra-2025/),
[CDC-TF 2025 results](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-cdc-tf-2025/)

## NEXT SESSION — START HERE (Aug 30) — UNSEEN TRACKS NOW WORK

The pipeline failure recorded below is **fixed and verified by driving**. Scored
the competition's own way (1 warm-up + 10 scored laps, race = 10 x median +
10 s per collision, >10 collisions = DQ) with `tools/score_race.py`:

| round | track | tuned? | laps | collisions | median | race | result |
|---|---|---|---|---|---|---|---|
| 1 qualification | Porto | yes | 18 | **0** | 9.179 s | **91.8 s** | PASS |
| 2 finals | SRL 2025 CDC-TF | **never** | 20 | **0** | 12.451 s | **124.5 s** | PASS |
| 2 finals (harder) | SRL 2025 ICRA | **never** | 17 | 5 | 17.012 s | 199.5 s | PASS |

Both 2025 circuits previously produced **no drivable line at all**. Three
defects, all found using the two committed map fixtures:

1. `trace_via_waypoints` chains Dijkstra shortest paths, and the shortest route
   between two points on a loop may go either way round. It **retreads
   itself**: both 2025 circuits emitted ~1.7x the cycle length, and the periodic
   spline folds through the repeats. Detected by overlap ratio and handed to a
   new `walk_cycle`, which walks the pruned skeleton directly (all four
   skeletons are single components with zero endpoints, so the cycle was always
   there). **Trigger on overlap, not on net turn** — ICRA-2026's trace also
   folds, at 1.01x, and `excise_spurs` plus the spline have cleaned that up for
   weeks; walking its cycle instead is strictly worse, because the corridor
   merges an infield passage and the walk cuts across it (min corridor width
   1.32 m -> 0.05 m).
2. A line merely **tighter than the steering lock** was vetoed as hard as one
   that doubles back. What tightness costs is a bounded apex deviation of
   (r_lock - r_line) — 0.018 m and 0.074 m on these tracks — which now gets
   charged against corner clearance. Only under half the lock is vetoed.
3. `open_corners.py` is wired in as a fallback candidate, promoted only when the
   plain centerline is not drivable. On these narrow tracks it loses (it halves
   clearance and doubles the ideal lap), but it costs nothing to offer.

Porto and ICRA-2026 are **byte-identical**; `check_pipeline.sh` passes.

### Still weak: the reactive fallback

Follow-the-Gap took 24 collisions in 194 s on 2025 ICRA — it would be
disqualified. A direction lock was added (gyro-integrated heading, held for
`direction_lock_s` after an escape, so the car cannot resume back down the
track) but **it did not engage in that run — zero escapes — so it is
unproven**. The mechanism addresses the documented failure; it still needs a run
that actually triggers it. FTG also needs a narrower bubble on tight tracks
(`FTG_ARGS` in the drill).

## CHECKPOINTS — race time (1-3 DONE Aug 30, outcomes below)

Shipping is now **ay 4.5 / ax 6.5 / brake 4.0 / jerk 40**, re-validated by
driving both tracks: Porto **91.1 s** (36 laps, 0 collisions) and ICRA-2026
**165.6 s** (24 laps, 0 collisions).

- [x] **1. Jerk limiter fixed.** Across a sign change
      |ay_big - ay_small| = |ay_big| + |ay_small|, so |ay_big| <= lim -
      |ay_small|. The algebra was right, the remedy was not: when the SMALLER
      side alone exceeds the budget the constraint is infeasible with it held
      fixed, and the old code drove the other side to zero, clamping it to the
      0.5 m/s floor, instead of reducing both. Splitting the budget is worth
      0.3 s of ideal lap at ay 9.0. **The cap mattered more:** at j_max >= 40
      the limiter stops binding (ay 5.0: 8.5 -> 8.2 s; ay 9.0: 7.8 -> 6.8 s).
      j_max 20 was set pre-brakes and never revisited.
- [x] **2. The ay cap cannot be raised — it had to be LOWERED, to 4.5.** At
      j_max 20 the buggy branch was also the corner-entry brake into the
      s ~ 16 corner; freeing the cap removes it, so the ay cap must do that job
      honestly. Driven on Porto, one session:

      ay 5.0 jerk 20 (old)  18 laps  0 coll  median 9.179   91.8 s
      ay 5.0 jerk 40        33 laps  1 coll  median 8.944   92.5 s
      ay 5.5 jerk 40        34 laps  8 coll  median 8.899  111.8 s
      ay 4.8 jerk 40        37 laps  1 coll  median 8.979   92.5 s
      ay 4.5 jerk 40        36 laps  0 coll  median 9.112   91.1 s  <-- ships

      The collision threshold sits between 4.5 and 4.8, which is exactly the
      measured 4.64 m/s2 sustained cornering limit. ay 5.5 buys 0.045 s of
      median and pays 8 collisions for it.
- [x] **3. Control resolution is NOT the limit — hypothesis disproved.** The
      MPC's 9 candidates land exactly one rate-limit step apart, so only three
      applied values are reachable. That was documented as a defect and never
      tested. With 17 candidates: 36 laps, 0 collisions, median 9.165 against
      9.112, means identical at 9.13/9.14. Do not spend more here.

### What this means for the 6.53 s benchmark

The geometry supports it (6.6 s ideal on the safe line at ay 9.0), and the
planner no longer distorts the profile. **The binding constraint is now purely
the tyre operating point.** We sustain 4.64 m/s2; the 9.81 peak sits at 0.01
slip, on the rising side of the curve. Holding it needs slip regulation, and
every sensor shares one WebSocket at ~10 Hz. Three independent things are now
ruled out as the cause: the planner's optimizer, the tracker's fidelity
(0.95-0.96 of asked curvature), and its steering resolution.

### Encoders (Aug 30) — the sensor was broken; race time is unchanged

The `SpeedEstimator` that the MPC's braking and the particle filter's motion
model both read was measured against IPS ground truth for the first time:

|  | bias | RMS | p95 abs | scale | corr |
|---|---|---|---|---|---|
| before | -0.608 | 1.949 | 5.570 | 0.785 | 0.275 |
| after | -0.171 | **0.489** | **1.110** | 0.937 | **0.898** |

The cumulative wheel angle steps backwards on **6.7%** of samples, in pairs one
apart, by up to 18.5 rad; the guard was 1.1x the *claimed* `TOP_SPEED` of
22.88 m/s and let all of them through as speeds down to -10.8 m/s. `_last` was
also advanced before the check, so each glitch poisoned the following sample
too, and both wheels shared one filter.

**Fixing it destabilised the car**, because `brake_gain` 0.3 was tuned against
that broken signal — braking barely fired. Negative throttle here is reverse
drive: at 0.3 the throttle went 52% negative and the car averaged -2.95 m/s,
176 collisions. `brake_gain` is now **0.05**: Porto 36 laps 0 collisions
**91.2 s**, ICRA-2026 24 laps 0 collisions **164.9 s**.

**Race time is neutral.** What this buys is a correct sensor, a matching brake
gain, and one latent latch removed — reject-only cannot see a sustained
reverse, so the brake keeps firing, which is what causes the reverse.

### Remaining, in priority order

- [ ] **4. Slip-regulated lateral control** — the only remaining route to the
      envelope. Needs a slip estimate (IMU yaw rate + encoder speed + steer)
      and a controller that regulates it rather than commanding curvature open
      loop. High effort, uncertain, and the 10 Hz sensing may make it
      impossible. Nothing else moves lap time materially.
- [ ] **5. Make the plan honest through Porto's s ~ 16 corner.** It still
      produces every collision above ay 4.5. Cross-track p95 is 0.18 m lap-wide
      and 0.31 m (0.59 m worst) through that entry. Re-cut the line locally to
      cut d(kappa)/ds, holding clearance >= 0.325 m.
- [ ] **6. Cap planned curvature at `vehicle.achieved_curvature()` in the
      PLANNER.** Still not done, still the documented right home for it.
- [ ] **7. Do NOT chase the plan-tracking deficit.** Measured and closed: the
      car sits 0.35 m/s under its own plan and that is load-bearing.
      `throttle_kp` 0.08 goes bang-bang at 10 Hz (75 collisions);
      `throttle_a_gain` 0.041 is stable but cannot hold the s ~ 16 corner
      (5 collisions, 102.5 s). Both knobs exist, both default 0.
- [ ] **8. Prove the Follow-the-Gap direction lock.** It has never engaged in a
      run. FTG is the RECON DRIVER, not just a fallback: it sets the racing
      direction of every line we build, and a backwards recon means zero
      counted laps however cleanly we drive.
- [ ] **9. Smaller:** `--ax` 6.5 -> 7.1; corrected-LiDAR map with brakes; grip
      watchdog with brakes (low — lost everywhere).

## SUPERSEDED — was START HERE (Aug 29, evening); still current on lap times

**Shipping is unchanged and is still the best thing measured: safe centerline
+ MPC, ay 5.0 / ax 6.5 / brake 4.0 / jerk 20, `brake_gain` 0.3.** Everything
tried this session lost. Three new knobs exist and all three default to OFF, so
`check_pipeline.sh` reports the shipping lines byte-identical.

### The one finding that matters

**The car is 0.35 m/s under its own plan on average, and that deficit is
load-bearing.** Measured on the brake-7.0 race trace (7492 samples): it is below
plan on 62% of samples, by **0.96 m/s** wherever the profile asks ax > 3 and by
**1.2 m/s** wherever it asks for more than 5 m/s. Least squares gives
`throttle = 0.0144*v + 0.0414*a + 0.055`, against a controller that commands
`0.040*v + 0.005` and knows nothing about `a`.

That looks like free time. It is not. **Every attempt to make the car track its
own plan more faithfully made it slower**, because the plan through one corner
is already past what the car can hold and the under-delivery is what keeps it
clean:

| change | laps | collisions | median | race |
|---|---|---|---|---|
| **shipping** (brake 4.0, jerk 20) | 59 | **0** | 9.190 | **91.9 s** |
| brake 7.0, jerk 20 | 78 | 1 | 9.140 | 92.7 s |
| brake 7.0, jerk off | 69 | 3 | 8.920 | 93.5 s |
| + accel feedforward `a_gain` 0.041, gated | 65 | 5 | 9.480 | 102.5 s |
| + accel feedforward, ungated | ~20 | 14 | — | aborted |
| + closed speed loop `throttle_kp` 0.08 | ~20 | 75 | — | aborted |

All measured in one session on Xvfb, so they are comparable to each other.

### What is settled now

- **Do not ship `--brake` 7.0.** Last session's 39 clean laps were not enough
  evidence. Over 78 laps it takes a collision, and its median gain is 0.05 s —
  noise. Race estimate is *worse* than shipping.
- **Do not remove the lateral-jerk limiter.** Removing it is worth 0.22 s of
  median but costs 3 collisions in 69 laps. The bug in it is real and now
  understood: at a curvature sign reversal `allowed = lim - |ay_small|`, and on
  the safe line that fires at **s = 15.3**, slamming the plan to 2.10 m/s. That
  point is the entry to the s ~ 16 corner. **The bug is accidentally acting as
  the corner-entry brake that keeps the lap clean.** Fixing it without replacing
  what it does will reintroduce the collisions.
- **Closed-loop speed control is confirmed unusable at 10 Hz.** `throttle_kp`
  0.08 goes bang-bang: throttle saturated at ±1.0 on 62% of samples, full ±2.0
  swings between steps, the car driving backwards into walls. The
  `ThrottleController` docstring warned about exactly this and was right.
- **Acceleration feedforward is open loop and still loses.** Ungated it adds
  throttle while the braking horizon is already lifting; gated on the horizon it
  is well behaved but the car simply arrives at the s ~ 16 corner at plan speed
  and cannot hold it.

### Where the remaining time actually is

**One corner produces every collision in every configuration** — (-1.84, -3.42),
s ~ 16.2, where kappa ramps 0.04 -> 0.38 in half a metre. Lap-wide cross-track
p95 is 0.18 m; through that entry it is **0.31 m with a 0.59 m worst case**, and
the car is 1.05 m/s under plan at the apex while still running wide.

So the next gain is **geometric, not longitudinal**: make the plan through
s = 14-20 something the car can hold, rather than making the car chase a plan it
cannot. Concretely, in order:

1. Re-cut the safe line locally through that corner to cut the curvature *rate*
   (`tools/open_corners.py` maximises the smallest radius; what is needed here
   is the smallest d(kappa)/ds), holding clearance at or above 0.325 m.
2. Cap planned curvature at `vehicle.achieved_curvature()` **in the planner** —
   still not done, still the documented right home for it. It is what would make
   the plan honest at turn-in without the jerk limiter's accident.
3. `--turn-in` (added this session, default 0) is a blunt first cut at the same
   idea: it caps speed on the forward-window maximum curvature. Offline it costs
   0.17-0.43 s of ideal lap and it did not target the entry sharply enough to be
   worth driving on its own. Keep it as a comparison, not a candidate.

### Still open from before

- Corrected-LiDAR map re-measured with brakes.
- Grip watchdog re-measured with brakes (low priority — lost everywhere).
- `--ax` 6.5 -> 7.1.
- Teleport-excision centerline fix (Priyanshu Fault 4) — start a session with
  it, gate on `check_pipeline.sh`, not near a deadline.
- Two-phase submission: Phase 1 (Sep 12) robust, Phase 2 (Sep 19) fast.

## SUPERSEDED — was START HERE (Aug 28, late); see the Aug 29 evening section above

**The controller could not brake, and that was the "grip ceiling".**

`ThrottleController` gated its braking branch behind `kp > 0`, and nothing sets
`kp`. Every throttle command was `0.040*v_target + 0.005`, strictly positive.
The car had **no braking authority at all** — it shed speed only by coasting —
while the offline velocity profile was built assuming 4.0 m/s2 of deceleration.
It could plan a corner-entry speed it had no way to reach. Found by Deepak G's
formulation (`docs/Deepak G/25-08-26/`, defect B5); response in
`docs/reviews/deepak-g.md`.

Every one of the four "independent demonstrations" of the grip ceiling recorded
below was measured in that state. ay 4.0 (23 collisions) and ay 5.0 (31) are
what a car with no brakes looks like. **With brakes, both run clean.**

### Shipping config changed

**safe centerline + MPC, ay 5.0 / ax 6.5 / brake 4.0 / jerk 20, `brake_gain`
0.3.** Porto race 97.2 -> **91.4 s**; ICRA 173.6 -> **165.3 s**. The regenerated
`safe.csv` on both tracks is byte-identical to the lines actually driven, so
what ships is what was validated.

| Porto, ~40 laps each | median | best | collisions | race |
|---|---|---|---|---|
| ay 3.5, no braking (old shipping) | 9.72 s | 9.59 s | 0 / 37 | 97.2 s |
| ay 3.5, brake 0.3 | 9.89 s | 9.70 s | 0 / 43 | 98.9 s |
| ay 4.0, brake 0.3 | 9.54 s | 9.40 s | 0 / 38 | 95.4 s |
| ay 4.5, brake 0.3 | 9.31 s | 9.13 s | 0 / 39 | 93.1 s |
| **ay 5.0, brake 0.3** | **9.14 s** | **9.01 s** | **0 / 40** | **91.4 s** |
| ay 5.5, brake 0.3 | 9.20 s | 8.99 s | 0 / 39 | 92.0 s |
| ay 6.0, brake 0.3 | 9.44 s | 9.26 s | 0 / 38 | 94.4 s |

| ICRA, never tuned on | median | collisions | race |
|---|---|---|---|
| safe, ay 3.5, no braking | 17.36 s | 0 / 27 | 173.6 s |
| **ay 5.0 + braking** | **16.53 s** | **0 / 29** | **165.3 s** |

Two subtleties worth keeping:

- **Braking alone is slower.** At ay 3.5 it costs 0.17 s/lap, because the car had
  been overshooting a conservative profile and getting away with it — coast-down
  was acting as a free speed bonus. Braking only pays once the profile is
  aggressive enough to need it. Do not evaluate the two changes separately.
- **ay 6.0 turning back is a real limit.** Past ~5 the car cannot deliver the
  planned curvature and runs wide (cleanly — `ay_limit` catches it). So there is
  still a ceiling; it is just further out and differently caused.

`brake_gain` 0.5 measured 91.9 s against 0.3's 91.4 s, so 0.3 stands.

### ay_limit swept — not the lever (Aug 28, later)

The mismatch flagged below (cap 4.5 while the profile plans 5.0) turns out not
to matter. Porto, same session:

| ay_limit | median | best | collisions | race |
|---|---|---|---|---|
| **4.5 (ships)** | 9.14 s | 9.01 s | 0 / 40 | **91.4 s** |
| 5.0 | 9.14 s | 8.95 s | 0 / 51 | **91.4 s** |
| 5.5 | 9.11 s | 8.82 s | 1 / 39 | 93.6 s |

Flat from 4.5 to 5.0 and collisions appear at 5.5. **The steering cap is not
binding — the planned ay is.** Keep 4.5; do not re-sweep without a reason.

### What to do next

1. Re-measure anything concluded pre-brakes (see below) — that is where the
   remaining time is, not in the steering cap.
1b. **Submit different configs for the two phases.** Qualification is explicitly
   speed-independent ("complete 10 laps; speed doesn't matter") while the final
   is ranked purely on time — opposite risk profiles, and we currently ship one
   config for both. Phase 1 (Sep 12): the most robust thing we have, since a
   faster lap earns nothing and a collision only hurts. Phase 2 (Sep 19): the
   fastest clean config. Selectable at runtime already via `PIT_CREW_MODE` /
   `PIT_CREW_LINE` / `PIT_CREW_SPEED_SCALE`.
1c. **Open TODO — teleport-robust centerline tracing.** Priyanshu's Fault 4
   (excise the retry segment between a >0.5 m jump and the nearest past point).
   Two earlier attempts were reverted for breaking the ICRA line, so START a
   session with this, never end one with it, and gate it on
   `./scripts/check_pipeline.sh`. Not near a deadline.
2. Re-measure anything that was concluded pre-Aug-28 **with brakes on**. The
   grip watchdog, the optimized raceline, the corrected LiDAR map and the
   `w_veh` sweep were all evaluated against a car that could not slow down, so
   their negative results are not safe to trust any more.
3. Cold-start the built submission image and confirm it races the new profile.
4. Still untouched: `new_track_drill.sh` end to end on a fresh `*-compete` tag.

---

## Aug 28 (earlier) — Vidit Gupta's review, superseded in part by the section above

Worked through **Vidit Gupta's review** (`docs/Vidit Gupta/25 August/`). Full
response, with every verdict re-derived against current main, is in
`docs/reviews/vidit-gupta.md`. Six real defects fixed; the performance idea was
real offline and lost when driven.

**Two things here change how you read every other number in this file.**

### 1. The simulator now runs on an offscreen display, and it is faster

`sim_restart_connect.sh` runs the sim under **Xvfb :99 inside its own
container** instead of on the host X server. Two reasons: it no longer hijacks
the pointer of whoever is using the machine, and it does not compete with the
desktop compositor for the GPU.

The second one moved the numbers. The shipping config, byte-identical line and
controller, measured **97.2 s** on Aug 28 against the **101.2 s** recorded on
Aug 23. Every lap time in this file from before Aug 28 was measured under GPU
contention and is **pessimistic by roughly 0.4 s/lap**.

Rankings measured A/B inside one session are unaffected. **Cross-session
comparisons against pre-Aug-28 numbers are not valid** — that mistake is what
made a losing line look like a winner for an hour today.

This is not the frozen-LiDAR trap (`-batchmode -nographics`). Xvfb is a real X
server, Unity really renders, and the GPU raytracer is reached through CUDA:
measured 10.97 Hz with ten consecutive scans all distinct. There is no API to
connect the sim without a click — `-ip`/`-port` only pre-fill the fields, Unity
PlayerPrefs hold no connection state, and Unity ignores XSendEvent — so the
click is still a real click, just on a display nobody is looking at.
`VISIBLE=1 ./scripts/sim_restart_connect.sh` puts it back on the real display
for watching a run; it takes over the pointer, so only use it when asked.

### 2. Offline ideal lap time mispredicts the driven ranking

`make_track_lines.py` picks `race.csv` as the fastest **ideal** lap above the
clearance floor. On Porto that rule chose a 9.28 s-ideal line over a 9.59 s-ideal
one, and the 9.28 s line **drives slower** (9.43 s median with 4 collisions in
38 laps, versus 9.72 s clean). With `MIN_CLEARANCE` at 0.22 m the pipeline would
have shipped the worse line on a track nobody had driven.

`MIN_CLEARANCE` is now **0.30 m**, from driven evidence rather than an error
budget: 0.283 m collides, 0.301 m and 0.325 m do not. Porto selects `safe` again.
Do not lower it without driving the line that the lower floor admits.

### Measured Aug 28 — same session, same sim, 37-40 laps each

| config | median | best | collisions | 10-lap race |
|---|---|---|---|---|
| **`safe`, ay 3.5 (shipping)** | **9.72 s** | 9.59 s | **0 / 37** | **97.2 s** |
| `w_veh 0.44` @ scale 0.97 | 9.69 s | 9.52 s | 1 / 40 | 99.4 s |
| `w_veh 0.44` @ scale 1.00 | 9.43 s | **9.29 s** | 4 / 38 | 104.8 s |
| corrected LiDAR map + `w_veh 0.52` | 9.90 s | 9.74 s | 0 / 37 | 99.0 s |

`--w-veh` had never been pushed past its default and `sweep_lines.py`'s grid
stopped at 0.32 (and tied `--ax` to `--ay`). That is where **"on Porto the
geometry is settled" came from, and it was wrong** — the optimized line beats
the centerline by 0.31 s of ideal lap for 13% of the clearance, not 0.05 s for
44%. `make_track_lines.py` now sweeps `w_veh` per track and lets the gates
choose. On ICRA every optimized line is still rejected on the steering lock.

The line still loses when driven, because all four of its collisions are at its
own tightest point, and because tighter lines track worse: driven/ideal is 1.4%
on the safe line and 4.2% on the optimized one. The corrected-LiDAR map (lateral
p95 0.155 -> 0.041 m) did not rescue it either.

### What to do next

1. **Do not chase lap time through line geometry on Porto.** It has now been
   measured three ways and the plain centerline wins every time.
2. The remaining honest gap is the **driven/ideal ratio on tight lines** — the
   controller loses 4.2% on the optimized line against 1.4% on the safe one.
   That is a tracking question, not a planning one, and it is the first thing
   that has pointed at the controller in a while.
3. Re-baseline anything you intend to compare against: pre-Aug-28 numbers are
   ~0.4 s/lap pessimistic.
4. Still untouched from Aug 23: run `new_track_drill.sh` on a genuinely fresh
   `*-compete` track end to end, and cold-start the built submission image.

---

## HISTORICAL — START HERE as of Aug 23 (superseded by the Aug 28 section above)

**Shipping: safe centerline + MPC, ay 3.5 / ax 6.5 / brake 4.0 / jerk 20, with
`ay_limit` 4.5. Porto 101.2 s race, best 9.81 s; ICRA 17.95 s/lap; zero
collisions on both.** Down from 115.0 s at the start of Aug 23.

**Before any commit that touches `tools/` or `src/tools/`, run
`./scripts/check_pipeline.sh`.** It regenerates every track and fails if one
stops producing a drivable line or if a shipping artefact moves a byte. It
exists because a change validated on Porto shipped having broken ICRA
completely, and nothing said so.

**Today's retained result is evidence, not a new shipping default.** A
controlled replay of the same 176 s Porto mapping bag isolated the localization
variables:

| map / sensor model | motion noise | total median / p95 | lateral median / p95 |
|---|---:|---:|---:|
| original map, zero offset | 0.035 m | 0.144 / 0.370 m | 0.027 / 0.155 m |
| corrected map, 0.1933 m offset | 0.18 m | 0.041 / 0.103 m | 0.016 / 0.041 m |
| corrected map, 0.1933 m offset | 0.035 m | **0.046 / 0.084 m** | **0.015 / 0.039 m** |

The LiDAR mount/map consistency is the dominant improvement. Merely reducing
particle motion noise on the old map is a regression. A live run of the combined
candidate completed **17 laps, 0 collisions**, best 9.89 s, and measured lateral
p95 0.047 m; it did not beat the shipping 9.81 s. High-speed and optimized-line
attempts still collided, so none of those controller/profile experiments were
retained.

**TODO — do not resume with another blind ay sweep:**

1. Make corrected-vs-uncorrected map selection carry its matching
   `lidar_offset` into runtime; the drill currently promotes map files but loses
   which sensor origin generated them.
2. Validate the corrected-map candidate on ICRA and then on one untouched track.
   Do not change defaults until both are clean.
3. Step speed up progressively (ay 4, 5, 6), one variable at a time, using the
   corrected localization. Jumping directly to the reconstructed winner profile
   caused repeated collisions and MCL resets.
4. Replace or substantially improve the current candidate-grid kinematic MPC
   before attempting ay 8-9. Winner telemetry proves that envelope exists, but
   the present tracker cannot hold the optimized path there.
5. After any candidate improvement: 20-30 Porto laps, an ICRA run,
   `check_pipeline.sh`, and a cold-started submission-image test. Keep the
   **9.81 s / zero-collision** artefact as the fallback throughout.

Public-code audit: VAUL's winning integration is not published; its public
competition repository contains an entrypoint placeholder/wall follower. The
public ForzaETH repository contains no runnable winning stack, and the public
RL-controller repository advertises a trained policy but does not contain the
model artefact (and consumes restricted scoring topics). There is therefore no
honest winner-stack cross-drive available today.

## Steps only the human can do

- Register (done by user), push the image to DockerHub, and submit the Phase 1
  and Phase 2 Google Forms. Everything else in this document is automated.


Updated 2026-08-07; stale sections reconciled 2026-08-22. Every number here
came from a run; nothing is a design estimate. Sections are append-only and in
rough chronological order, so where an early conclusion was later overturned it
is marked SUPERSEDED rather than deleted — the corrections are the useful part.

## The stack

**As shipped** — the tracker publishes to the car directly:

```
LiDAR ── localization ──── pose ──── mpc ─────► car
            (MCL vs map)          (raceline)

LiDAR ── follow_the_gap ──────────────────────► car     (PIT_CREW_MODE=reactive:
                                                         map-free, any track)
```

The two modes are selected at container start and never switch mid-run. The
`arbiter` that used to sit between them is **deliberately not started** — it
caused the collisions it was meant to prevent (see "do not re-derive", below).

Offline pipeline, all track-agnostic — this *is* the finals drill, and
`scripts/new_track_drill.sh` runs it end to end in ~271 s measured:

```
record a recon lap → build_map_from_bag.py → extract_centerline.py
                   → make_track_lines.py   → footprint_analysis.py → race it
```

## Measured results

| What | Result |
|---|---|
| **Porto, shipping** | **27 laps, 0 collisions**, best **9.81 s**, median 10.12 s, race **101.2 s** |
| **ICRA (never tuned on), shipping** | **16 laps, 0 collisions**, median **17.95 s/lap** |
| *(historical: Porto 22 laps best 11.32 s, ICRA 20.9 s/lap, before Aug 23)* | |
| Follow-the-Gap alone | **11 laps, 0 collisions**, best 33.43 s — clears the qualification bar |
| Localization, offline on an unseen bag | median 0.093 m, p95 0.299 m |
| Localization, **shipping live** | median 0.092 m, p95 0.298 m; lateral p95 0.17 m |
| Localization, **corrected-map candidate** | live 17 clean laps, lateral p95 **0.047 m**; offline total p95 **0.084 m**, lateral p95 **0.039 m** |
| Tyre, steady state on open floor | holds ~0.9 of asked curvature to **9.9 m/s2**, then collapses abruptly |
| Tyre, usable on track | **ay 3.5** — set by the grip cliff against 0.17 m of position uncertainty |
| Pure pursuit tracking error | median 0.12 m, p95 0.36 m |
| Pure pursuit lap time | 20.7 s (but not yet collision-free) |
| Optimized raceline | 27.0 m, ideal lap ~10.4 s |
| Centerline "safe" line | 29.7 m, ideal lap ~11.0 s |
| Throttle calibration | **0.040 throttle per m/s** |

## Findings that cost the most time (do not re-derive)

1. **Headless mode freezes the LiDAR.** It is GPU-raytraced; `-batchmode
   -nographics` serves a stale scan forever. The sim must render.
2. **Encoder speed WORKS — this entry used to say the opposite and was wrong.**
   `position` is the cumulative wheel angle in radians, so speed is
   `wheel_radius * d(position)/dt`; scale comes out 0.993 and 0.999 against IPS
   on two tracks. The |r| < 0.2 correlation that condemned it was measured on a
   stretch where the car was stuck, reversing and being teleported to
   checkpoints, which makes any derivative look like noise. **Judge a sensor on
   clean data.** Encoders are now the filter's motion source; throttle
   *actuation* is still open loop at the calibrated 0.040 per m/s.
3. **Throttle gain was guessed 3.6× too high** (0.14 vs the measured 0.040),
   which is what made the car overspeed a 1.3 m corridor and crash constantly.
   Calibrating it produced the first clean laps.
4. **The pose is the rear axle, but the car is 0.5 m long.** A path whose
   *centre* clears the wall can still put a body corner through it: the old
   raceline had 0.335 m centre clearance and **0.000 m corner clearance**.
   `optimize_raceline.py` now shrinks the corridor by the curvature-dependent
   swept width and re-optimizes.
5. **A safety layer without hysteresis is worse than none** — re-deciding every
   scan chattered ~3200 times in one run and ground the car along the wall.
   Modes now have separate enter/exit thresholds and a dwell time.
6. **Braking is the wrong reflex here.** Stopping parks the car against a wall;
   FTG steers around obstructions and laps cleanly. The safety response is now
   "hand over to FTG", with a true stop only for imminent contact.
7. Collisions **teleport the car to a checkpoint**, so a logged collision
   position is the *reset* point, not the impact point — look ~0.4 s earlier.
8. TUM's `psi` column is **north-zero**, not standard math angles.
9. **The safety arbiter caused the crashes it was meant to prevent.** With the
   arbiter in the loop the safe line collided constantly (18-29 collisions, no
   completed laps); with the identical line, speed and localization but pure
   pursuit publishing directly, it ran clean. Handing control back and forth
   between two controllers with different steering states destabilises the car
   far more than the hazards it was reacting to. Any safety layer must be
   re-earned with evidence, not assumed to help.
10. **Feeding a velocity input into the filter made it worse.** The calibrated
   throttle->speed curve is a straight-line fit, so it over-predicts through
   corners: the estimate led reality by ~50 ms and median error degraded
   monotonically with the gain (0.093 m at zero, 0.157 m at k=32). The
   likelihood field localizes better with a plain random walk. Live error
   dropped 0.140 -> 0.092 m by switching it off.

## Qualification: met

Follow-the-Gap alone ran **11 consecutive laps with 0 collisions**, lap times
33.43-33.57 s (spread under 0.15 s). Qualification requires 10 clean laps and
ignores speed, so this configuration clears it. The submission image runs it
with `PIT_CREW_MODE=reactive`, needs no map, and therefore works on a track we
have never seen — which also makes it the safe finals fallback.

## Raceline stack: clean, and faster than FTG

`scripts/run_pp_only.sh` (localization + pure pursuit, **no arbiter**).

| speed_scale | result |
|---|---|
| 0.30 | **3 laps, 0 collisions, best 32.43 s** — beats FTG's 33.43 s |
| 0.40 | best 28.02 s but ~1 collision/lap (4 over 3 laps) |
| 0.50 | 6 collisions in 1 lap, best 40.7 s — *slower*, resets cost more than the pace buys |

0.30 is the shipping default. 0.40 is quicker per clean lap (28.0 s) but at
~1 collision/lap it costs +10 s each, so it loses on total race time; worth
revisiting once tracking at speed improves.

## New-track drill: 4.5 minutes, verified on an unseen circuit

`scripts/new_track_drill.sh <name> <sim_tag> <seconds>` runs the whole pipeline
as one timed procedure: reconnaissance (drive blind with FTG and record) → map
→ centerline → racing lines → footprint clearance check.

Validated against the **2026-icra-compete** image, which ships a different and
much larger circuit this stack had never seen (57.9 m loop vs Porto's 28.6 m,
min corridor 1.33 m vs 0.88 m). **Total: 271 s** — against a three-day window
between the finals track release and the deadline. The generated safe line has
0.414 m worst-corner clearance; the optimized line came out at 0.075 m, which
the footprint check flags as undrivable *before* we ever drive it.

Past competition images are useful unseen tracks for practice: `*-compete` tags
carry the real competition circuits, while every `*-practice` tag is Porto.

**Driving result on that unseen track is honest but not yet clean:** the
generated stack laps it, best 67.8 s, but at ~1 collision/lap at speed_scale
0.30 *and* 0.20 — so on this circuit the problem is not speed. The likely cause
is recon quality: the blind FTG drive that produced the map itself had contacts
(0.93% of samples with a body corner inside a wall, versus 0.00% on Porto), so
the map is built from a worse trajectory. Next: improve the reconnaissance lap
(slower FTG, or a second mapping pass driven from the first map) before trusting
the drill end-to-end on an unfamiliar circuit.

## Speed: 32.43 s -> 11.21 s with Stanley (2.9x)

`scripts/run_stanley.sh 1.0 track_v2 safe` — localization + Stanley tracker,
full profile speed: **3 laps, 0 collisions, best 11.21 s**, cross-track error
~7 mm.

**Superseded (Aug 7): the shipping controller is now MPC**, via
`CONTROLLER=mpc ./scripts/run_stanley.sh 1.0 track_v2 safe`. It measures 22
laps / 0 collisions / best 11.32 s on Porto, and it is the only controller
verified on a track it was not tuned on (ICRA, 13 laps, 0 collisions, full
speed). Stanley has neither the distance-based horizon nor stuck detection, so
it has not been validated on an unseen circuit; it stays available through
`PIT_CREW_CONTROLLER=stanley`. Everything below still applies — MPC inherits
the curvature feedforward, pose prediction and 50 Hz loop.

What made the difference, in order:
1. **Curvature feedforward.** Most of the steering now comes from the plan
   (atan(L*kappa)), so feedback only corrects the residual. Cross-track error
   fell from ~120 mm (pure pursuit) to ~7 mm.
2. **Front-axle reference (Stanley).** Stable under heading error, where pure
   pursuit cuts corners.
3. **Pose prediction + 50 Hz control.** All sensors arrive at ~10 Hz on one
   WebSocket so there is no extra information between scans, but the pose can
   be dead-reckoned to 'now' and the loop run faster than the sensor rate.

Measured along the way:

| Configuration | Result |
|---|---|
| Stanley, safe line, scale 1.0, ay 3 | **11.21 s, 0 collisions** ← shipping |
| Stanley, safe line, ay 4 | 10.58 s but 1 collision — loses on race time (+10 s each) |
| Stanley, safe line, ay 6 | 1352 collisions — looked like a grip limit near ay 3-4 (see Aug 22 note) |
| Stanley, **optimized** raceline | 12.28 s, 17 collisions — its 0.146 m corner clearance cannot absorb residual error |
| Pure pursuit, safe line | 32.43 s, 0 collisions |

Two lessons worth keeping: the **safe (centerline) line beats the optimized
raceline** on this track because clearance matters more than path length once
collisions cost 10 s; and a **long braking horizon double-counts** the profile's
own braking distance (52 s/lap at 1.2 s horizon vs 11 s at 0.35 s).

## Steering oscillation (the visible left-right swinging)

Measured with `tools/steering_smoothness.py`: **1.68 steering reversals per
second** and a p95 steering rate of **4.32 rad/s against a 3.2 rad/s actuator**
— we were commanding faster than the wheels can physically turn, and phase lag
in a feedback loop is what turns a correction into an oscillation. Mean lock was
0.293 rad (17 deg) just to hold a line.

Three causes, all now addressed at source (filtering the output harder only adds
more lag):
1. **The pose snapped.** The filter publishes a resampled-particle mean at
   10 Hz, so consecutive fixes differ by centimetres; the controller ran at
   50 Hz on a pose that teleported every 100 ms. Now each fix is blended into a
   continuously-propagated state.
2. **No actuator rate limit.** Commands now respect 3.2 rad/s.
3. **Noisy heading reference.** Path heading was computed between raceline
   points 0.1 m apart, where centimetre spline wiggle dominates; now taken over
   a ~0.5 m baseline, and curvature smoothed to match.

| Config | Lap | Collisions | Reversals/s | Steer rate p95 | Mean lock |
|---|---|---|---|---|---|
| Original gains | 11.21 s | 0 | 1.68 | 4.32 (over limit) | 0.293 rad |
| + all three fixes | 11.74 s | 1 | 1.68 | 4.12 | 0.217 |
| + low feedback gains | 12.06 s | 2 | 1.50 | 3.20 (at limit) | 0.142 |

**Honest state:** mean steering effort halved, but smoothness trades against
tracking, and the *reversal rate* barely moved. Part of it is geometric (this
track genuinely alternates direction) and part is fundamental: any feedback
controller reacting to error sampled at 10 Hz will ring. Follow-the-Gap shows
the same swinging and uses no localization at all, which confirms the sensor
rate — not the pose — is the floor.

**MPC was built on that argument and it worked.** `src/pit_crew/mpc.py`,
selected with `CONTROLLER=mpc ./scripts/run_stanley.sh`:

| Controller | Lap | Collisions | Reversals/s | Steer rate p95 | Mean lock |
|---|---|---|---|---|---|
| Pure pursuit | 32.43 s | 0 | — | — | — |
| Stanley | 11.21 s | 0 | 1.68 | 4.32 (over limit) | 0.293 |
| Stanley, low gains | 12.06 s | 2 | 1.50 | 3.20 | 0.142 |
| **MPC** | 11.47 s | **0** | **1.42** | **2.80 (within limit)** | 0.204 |

MPC is the only controller that keeps the steering rate inside the actuator's
3.2 rad/s limit while staying collision-free — it optimises a steering sequence
subject to the constraint rather than reacting pointwise, so smoothness costs no
accuracy. Stanley remains the shipped default until MPC has more laps behind it;
both are collision-free at ~11.2-11.5 s.

## Generalization: what transfers and what does not

Tested by regenerating everything for the **ICRA 2026 competition circuit**
(different, larger, and never seen by this stack) and racing it with the
identical controller and gains.

**Transfers (vehicle properties, not track properties):** Stanley gains, the
grip limit (ay ~3), throttle calibration (0.040 per m/s), localization
parameters, braking horizon. None of these were re-tuned per track.

**Root cause found: the error budget does not fit on ICRA.** Localization there
is far worse than on Porto — without a velocity input the filter *diverges*
(mean 1.48 m, p95 7.9 m, 5 relocalizations), because the car covers ~0.3 m
between 10 Hz scans on a track twice Porto's size and a random walk cannot keep
up. Re-enabling the velocity input fixes the divergence (mean 0.236 m, p95
0.54 m, 0 relocalizations) — **disabling it had been overfitted to Porto, and
that default is now corrected.** But driving still fails, because:

| Track | localization p95 | safe-line corner clearance | fits? |
|---|---|---|---|
| Porto | 0.30 m | 0.325 m | yes — clean |
| ICRA | 0.54 m | 0.215 m | **no — 2.5x over** |

So ICRA needs *both* better localization and more clearance. Concrete levers,
untried: raise map resolution (0.025 m may be coarse for a 58 m track), more
particles with the velocity input, and a line that gives up lap time for
clearance where the corridor is tight. Recon quality still matters but is no
longer the leading suspect.

**Earlier hypothesis, now superseded — the rewrite did not fix it:** Anchoring the
centerline to the driven trajectory produced clean geometry (uniform 0.153 m
spacing, closes to 0.154 m, no discontinuities, 1.33 m corridor) and better
clearance (0.075 -> 0.215 m), but driving it was *worse*: 581 collisions, no
completed laps, cross-track error 0.58 m. So the line is not malformed — the
failure is downstream (localization on that map, or the line faithfully
following an excursion the messy recon drive made). **Both** centerline methods
fail on the ICRA recon, which points squarely at task #18: the blind
Follow-the-Gap reconnaissance lap on an unknown track is not good enough to
build a map from. Fix the recon first, then re-test.

**Did NOT transfer: the centerline extraction.** The skeleton method assumes
the drivable area is a simple loop. The ICRA circuit has an inner passage, so
the flood-filled corridor merged regions that are not connected in the driving
sense and the skeleton cut across the infield — the car followed a line through
a wall (18 collisions; slowing to half speed made it *worse*, 116, which is what
ruled out speed). Fixed by anchoring the centerline to the driven trajectory,
which is evidence of what is actually drivable. **Not yet re-verified by
driving** — that is the next run.

**Caveat on the headline number:** 11.21 s is measured on Porto only. Treat it
as a Porto result until the ICRA rerun confirms the pipeline end to end.

## Historical: where the 5x gap was

Profile tuning is exhausted; the limit is the **controller**, not the plan.
Measured lateral-acceleration demand vs collisions:

| peak speed (p95) | lateral accel (p95) | collisions |
|---|---|---|
| 1.73 m/s | **2.22 m/s²** | **0** |
| 2.66 | 5.57 | 7 |
| 3.25 | 8.04 | 12 |
| 3.94 | 9.74 | 23 |

Three profiles were tried and all confirm the same ceiling: ay = 3 (ideal 11 s,
clean only when scaled to 0.30), ay = 8 (ideal 6.8 s — right ballpark, but
crashes), and ay = 2.5 with vmax 2.0 built from the measured numbers (ideal
15.4 s, still crashes). The clean run *averaged* 0.92 m/s, so sustained pace,
not peak, is what the controller holds. Winners average 4.4 m/s.

**Conclusion: to approach 6.5 s we need a better tracking controller, not a
faster plan.** Pure pursuit is a geometric reactive controller fed by a 10 Hz
pose; at speed it cannot hold a 1.3 m corridor. Candidates, in order of
expected payoff:
1. **Curvature feedforward + cross-track feedback** (or Stanley) instead of
   pure geometric pursuit — the raceline's curvature is known ahead of time, so
   steering should not be derived solely from a lookahead point.
2. **Higher-rate pose**: integrate IMU between LiDAR scans to publish at
   ~100 Hz instead of 10 Hz, so the controller is not steering on 0.1 s-old data.
3. **MPC** over the raceline, which handles the actuator rate limit
   (3.2 rad/s) explicitly — the likely reason aggressive steering fails.

## Open — HISTORICAL (Aug 8), all of it since closed

- ~~Build the winners' stack and benchmark against it~~ — done Aug 22; there is
  no published stack to run, and what is public is documented below.
- The arbiter needs rebuilding or dropping (#11). Still true, still not shipped.

Closed since this section was written: `speed_scale` is 1.0 and clean on both
tracks, and the optimized line was driven and **rejected on measurement** (57
collisions, 6 zones) — it is not "left on the table", it does not work here.


## HISTORICAL PLAN (Aug 8) — superseded, kept for the reasoning

The current starting point is at the top of this file. This section is what was
planned on Aug 8 and every step of it has since been carried out.

**Implement the winners' stack and compare it against ours on the same track,
same sim, same vehicle.** Decided Aug 8 after mapping our own speed frontier and
finding it tops out at ~10.5-11.3 s against their published 6.53 s.

Why this and not more tuning: every lever we own has been swept and measured
(see "Closing on the winners" below). ay, corridor margin, curvature bound, and
line choice all trade collisions for time along a frontier that does not reach
6.53 s. We track our own plan to 2.4%, so the tracker is not the problem. The
remaining gap must be something structural in how they plan, and the only
honest way to find it is to run their code and measure the difference rather
than keep guessing at ours.

Order of work:

1. ~~**Validate the benchmark first**~~ — **DONE Aug 22. It did not reset the
   target, it sharpened it.** Same car, same track mesh, same physics, verified
   asset-by-asset. It also showed the tyre model allows **ay 9.81 m/s2** where
   we cap at 3.0. See "Step 1 DONE" below.
2. ~~**Get VAUL's stack running**~~ — **DONE Aug 22: there is nothing to run.**
   Everything public is their dev environment plus forks of third-party
   components; the winning autonomy stack and its tuning are not published.
   See "Step 2" below for what the code does reveal, including the 0.32 m
   corridor width they plan with against our 0.325 m of measured clearance.
3. **BLOCKED as written** (no raceline of theirs to diff) — replaced by:
   **cut the localization p95 and spend it on corner radius.** Both the grip
   curve and their 0.32 m corridor point at the same constraint. Concretely:
   measure where the filter's error actually comes from at racing speed, then
   re-run the `--w-veh` sweep with the margin that a better p95 justifies. The
   old sweep said the frontier was flat (0.146 m -> 10.41 s vs 0.292 m ->
   10.67 s) but it was run at ay 3.0, below the 4.5 m/s2 the tyres will give.
   ~~Diff the artefacts, not the prose~~: their raceline vs ours on the same
   map — path length, peak curvature, curvature p99, worst-corner clearance,
   and the velocity profile's ay ceiling. Our numbers to beat, all on Porto:
   ideal 11.05 s, peak kappa 1.019, clearance 0.325 m, ay cap 3.0.
4. **Drive their line with our controller, and our line with theirs.** That
   separates "better plan" from "better tracker" in two runs. Our controller
   tracks to ~7 mm, so if their line is faster on our stack, the win is in the
   planner.

The specific thing to look for: our optimized line's *peak* curvature (1.717)
comes out **worse** than the plain centerline's (1.019). Min-curvature reduces
the integral of kappa and is free to spike locally, and those spikes are what
saturate the steering at exactly the two corners where ay 4.0 fails. If they
bound peak curvature, or use a different objective, that alone could be the
difference.

## Step 1 DONE: the 6.53 s benchmark is real, and the tyres have 3x the grip we use (Aug 22)

**The benchmark is valid.** The 6.53 s lap was set on `2025-cdc-tf-practice`;
we develop on `2026-iros-practice`. Those are different simulator builds, so
the first question was whether the comparison means anything at all. It does —
verified from the Unity assets themselves, not by inference
(`tools/compare_sim_builds.py`, re-runnable on any pair of tags):

| | 2025-cdc-tf-practice | 2026-iros-practice |
|---|---|---|
| Unity engine | 2022.3.52f1 | 2022.3.52f1 |
| PhysicsManager + TimeManager | gravity 9.81, 6 solver iters, **1 ms** fixed step | identical |
| Rigidbody | mass **3.47 kg**, drag 0.273 | identical |
| WheelCollider x4 | radius 0.059 m, spring 500 / damper 100, same friction curves | identical |
| Vehicle transform subtree | 248 of **249** parts identical | " |
| Active track | `Porto Track` | `Porto Track` |
| Track mesh | md5 `6c879809d3e8`, 5 970 752 B | **byte-identical** |
| Checkpoints (24) + spawn points | identical | identical |

The single differing vehicle part is the scene placement of the car root
(z 0.74 -> 0.80), which the spawn point overrides at runtime. Same car, same
track, same physics. **Our 11.32 s against their 6.53 s is a real 1.73x gap and
the target stands.** This retires the "maybe their sim just grips differently"
escape hatch.

**What the same comparison turned up is more useful than the answer.** Reading
the tyre model out of the asset:

| | value | implies |
|---|---|---|
| lateral mu at peak | 1.00 (extremumValue 1.0 x stiffness 1.0) | ay 9.81 m/s2 — *transient only, see below* |
| ...at slip | **0.01** | a very narrow window |
| lateral mu at asymptote | 0.50 | ay 4.91 m/s2 |
| ...at slip | 0.10 | |
| longitudinal mu at peak | 0.72 (0.9 x 0.8) | ax up to 7.1 m/s2 |

Unity scales `WheelFrictionCurve` values by wheel load, so value x stiffness is
effectively mu. We cap the velocity profile at ay 3.0. **The 9.81 figure is the
peak of the curve and the car cannot live there — the measured plateau is
4.5 m/s2, the asymptote. The real headroom over our cap is ~1.5x, not 3x.**

This reframes every grip conclusion in this document. The ~5 m/s2 ceiling we
kept hitting is not the friction limit, it is the **asymptote**: past 0.10 slip
the lateral force halves to 4.91 m/s2, which is almost exactly where our runs
fall apart. And we know independently that the ay-4.0 run had steering at full
lock **20% of samples** — i.e. it was past peak slip and riding the asymptote.
So the failure was never "the tyres let go at 3-4"; it was a controller and a
line that could not hold slip inside a 0.01-0.10 window.

It also resolves the arithmetic that made the target look impossible. 4.38 m/s
average over an R = 0.98 m corner appeared to need 9-20 m/s2, above any
plausible limit — but that estimate wrongly assumed the tightest corner is
taken at the average speed. Against a real 9.81 m/s2 peak, a 6.53 s lap sits
inside the tyre model, not outside it.

**Driven the same day, and it cut the headroom claim down.** See the next
section: the peak is real but unreachable in sustained cornering, and the
number that matters is the *asymptote*, 4.91 m/s2. Treat "3x the grip we use"
as wrong and "about 1.5x" as right.

## Driving the jerk-limited profile: partly right, and the grip question settled (Aug 22)

Step 1 predicted that our failures above ay 3.0 come from crossing the tyre's
narrow peak-slip window at curvature *transitions*, not from running out of
grip in the corners. The test: same geometry, only the speed profile changed.

`optimize_raceline.py --jerk-lat` caps d(ay)/dt. The shipping ay-3.0 profile
peaks at 21 m/s3 and runs clean over 22 laps, so 20 m/s3 is a level known to be
survivable; an unlimited ay-4.0 profile peaks at 32 m/s3.

| profile | ideal | driven | best lap | collisions |
|---|---|---|---|---|
| safe, ay 3.0 (shipping) | 11.01 s | 22 laps | **11.32 s** | **0** |
| safe, ay 4.0, no jerk cap | 9.57 s | 21 laps | 10.11 s | 23 |
| **safe, ay 4.0, jerk <= 20** | 9.56 s | **20 laps** | **9.96 s** | **10** |

The path is byte-identical to the shipping line, so clearance is unchanged at
0.325 m and the only variable is speed.

**The prediction half-held.** Collisions fell from 23 to 10 at the same ay cap
and the same lap count, and the best lap improved to 9.96 s. Localizing them
the way the ICRA spur was found: the 10 events are only **4 incidents**, one of
which is a collision-reset loop — the car hits, is teleported to the checkpoint,
drives the same 0.8 m and hits again, six times. Every incident sits at
kappa 0.04-0.70, never at the tightest corner (1.019). So the failure really is
transitional. But 4 incidents is not clean, and 10 collisions cost +100 s. **Not
shippable.**

**What the run settled is more valuable than the lap time.** Differentiating the
IPS trace gives the yaw rate the car actually achieved, which can be compared
against what the steering angle asked for (`kappa = tan(delta)/L`) — the
measurement `skidpad.py` needs open space for, extracted from a normal racing
lap instead (`tools/grip_curve.py`):

| steering | kappa asked | achieved | ratio | ay achieved |
|---|---|---|---|---|
| 0.05-0.10 | 0.24 | 0.24 | **1.01** | 2.52 |
| 0.10-0.15 | 0.38 | 0.34 | 0.88 | 3.98 |
| 0.15-0.20 | 0.54 | 0.42 | 0.80 | **4.49** |
| 0.20-0.25 | 0.67 | 0.55 | 0.79 | 4.31 |
| 0.25-0.30 | 0.87 | 0.58 | 0.68 | 4.38 |
| 0.30-0.40 | 1.13 | 0.62 | 0.52 | 4.09 |
| 0.40-0.52 | 1.49 | 0.64 | 0.42 | 3.87 |

Below 0.10 rad of steering the car turns exactly as the bicycle model says.
Past ~0.15 rad the front tyres saturate and **more steering buys no more
lateral acceleration at all** — achieved ay flattens at 4.1-4.5 and then
declines while the asked-for curvature keeps climbing. At full lock the car
achieves 37% of the curvature it is asking for.

That plateau, 4.5 m/s2, is the tyre model's **asymptote** (4.91), not its peak
(9.81). The peak sits at 0.01 slip, a window too narrow to hold, so in practice
the car races on the falling branch. **Corrected conclusion: usable grip is
about 4.5 m/s2 — real headroom over our ay 3.0 cap is ~1.5x, not 3x.** The old
"grip is genuinely limited near ay 3-4" entry was closer to right than the
Step 1 reading of the asset was.

**Where that leaves the pace problem.** Staying inside the linear region means
keeping steering under ~0.15 rad, i.e. planned curvature under
`tan(0.15)/0.324 = 0.46 1/m`. Porto's tightest corner is 1.019 — **geometrically
impossible to take inside the linear region without using track width to open
it**. Which returns to the clearance problem: opening corners costs the margin
that keeps us collision-free. That, not the ay cap, is the real constraint, and
it is what the winners must be solving differently.

## The grip watchdog works and is not worth shipping (Aug 23)

`ay_limit` is open loop — it caps what the controller may *demand* against a
constant. The closed-loop half is to watch what the tyres are actually
delivering: the skidpad showed they hold ~0.9 of the curvature asked of them up
to 9.9 m/s2 and then collapse to 0.24-0.35 within one 0.5 m/s step. Abrupt, but
the IMU can see it — the yaw rate stops following the steering.

`mpc.py: update_grip()` evaluates achieved/asked curvature per IMU sample (the
only new information, ~10 Hz), and on two consecutive bad samples scales the
throttle target down, clawing it back as grip returns.

**It does what it was designed to do.** At ay 5.0 collisions fell from **31 to
3**; at ay 4.0 from **2 to 0**. The detector is real.

**And it loses on race time in every configuration tested:**

| config | collisions | median | 10-lap race |
|---|---|---|---|
| **ay 3.5, no watchdog (shipping)** | **0** | **10.12 s** | **101.2 s** |
| ay 3.5 + watchdog | 1 | 10.79 s | 112.1 s |
| ay 4.0 + watchdog | 0 | 10.29 s | 102.8 s |
| ay 5.0 + watchdog | 3 | 10.01 s | 111.2 s |

It is not even free insurance: at the shipping cap it triggers on a clean
configuration, sheds speed that did not need shedding, and cost a collision it
was supposed to prevent. Default off.

**The row that matters is the median column: 10.0 to 10.8 s everywhere.** Plan
ay 3.5, 4.0 or 5.0, with a safety net or without, and the car laps at the same
speed. More aggressive plans do not produce more speed — they get clipped,
either by hitting a wall or by the watchdog clawing the speed back. Detecting
breakaway 0.2 s late is too late to convert into lap time, because at 10 Hz that
is the earliest this platform can know.

**That is the ceiling, demonstrated four ways now:** raising ay alone (31
collisions), opening the corners (5x fewer collisions, no net time), the tyre's
real 9.9 m/s2 (unusable), and now a working breakaway detector (no net time). We
are at the limit of this line and this error budget, and the limit is set by
10 Hz sensing against a grip cliff — not by the planner, the controller, or the
tyre.

## Lateral localization error is not tuning-limited (Aug 23)

Lateral error is what sets how much clearance a line must keep, so it sets how
close to the grip cliff we can plan. Three live configurations, measured against
IPS on the same track at the shipping profile:

| config | total median | total p95 | **lateral p95** |
|---|---|---|---|
| 800 particles, sigma_hit 0.06, 60 beams (shipping) | 0.174 m | 0.390 m | **0.168 m** |
| 2000 particles | 0.178 m | 0.389 m | 0.170 m |
| sigma_hit 0.03, 90 beams | 0.152 m | 0.391 m | **0.185 m** |

**Lateral p95 is 0.17 +- 0.02 whatever we change.** Particle count does nothing —
the filter is not particle-starved. Sharpening the likelihood field and adding
beams improves the *median* (0.174 -> 0.152) and makes the *tail worse*, and the
tail is the number clearance has to cover. Shipping keeps the best lateral p95.

Two things worth extracting from this:

**The error that grows with speed is lag, not drift.** Against the Aug 7
baseline at a much slower profile, lateral error is unchanged (p95 0.168 vs
0.171) while along-track error grew (p95 0.246 -> 0.377). Going faster does not
push us closer to the wall; it makes us less sure how far along the lap we are.
That is a much more comfortable failure mode, and it is why the 0.325 m line
stays clean at ay 3.5.

**The offline PF harness cannot be used for this.** `offline_pf.py` reports 192
relocalizations and p95 of 11 m on a track where the live filter does 0.092 m
median with zero relocalizations, with or without the `--centerline` argument it
shares with the live node. It does not reproduce live behaviour, so every number
above had to come from driving. Fixing that harness is its own task and would
make this kind of sweep minutes instead of hours.

So the clearance budget is a structural property of a 10 Hz sensor on a 0.025 m
map, not something waiting to be tuned out. Reducing it means changing the
information available — more sensor rate we do not have, or a second odometry
source — not the filter's parameters.

## Recon teleports and the centerline — REVERTED, the cure was worse (Aug 23)

Chasing the extractor blocker found something that matters more than the
blocker. **Waypoints for the centerline trace come from the driven recon
trajectory, and a recon run collides — that is what recon is.** Every collision
teleports the car back to a checkpoint, which appears as a >0.6 m jump between
adjacent 10 Hz samples and reads as a 180-degree heading flip. Waypoints
straddling one step *backwards* along the track, and the shortest path ordering
the centerline dutifully doubles back to reach them.

The ICRA recon contains **3 teleports in 1942 samples**, `maplap_short` has 58.

**`longest_clean_run()` was written to take the longest teleport-free stretch,
and it is REVERTED. It fixed the corrected-map case and broke the one we ship.**
On ICRA it cuts a 56.7 m loop's trajectory to 50.4 m; the traced line comes out
with a 0.421 m minimum radius and fails the steering-lock gate, so the ICRA line
stops existing entirely. It was committed having run only the Porto regression,
and shipped broken for one commit before the ICRA check caught it.

Two further attempts at the underlying lap-end detection were also wrong, kept
here so they are not repeated. Global closest-approach finds the *last* return
on a multi-lap run: a 28.6 m lap read as 134 m. The first local minimum of
distance-to-start is not the lap either — that signal has many minima as the
track weaves, and Porto's first sits 3.07 m from home at s = 21.1 m. The
original threshold ("first genuine return within 0.6 m after having left") asks
the right question and survives.

**The problem is real and still open.** Any cure has to preserve full loop
coverage — stitch the clean segments in order, rather than keep only the longest
one. The numbers below are what the reverted version achieved, recorded because
the diagnosis stands even though the fix did not.

| | traced length | reversals |
|---|---|---|
| corrected map, before | 94.7 m | 2 |
| **corrected map, after** | **57.8 m** | **0** |

against a true loop of 56.7 m. Porto regenerates byte-identical, so this costs
nothing where it was not needed.

**This was masked, not absent.** A smeared map's narrower skeleton physically
cannot follow a teleport jump; a sharper one can. It surfaced only because the
LiDAR correction sharpened the map — but the bug was always there, and the
finals recon will contain teleports too.

Two wrong diagnoses on the way, both cheap to discard and worth recording:
denser waypoints (10 -> 30) made it slightly *worse*, 88.5 -> 94.7 m, and
masking the skeleton to within 2.5 m of the driven line was a complete no-op
because the whole skeleton already is.

**The LiDAR offset still does not ship.** With the extractor fixed, ICRA's
corrected map produces a line rejected on min radius (0.543 m against the
0.561 m lock) — and opening that corner with `open_corners.py` clears every gate
but spends clearance down to **0.224 m**, against the 0.564 m the shipping line
has while driving 17.95 s/lap clean. Not a trade to make on an unseen track.

## The tyre really does make 9.9 m/s2 — and we still cannot use it (Aug 23)

`skidpad.py` was written to measure grip directly and had never run anywhere
usable: on Porto the slide widens the circle until the car grinds the walls.
**The `-explore` image turns out to have no track at all — only `Floor` is
active, every track object disabled.** That is the open space it always needed.
(It also had a latent bug: it called `rclpy.shutdown()` from inside its own timer
callback, tearing the context down under the spin that was running it and
burying the results in a traceback. Fixed.)

Steady state, constant steering, speed ramped:

| steer | ratio through the sweep | peak ay | breakaway |
|---|---|---|---|
| 0.10 | 0.88-0.94 | **6.24 m/s2** | never reached |
| 0.20 | 0.86-0.92 | **9.87 m/s2** | abrupt, ratio -> 0.35 |
| 0.30 | 0.84-0.92 | **8.37 m/s2** | abrupt, ratio -> 0.34 |

**This overturns what this document has said since the morning.** The claim was
that sustained grip is the tyre's 4.9 m/s2 asymptote and the 9.81 peak is
unreachable. Wrong: the car holds ~0.9 of the curvature it asks for all the way
to **9.87 m/s2**, essentially the asset's mu = 1.0. The earlier figure came from
race logs, where — as the steady-state filter showed — only **2% of samples are
steady** and none in the steering bins that matter. We had been measuring a car
mid-correction and calling it a tyre curve.

**And then it did not help at all.** Planned ay 5.0 with the controller's
`ay_limit` raised to 7.0: **31 collisions** over 25 laps. Best lap 9.10 s, the
quickest of the project, and a ten-lap race of **228 s** against 101.2.

Both numbers are real and the reconciliation is the useful part. Breakaway is
**abrupt** — ratio 0.9 to 0.35 within one 0.5 m/s speed step, with no
intermediate warning. On an open floor exceeding the limit costs nothing and you
drive out of it. In a 1.3 m corridor with 0.325 m of clearance and ~0.17 m of
lateral localization error, it costs a collision, and a collision costs +10 s.

**So the operative limit is not the tyre.** It is how close to an abrupt cliff
you can plan while holding enough margin to survive being wrong about where you
are. Measured, that is ay 3.5 — about 40% of the tyre's capability, and the
shipping configuration stays exactly where it is.

This is the same shape as every other result today: a true measurement, applied
in the wrong place, makes things worse. The tyre limit belongs in `ay_limit` as
an upper bound on what the controller may *demand*, not in the plan as a target
to aim at.

## The LiDAR mount offset: right, worth 1.6 s, and not shippable yet (Aug 23)

An MPC review flagged that the 0.2733 m LiDAR-to-rear-axle offset appeared to be
missing from both mapping and localization. It is. Measured directly out of the
simulator's scene rather than taken from the datasheet:

| | z (m) | relative to rear axle |
|---|---|---|
| rear axle centre | 0.640 | — |
| IPS / IMU | 0.720 | **+0.0800 m** |
| LiDAR | 0.9133 | **+0.2733 m** |

The pose we localize and race against is the **IPS point**, not the rear axle, so
what ray casting needs is the LiDAR's offset from *that*: **0.1933 m**. Rays were
being cast 0.19 m behind where they physically originate — larger than the entire
0.171 m lateral localization error we are trying to reduce.

**Correcting it in both the mapper and the filter measurably sharpens the map**,
which is the signature of a correct sensor origin:

| map | wall pixels before | after |
|---|---|---|
| Porto | 6.53% | **4.66%** |
| ICRA | 7.03% | **4.19%** |

**And it is worth 1.6 s a race on Porto**: 27 laps, 0 collisions, best 9.69 s,
median 9.96 s, ten-lap race **99.6 s** against 101.2 s. First sub-100 result.

**It is off by default anyway, because it breaks the unseen-track drill.** On the
sharper ICRA map the skeletonizer sees a different track topology: the traced
loop goes from 56.8 m to **88.5 m** with two direction reversals, and
`make_track_lines.py` correctly rejects every candidate — no drivable centerline
at all. The reversals are genuine doubling-back, not spurs: at one of them the
two legs never separate by 0.5 m within 60 points, so `excise_spurs` rightly
declines to touch it. A sharper map opens up infield regions the smeared map had
walled off, and the loop tracer follows them.

Porto gaining 1.6 s does not buy the right to break the only unseen-track proxy
we have three weeks before the finals. **Enabling it requires fixing the
centerline extractor's loop tracing first**, and whatever builds the map and
whatever localizes against it must always agree — `lidar_offset` on the
localization node, `PIT_CREW_LIDAR_OFFSET` for the mapper, both default 0.

Worth noting from the same measurement: the scene's front-to-rear axle distance
is **0.3312 m**, against the 0.324 in `vehicle.py`. 2.2%, it feeds the steering
model, and the entire shipping config is validated against 0.324. Recorded here
to be revisited deliberately rather than changed in passing.

## SHIPPING (Aug 23): race time 115.0 -> 101.2 s, Porto best lap 9.81 s

Final configuration: safe centerline, **ay 3.5 / ax 6.5 / brake 4.0 / jerk 20**,
MPC with `ay_limit` 4.5.

| | Porto | ICRA (never tuned on) |
|---|---|---|
| before today | 22 laps, 0 coll, best 11.32 s, **115.0 s** race | 13 laps, 0 coll, 20.9 s/lap |
| **now** | **27 laps, 0 coll, best 9.81 s, median 10.12 s, 101.2 s race** | **16 laps, 0 coll, median 17.95 s** |

**Sub-10-second lap for the first time, and a 13.8 s gain over a ten-lap race,
with zero collisions on both tracks.**

The grip cap was swept against race time once the steering cap made higher
values survivable at all:

| plan ay | median | collisions | 10-lap race |
|---|---|---|---|
| 3.0 | 10.75 s | 0 | 107.5 s |
| **3.5** | **10.12 s** | **0** | **101.2 s** |
| 4.0 | 9.80 s | 2 | 105.4 s |

ay 4.0 holds the quickest single lap of the sweep (**9.44 s**) and loses the race
by four seconds to two collisions. That is this whole project in one row.

## How it was found: 115.0 -> 107.5 s, from two measurements

**The step that unlocked everything above: ay 3.0, ax 6.5, brake 4.0, jerk 20,
MPC with `ay_limit` 4.5 — 25 laps, 0 collisions, best 10.40 s, median 10.75 s,
steering at full lock 0.0% of samples.**

Two independent findings, neither of which is a tuning parameter.

**1. Longitudinal grip was capped at 42% of its measured value.** The tyre asset
gives longitudinal mu 0.72, i.e. ax up to 7.1 m/s2. Every run in this project's
history had `--ax` set equal to `--ay`, which is 3.0. Acceleration happens on
straights where there is no lateral demand at all, so it is very nearly free:

| ay | ax | brake | v_max | ideal |
|---|---|---|---|---|
| 3.0 | 3.0 | 4.0 | 5.39 m/s | 11.0 s |
| 3.0 | 6.5 | 6.5 | 7.00 m/s | 10.1 s |

Driven on its own it was *not* free — 4 collisions — but the reason was not the
friction circle, because holding brake at the proven 4.0 gave the same 4.

**2. The controller had no idea what lateral acceleration its steering implied.**
Localizing those 4 collisions: all between s = 29.2 and s = 2.2 m, two of them on
a section with kappa = -0.00 and the car **0.054 m from its own line**. The
tracking was fine. At the moment of contact the controller had commanded
**0.202 rad at 5.45 m/s**, which through the measured understeer curve is a
demand for **~15 m/s2** against the ~4.5 the tyres deliver. The command was
simply unpayable, and no amount of clearance or tracking accuracy would have
saved it.

Steering authority is not a constant. It is `delta_max = atan(L * ay_max / v^2)`
— whatever the tyres can pay for at the current speed:

| speed | max steer allowed | what the ay-3.0 plan asks |
|---|---|---|
| 2.1 m/s (hairpin) | 0.319 rad | 0.319 rad |
| 3.0 m/s | 0.161 rad | 0.107 rad |
| 5.4 m/s (straight) | 0.050 rad | ~0 |

So it leaves ~50% headroom against the plan everywhere and binds only on gross
over-commands. This is the cheap half of what ForzaETH's MAP controller does
structurally — derive steering from the lateral acceleration you can afford
rather than from geometry alone (arxiv 2209.04346, 0.055 m tracking error at up
to 11 m/s, and the stack won ICRA 2023).

**Same line, same profile, only the limit added:**

| | laps | collisions | best | median | full lock | 10-lap race |
|---|---|---|---|---|---|---|
| shipping ay3.0 / ax3.0 | 22 | 0 | 11.32 s | ~11.5 s | ~5% | **115.0 s** |
| ax 6.5, no limit | 24 | 4 | 10.53 s | 11.02 s | 15.6% | 126.9 s |
| **ax 6.5 + ay_limit 4.5** | **25** | **0** | **10.40 s** | **10.75 s** | **0.0%** | **107.5 s** |

Full lock went from 15.6% of samples to **zero**. That is the mechanism behind
every collision this project has ever logged: the controller demanding lateral
acceleration that does not exist, sliding wide, and answering with more lock.

**Why the earlier understeer attempt failed and this one works.** Both use the
same measured curve. Putting it in the rollout cost told the controller the
tyres give less, and its only lever being steering, it asked for *more* lock —
0.88 collisions/lap. Putting it in the command limit removes the option
entirely. Same measurement, opposite outcome, decided by which component holds
it.

## The MPC review: every claim true, the headline prescription 9x worse (Aug 23)

An external review of `mpc.py` produced five specific defects. **All five are
factually correct** — verified in the code before touching anything:

1. Candidates spread over `steer_rate * control_dt * 4.0` while only
   `steer_rate * control_dt` is ever applied, so with 9 candidates the spacing
   equals one rate-limit step and four positive candidates clip to the same
   applied +0.064 rad. Searched action != applied action.
2. Future steering in the rollout is hard-coded `delta += 0.5 * (ff - delta)`,
   which ignores the 3.2 rad/s limit — it is not optimising a sequence.
3. Prediction and rollout use the *commanded* profile speed, not measured; the
   encoders are validated and were already feeding localization.
4. `on_pose` discards `msg.header.stamp` in favour of arrival time.
5. Reference index is a global `argmin` that can jump across the track.

**Implemented all five. Result: 27 collisions over 23 laps, against 3 over 22
for the same line with the original controller.** Best lap 11.16 s vs 10.45,
full lock 24.0% vs 17.7%.

**Two things the diagnosis got wrong, and they matter more than the five that
were right.**

*The timestamp fix is a no-op.* Measured `now - scan_stamp` over 40 scans:
**min 0.000, median 0.001, max 0.001 s**. The bridge stamps scans on arrival, so
there is no scan-to-now latency to recover. The defect is real as written and
worth nothing.

*The candidate range is not a bug — it is the design.* The wide range selects a
steering **target**; the rate limiter below it is the actuator model. Narrowing
it to exactly one applicable step leaves the rollout unable to evaluate
committing to a larger angle at all, and the cost differences between
neighbouring candidates collapse into noise. That is the change most likely to
have caused the regression, and the telling detail is that the wiggle metric did
*not* move (87.7 steering zero-crossings per minute, against ~88/min measured
before). The failure was not chatter; it was the controller no longer being able
to see where it needed to steer to.

**Reverted to the proven defaults; every one of the five is now a parameter**
(`reach_steps`, `use_measured_speed`, `rate_limited_rollout`, `windowed_index`;
the timestamp fix is kept, being both harmless and marginally more correct), so
each can be isolated later rather than tested as a bundle again. Baseline
re-verified after reverting: **19 laps, 0 collisions, best 11.09 s.**

Honest gap in this result: the five were implemented and reverted *together*, so
which one caused the regression is inferred, not measured. The flags exist to
settle that.

What stands from the review regardless: `tools/steering_smoothness.py` counting
sign crossings does conflate genuine track transitions with control oscillation;
the controller has no speed authority, so it can only answer insufficient grip
with more lock; and the LiDAR-to-rear-axle offset (0.2733 m) appears to be
omitted in both mapping and localization, which may partly self-cancel but is
worth verifying.

## Acceleration limits in the shaping objective

The point-mass `v = sqrt(ay/kappa)` assumed the car arrives at and leaves every
corner at cornering speed, so it priced radius as free and straights as
worthless. Replacing it with two forward/backward sweeps under the real ax
limit (`profile_time()` in `tools/open_corners.py`):

| line | clearance | kappa_max | v_max | ideal |
|---|---|---|---|---|
| safe, ay 3.5 | 0.325 m | 1.019 | 5.82 m/s | **10.20 s** |
| opened, point-mass objective | 0.269 m | 0.640 | 4.36 m/s | 10.51 s |
| **opened, acceleration-limited** | **0.300 m** | 0.642 | 4.26 m/s | **10.28 s** |

Better on every axis than the point-mass version — clearance back up to 0.300 m
and the ideal lap now a tie with the safe line — while asking for kappa 0.64
instead of 1.019. Top speed is still down, and raising the Fourier resolution
does not fix it: L-BFGS-B converges worse as modes rise (123 iterations at 40
modes, 63 at 90), so those lines come out *slower*, 12.4-12.9 s.

## Opening the corners: three objectives, and only the third is the right one (Aug 23)

The corridor at Porto's tightest corner is **2.075 m wide** — I had assumed it
was the 0.875 m track minimum, and it is not. The car is 0.270 m wide, so there
is +-0.90 m of lateral freedom there, +-0.73 m even after the measured lateral
localization error. **The room to open that corner has been there all along; our
optimisers were not finding it.** `tools/open_corners.py` exists to find it.

Three objectives, tried in order, on the same corridor:

| objective | R_min | length | top speed | verdict |
|---|---|---|---|---|
| centerline (baseline) | 0.951 m | 29.82 m | 5.39 m/s | kappa_max 1.051 |
| minimum *integral* curvature (`optimize_raceline`) | 0.58 m | 27.0 m | — | peak gets **worse**: 1.72 |
| curvature-weighted Laplacian relaxation | 0.727 m | 28.20 m | — | **worse**, every setting |
| minimum *peak* curvature (minimax) | **1.772 m** | 31.95 m | 2.97 m/s | peak beautiful, **14.0 s lap** |
| **minimum point-mass lap time, curvature-capped** | **1.560 m** | 29.62 m | 4.36 m/s | **the right objective** |

Each failure taught the next one:

- **Min-integral curvature** spikes locally, because the integral does not care
  where the curvature goes. Peak rises from 1.02 to 1.72.
- **Laplacian relaxation** minimises *length*, not curvature. It cuts corners,
  which tightens exactly the peak it was meant to open: -24% to -61% of minimum
  radius across every setting tried. Kept in the tool only because the failure
  is instructive.
- **Minimax curvature** flattens the peak superbly — R_min 0.951 -> 1.772 m — and
  then drives a 14.0 s lap. Nothing in that objective wants a straight to stay
  straight, so it spends curvature everywhere to pull the maximum down and top
  speed collapses from 5.39 to 2.97 m/s. **Minimising peak curvature is not
  minimising lap time.**
- **Point-mass lap time** balances them, but on its own it plans corners the car
  cannot physically turn: R_min 0.305 m against a 0.561 m steering lock, shape
  score 6.91 s. That missing constraint is the planner-side curvature cap. With
  `--kappa-cap 0.64` (all the car achieves at racing speed even at full lock) it
  produces a usable line.

**The resulting line, `track_v2_opened`:** R_min 1.560 m (**+64%**), path length
essentially unchanged at 29.62 m, worst-corner clearance 0.269 m (still clear of
the 0.22 m gate), and the number that matters — **required steering at the
tightest point falls from 0.319 to 0.204 rad**. The measured grip curve puts
understeer at 0.58 of asked at 0.319 rad and 0.79 at 0.204. The line asks the
front tyres for something much closer to what they will actually give.

Ideal lap is *not* better at matched grip (11.08 s vs 11.01 s at ay 3.0): the
shaping trades some top speed for radius. The bet was that it could hold a
higher ay cleanly, where the safe line cannot.

**Driven at ay 4.0, the bet half paid.** 22 laps, **2 collisions**, best 10.80 s
— against the *same cap* on the safe line, which gave 10 collisions over 20
laps. **Collisions per lap fell 5x, 0.50 to 0.09**, which is the mechanism
working exactly as the grip curve predicted: ask the front tyres for 0.204 rad
instead of 0.319 and they deliver it. But median lap went the wrong way, 10.48
to 11.54 s, because the shaping gave up top speed (5.39 -> 4.63 m/s) to buy the
radius.

**Priced as race time — which is the only number that counts — nothing beats the
shipping config:**

| config | median | collisions/lap | 10-lap race time |
|---|---|---|---|
| **safe ay 3.0 (shipping)** | 11.50 s | **0.00** | **115.0 s** |
| safe ay 3.5 | 11.01 s | 0.14 | 123.7 s |
| safe ay 4.0 | 10.48 s | 0.50 | 154.8 s |
| opened ay 4.0 | 11.54 s | 0.09 | 124.5 s |

At +10 s a collision, a config has to be *both* faster and clean. Best lap is a
vanity metric here: the ay-4.0 safe line holds the fastest lap of the session
(9.96 s) and the worst race by 40 seconds.

**The specific flaw to fix next.** The shaping objective uses a point-mass speed
`v = min(vmax, sqrt(ay/kappa))`, which assumes the car arrives at every corner
already at its limit speed and leaves it the same way. It therefore overvalues
corner radius and undervalues straights — precisely the trade that cost 0.76 m/s
of top speed. Putting the real forward/backward acceleration-limited profile
inside the objective (the same one `optimize_raceline.py` already computes)
should keep the followability win while giving the straights back.

## Raising the grip cap on the safe line: 3.5 is faster and still not clean

| profile | laps | collisions | best | median | full lock |
|---|---|---|---|---|---|
| ay 3.0 (shipping) | 22 | **0** | 11.32 s | ~11.5 s | ~5% |
| ay 3.5 + jerk 20 | 22 | 3 | **10.45 s** | 11.01 s | 17.7% |
| ay 4.0 + jerk 20 | 20 | 10 | 9.96 s | 10.48 s | 16.1% |

ay 3.5 saves ~0.9 s/lap and costs 3 collisions, which is +30 s against ~9 s
saved over ten laps. **Not shippable.** The pattern is monotone and the cause is
the same one the grip curve identified: the safe line's 0.319 rad demand sits
where the tyres return 58% of what is asked.

## The clearance frontier is a dead end on Porto, and understeer compensation backfired (Aug 23)

Two experiments, both aimed at the ~2 s that separates our clean 11.32 s from
the ay-4.0 profile's 9.56 s ideal.

**1. Swept the corridor-margin x grip-cap grid** (`tools/sweep_lines.py`, gated
on the re-derived 0.22 m clearance floor). The ranking added two metrics the
earlier sweeps lacked: how much of the lap demands steering past the measured
saturation point, and how much steering lock is left in reserve at the tightest
corner. The top two candidates settle the geometry question:

| candidate | ideal | clearance | kappa_max | steer reserve |
|---|---|---|---|---|
| optimized, w_veh 0.32, ay 4.5 | 9.02 s | 0.225 m | 1.51 | **0.070 rad** |
| **safe geometry**, ay 4.5 | 9.07 s | **0.325 m** | **1.02** | **0.205 rad** |

**Fifty milliseconds apart.** The optimized line pays for them with 44% of its
clearance and two thirds of the authority the controller has to correct an error
exactly where it can least afford one. Every other optimized candidate fails the
clearance gate outright. So on Porto the geometry is settled — the safe
centerline is the line — and *all* remaining lap time is in the velocity
profile. This also explains why `--w-veh` looked flat before: it is flat.

**2. Put the measured understeer curve into the controller's model — and it made
everything worse.** `vehicle.achieved_curvature()` encodes the measured ratio
(1.00 below 0.10 rad, 0.42 at full lock), and `mpc.py` can roll out with it
instead of `tan(delta)/L`. Same line file as the morning's run, same caps, only
the controller's internal model changed:

| | laps | collisions | per lap | full lock | best | median |
|---|---|---|---|---|---|---|
| model **off** | 20 | 10 | 0.50 | 16.1% | 9.96 s | 10.48 s |
| model **on** | 8 | 7 | **0.88** | **36.0%** | 10.03 s | 10.82 s |

Worse on every axis: 76% more collisions per lap and 2.2x the time at full lock.

Caveat on the sample: the model-on run was stopped at 87 s by a stray background
timer, so it is 8 laps against the baseline's 20. The full-lock figure is a
per-sample statistic over thousands of samples and is unaffected by the shorter
run; the collisions-per-lap comparison rests on 7 events in 8 laps against 10 in
20, which is directional rather than tight. The effect is large and the mechanism
below explains it, so this was not re-run — but if the planner-side version ever
needs a head-to-head, use matched run lengths.

**The mechanism, in hindsight obvious.** Telling the controller the tyres deliver
less curvature than it thought does not make it slow down — it has no authority
over speed, because the velocity profile is fixed offline. Its only remaining
lever is to command *more lock*, which drives the front tyres further into the
saturation the model just described. Honesty about the tyre, handed to the wrong
component, amplifies the exact failure it was meant to prevent.

**The model itself is validated and kept** — the measurement is sound and
`achieved_curvature()` stays in `vehicle.py`. It belongs in the **planner**: cap
planned curvature at what the car can actually achieve and set the speed from
that, so the controller is never asked for turn-in that does not exist. The
parameter defaults to off; the shipping config is untouched.

## Submission integrity pass (Aug 23) — two of these would have scored zero

An architecture review proposed replacing the planner, controller and estimator.
Auditing the submission path first turned up defects that outrank every one of
those proposals, because they are certain rather than speculative.

**1. The image did not start itself.** The base devkit is
`ENTRYPOINT ["/bin/bash"]` with no `CMD`, and ours re-declared the same. `docker
run <image>` therefore landed in a shell and started nothing. Fixed by adding
`CMD ["-c", "/home/autodrive_devkit.sh"]`; passing `-c "..."` still overrides
it, so every existing script behaves identically. **Verified by cold-starting
the built image with no arguments** and watching `bringup_headless`,
`autodrive_bridge` and `localization` come up.

**2. We were breaking the devkit's NumPy ABI.** The base image ships numpy
1.22.2 and its ROS 2 Humble C-extensions are compiled against the 1.x ABI. Our
`pip3 install scipy` resolved numpy to **2.2.6**, after which the devkit's own
bridge printed an ABI warning on startup and `cv_bridge` failed outright with
`AttributeError: _ARRAY_API not found`. We are forbidden from modifying
`autodrive_roboracer` to repair that, so we must not break it. Pinned to
`"numpy<2" "scipy==1.10.1"`; verified numpy stays 1.22.2 and `cv_bridge`,
`sensor_msgs` and `pit_crew` all import cleanly.

**3. No stale-pose failsafe.** Commands latch in this simulator. If localization
died the controller kept publishing its last command forever — full throttle
into a wall. `mpc.py` now coasts to zero throttle after `stale_pose_s` (0.35 s,
about 3.5 missed scans) while *holding* the steering angle: snapping to zero
mid-corner aims at the outside wall.

**4. The teleport signal was set and never read.** `SpeedEstimator.jumped` was
raised on a physically impossible encoder derivative — which is exactly what a
checkpoint reset produces — and nothing consumed it. Now `consume_jump()` clears
the invalidated speed and sample history, and the filter stands still and
spreads for that step instead of marching the whole particle cloud away from
where the car now is.

**5. No `.dockerignore`.** The build context shipped `.venv/` and `upstream/` —
about 1.8 GB — to the daemon on every build.

Regression test, shipping config on Porto: **9 laps, 0 collisions, best 11.26 s**
against the 11.32 s baseline, with **zero** spurious failsafe or jump triggers.

## The clearance gate used the wrong error component (Aug 23)

`MIN_CLEARANCE` was 0.30 m, justified by localization's total Euclidean p95 of
0.298 m. That is the wrong statistic: only the error *normal* to the path pushes
the car towards a wall. Decomposing the same trace against the direction of
travel:

| component | median | p95 | max |
|---|---|---|---|
| total euclidean | 0.092 | 0.298 | 0.427 |
| along-track | 0.074 | 0.246 | 0.410 |
| **normal (lateral)** | **0.043** | **0.171** | **0.196** |

The headline figure is mostly along-track **lag** — it makes the car early or
late along the path, not closer to the wall. The gate was therefore about twice
as conservative as the geometry requires.

Re-derived to **0.22 m** = worst observed lateral error (0.196) + the map's own
0.025 m resolution, i.e. it now rejects what is physically impossible rather
than what is merely untested. **This changes no decision on either known
track**: Porto's optimized lines (0.146, 0.135) are still rejected and the safe
line still wins, ICRA's 0.564 still passes. It only matters for a future track
whose best line lands between 0.22 and 0.325 m — and such a line is written to
`race.csv`, which nothing races until it has been driven.

The 0.146 m (~57 collisions) to 0.325 m (22 clean laps) frontier remains
untested in between. That is the next experiment, not an assumption.

## Are we overfitting to Porto? Audit (Aug 22)

Worth asking, because the finals track is released three days before the
deadline and nothing can be hand-tuned on it.

**Regenerated per track, so not tuned at all:** map, centerline, safe line,
raceline, corridor widths, velocity profile, start pose. `new_track_drill.sh`
produces all of it in ~271 s measured.

**Vehicle-level, therefore track-independent by construction:** wheelbase, body
box, steering lock and its 3.2 rad/s rate, wheel radius, the 0.040 throttle
gain — and now the grip curve. Checked rather than assumed: every collider in
the scene, including all six track meshes, carries PhysicsMaterial (0,0), i.e.
the default, and Unity WheelColliders take friction from their own curves
regardless of the surface. **Every track in this simulator has identical grip**,
so the 4.5 m/s2 plateau and the 0.15 rad saturation point transfer to a track we
have never seen.

**Genuinely tuned on Porto:** the ay/ax caps, `--w-veh 0.16`, the jerk cap, the
MPC weights and horizon, and the localization parameters (`sigma_hit`,
`beam_temp`, motion noise, `throttle_to_mps`).

**The evidence that this is not badly overfit is ICRA.** A 56.7 m circuit, twice
Porto's length, never tuned on, ran **13 consecutive laps, 0 collisions, at full
speed** with *zero* parameter changes. We have also already caught and fixed one
genuine overfit: `throttle_to_mps` had been set to 0 because that helped Porto,
and it made the filter diverge on ICRA (mean 1.48 m); it went back to 25.0.

**Two places the Porto shape is still baked in:**

1. **`make_track_lines.py` always ships the safe centerline.** That choice came
   from a Porto-only result — the corridor there is too narrow for the optimized
   line to keep enough clearance. On a wider finals track the optimized line
   could well win. The script already picks the centerline *method* by
   measurement; safe-vs-optimized should be chosen the same way instead of by a
   constant.
2. **"`--kappa-bound` does nothing"** is explicitly a Porto-corridor
   observation, kept as such. On a wider track it may bind.

And the honest caveat: n = 2 tracks.

## Step 2: VAUL's winning stack is not published (Aug 22)

Cloned everything public under `github.com/vaul-ulaval` (kept in `upstream/vaul/`,
git-ignored). **The autonomy stack that won is not among it**, so "get VAUL's
stack running and diff it" cannot be done as written.

- `autodrive_roboracer_ws` is their *development environment* — Docker compose,
  devcontainer, bridge. Its competition entrypoint `devkit-final.bash` contains
  the literal line `# Autonomy stack goes here`, and what it actually launches
  is `wall_follow_node`, a reactive wall follower. The README dates it to the
  2nd league (CDC 2024).
- `global_racetrajectory_optimization` is a fork of TUM's, and the committed
  `params/racecar.ini` is **stock TUM** — mass 1200 kg, length 4.7 m,
  v_max 70 m/s. Nothing scaled to a 1:10 car.
- The rest (`particle_filter`, `kiss-icp`, `norlab_controllers`, `range_libc`)
  are forks of third-party projects: the components, not the integration or the
  tuning.

**What the code does still tell us**, all of it narrowing the gap rather than
widening it:

| Their code | Value | Ours | Verdict |
|---|---|---|---|
| `wheel_odometry_node.cpp` `counts_per_rev` | **6.5** (i.e. 2*pi) | radians, scale 0.993/0.999 | same interpretation — **closes** the counts-vs-radians question, and our calibration is tighter than their 6.5 vs 6.283 |
| same, `max_acceleration` outlier reject | 4.0 m/s2 | none | worth adding; note it matches our measured grip plateau |
| `ackermann_to_autodrive_node.cpp` throttle | `speed / 22.8` = **0.0439 per m/s**, open loop | 0.040 per m/s, open loop | match — no speed PID on their side either |
| `spline_bruteforcer.py` (their own file) | `width_veh=0.32` | 0.27 car + swept footprint + 0.16 margin + 7.5 cm/side wall dilation | **they plan a far tighter corridor than we dare** |
| same | brute-forces `stepsize_prep`/`stepsize_reg` over 0.05-1.5 m until spline normals stop crossing | fixed 0.10 m stepsize | a 1:10-scale adaptation of the TUM defaults (1.0-3.0 m) |

**The one number that stands out is 0.32 m.** Their corridor allows the car
0.32 m of width — 2.5 cm of margin per side over the 0.270 m body. We plan with
the swept footprint plus 0.16 m and race a line with 0.325 m of measured corner
clearance, because our error budget needs it: live localization is 0.092 m
median but 0.298 m p95. A team running `particle_filter` with `range_libc` ray
casting *and* kiss-icp can afford margins we cannot.

That is the same conclusion the grip curve reached from the other end. Staying
inside the linear tyre region needs curvature under 0.46 1/m; Porto's tightest
corner is 1.019; opening it requires track width; track width is exactly what
our localization error is spending. **The lever is localization accuracy, which
buys clearance, which buys corner radius, which buys speed.** Not the ay cap,
and not the tracker.

## What the winners do differently (VAUL, 1st at ICRA 2025 and CDC-TF 2025)

VAUL (Université Laval) publish their tooling at `github.com/vaul-ulaval`.
Comparing their component list against ours:

| Component | VAUL | Ours | Verdict |
|---|---|---|---|
| Raceline optimization | `global_racetrajectory_optimization` (TUM) | same TUM `tph` | **match** |
| Localization | `particle_filter` (fork of f1tenth/MIT, ray casting via `range_libc`) | own MCL, likelihood field | equivalent approach |
| Reactive baseline | `wall_follow_node` | Follow-the-Gap | **match** |
| Odometry | `wheel_odometry_node` **and `kiss-icp` (LiDAR odometry)** | wheel encoders (LiDAR odometry built, measured, rejected) | narrower than it looked |
| Control | `norlab_controllers` | Stanley / MPC | comparable |

**Written Aug 7, when this section concluded "the gap is odometry, and KISS-ICP
is the missing piece". Both halves turned out to be wrong, and the correction
is worth keeping.** The premise was that our encoders were unusable, so the
particle filter ran on a throttle-derived guess. The encoders are fine — the
"unusable" measurement was taken while the car was stuck and reversing (`position`
is cumulative wheel angle in radians; scale 0.993/0.999 verified on two tracks).
They are now the filter's motion source, and live localization sits at 0.092 m
median.

KISS-ICP was then the wrong prescription for a second, independent reason: we
built scan-to-scan LiDAR odometry and it **cannot work on a racetrack** — the
aperture problem, measured 5.7x underestimate. See the next section. VAUL
presumably carry it for robustness rather than because scan matching supplies
the forward speed; the wheels do that in their stack too.

So the odometry column is not the gap. What remains unexplained is pace, and
the current plan attacks it by diffing their *raceline* against ours.

Also noted at the time: VAUL's `wheel_odometry_node` treats the encoder
`position` field as **counts**, dividing by counts-per-revolution, where we
treated it as radians. **Closed** — radians is correct for this sim, verified
by the 0.993/0.999 scale against IPS on two tracks. The near-zero correlation
that prompted the question was junk data, not a unit error.

Sources: [vaul-ulaval](https://github.com/vaul-ulaval),
[KISS-ICP](https://github.com/PRBonn/kiss-icp),
[f1tenth/particle_filter](https://github.com/f1tenth/particle_filter),
[f1tenth_benchmarks](https://github.com/BDEvan5/f1tenth_benchmarks)

## LiDAR odometry: implemented, measured, not adopted

`src/pit_crew/scan_odometry.py` — translation-only 2D ICP between consecutive
scans, with yaw taken from the IMU so only the translation is unknown. Runs
inside the 10 Hz budget.

**It does not work on a racetrack, for a geometric reason.** Measured against
IPS: median 0.16 m/s against a true 0.87 m/s — a 5.7x underestimate, correlation
0.225. Through the filter: median 2.74 m versus 0.199 m with encoders.

The cause is the **aperture problem**. A racetrack is long, smooth walls running
parallel to the direction of travel; sliding along such a wall barely changes
the scan, so forward translation is close to unobservable from wall geometry
alone. This is exactly why the winning team runs wheel odometry *and* KISS-ICP
rather than scan matching alone — the wheels supply the forward speed that scan
matching cannot see.

The module is kept: fused with encoder speed it would still be the right tool
for correcting lateral drift and slip, which is where scan geometry *is*
informative.

**The real result of chasing this was discovering the encoders work.** That was
the missing odometry all along; it is now the filter's motion source.

## Remaining gap on unseen tracks — SUPERSEDED (was wrong)

This section argued that ICRA failed on an **error budget**: localization p95
0.549 m against 0.215 m of corner clearance, so the fix was to trade lap time
for margin in the line generator.

**That diagnosis was wrong and the section is kept only as a record of it.**
ICRA was failing on a centerline **spur** — a stretch where the path doubled
back on itself, so the car was commanded to stop and reverse, and wedged there
every lap. Clearance was never the binding constraint; the line already
measured a healthy 0.414 m. See "Unseen track (ICRA): SOLVED" below for the
evidence and the three fixes. Result after them: **13 laps, 0 collisions, full
speed**, and clearance improved to 0.564 m as a side effect.

The general lesson: an error-budget argument is easy to construct from two
numbers that are both real, and it will still be wrong if the failure has a
different mechanism. Localize the collisions in space before theorising.

## Closing on the winners: what is actually left (Aug 7-8)

We track our own plan to within **2.4%** (11.32 s run vs 11.05 s ideal), so the
controller is no longer the bottleneck. Everything left is in the plan. Four
levers swept, all measured on Porto:

| Change | Ideal | Best lap | Collisions |
|---|---|---|---|
| safe line, ay 3.0 (**shipping**) | 11.05 s | **11.32 s** | **0** over 22 laps |
| safe line, ay 4.0 | 9.57 s | 10.11 s | 23 over 21 laps |
| optimized raceline, ay 3.0 | 10.41 s | 10.65 s | 57 over ~25 laps |
| corridor-margin sweep (`--w-veh`) | 10.45 s best | not driven | — |

**The optimized raceline is not usable, and not for want of tracking.** Its
failures spread over 6 separate zones, the signature of a clearance problem
(0.146 m) rather than a defect. MPC's ~7 mm tracking did not rescue what
Stanley failed at, which retires the idea that the old rejection was the
controller's fault.

**ay 4.0 fails at exactly two corners, and they are not the tightest ones.**
kappa 0.379 and 0.699 against a maximum of 1.019 — both are *curvature
transitions* (zone A reverses from -0.611 to +0.379 within 1.5 m while
accelerating 2.38 -> 3.25 m/s). Steering sits at full lock **20% of the time**
versus ~5% in the clean run. So the ay-4 limit is transient tracking through
direction changes, not steady-state grip. ay 5.0 was not driven: with the
steering already saturating a fifth of the time, more speed can only be worse.

**Porto's corridor is too narrow for raceline optimization to buy much.** The
`--w-veh` sweep trades clearance for time along a nearly flat frontier: 0.146 m
-> 10.41 s against 0.292 m -> 10.67 s. `--kappa-bound` changes nothing at all
(0.8/1.0/1.2/1.6 all give the same line) because the corridor binds first. Note
the "optimized" line's **peak** curvature (1.717) is *worse* than the plain
centerline's (1.019) — min-curvature reduces the integral, and spikes locally,
which is precisely what saturates the steering.

**Where this leaves the 6.53 s target.** Our realistic ceiling with these levers
is ~10.5-11.3 s. Winners average 4.38 m/s over this track; the tightest corner
on our line is R = 0.98 m, which at that pace implies lateral accelerations
around 9-20 m/s2. Our car visibly slides around 5-7. Either their line opens
those corners using width we are not finding, or their vehicle/sim differs.
**Next step is to read their raceline generation, not to sweep our own
parameters further** — this frontier has been mapped and it does not reach.
Before spending on that, confirm the 6.53 s figure really is the same track,
same vehicle, same sim version; the implied grip is high enough to be worth
checking.

Tooling note: `src/pit_crew/skidpad.py` measures grip directly (hold steering,
ramp speed, watch achieved yaw rate fall away from v*tan(delta)/L). It works,
but **needs open space neither track has** — on Porto the slide widens the
circle until the car grinds the walls (354 collisions in 100 s), corrupting the
result. See the module docstring.


## Unseen track (ICRA): SOLVED — 13 clean laps at full speed (Aug 7)

**Result: 13 consecutive laps, 0 collisions, speed_scale 1.0, 20.9 s/lap**
(spread 20.37-21.53 s) with MPC on the 56.7 m ICRA circuit. The same setting
previously produced 134 collisions.

It was never speed, and never the error budget this section used to claim.
**The centerline had a skeleton spur**: at one point it ran into open track,
dead-ended, and doubled back on itself, the two legs sitting 0.11-0.17 m apart
for 1.6 m. The car was being commanded to drive somewhere, stop, and reverse
direction. It wedged there every lap and ground against the wall for 20-100 s.
That single point produced *every* collision — 83 of 83 in one run, 47 of 48 in
the next, all inside a 2.5 m circle at (-5.2,-2.9).

**Why it hid for so long, and the lesson**: the line measured a perfectly
healthy 0.414 m worst-corner clearance. Every geometry check we had looks at
clearance, and clearance cannot see a path that folds back on itself. The
misleading evidence was the speed sweep — collisions fell 8x when speed halved,
which looked like a pace problem, so "reduce speed" became the hypothesis. It
was a *dwell* effect: slower means fewer wedging attempts per minute, not a
more drivable line. The tell that should have caught it earlier: scale 0.3 was
*worse* than 0.5 (83 vs 17). No grip or actuator limit behaves that way. **When
lowering speed makes things worse, stop tuning speed.**

Three fixes, all track-agnostic:

1. `tools/extract_centerline.py: excise_spurs()` — removes there-and-back
   excursions from the ordered loop. It tells a spur from a real hairpin by leg
   separation, so it cannot eat a genuine corner. ICRA clearance 0.414 ->
   **0.564 m**; Porto rebuilt bit-identical at 0.325 m (no-op where there are
   no spurs).
2. `tools/make_track_lines.py: drivability()` — hard-rejects any line with a
   direction reversal or a radius tighter than the 0.561 m steering lock,
   *before* clearance is considered. A line can be roomy and still undrivable.
3. `src/pit_crew/mpc.py` — two independent controller defects found on the way:
   - **The horizon was measured in time, not distance.** 12 x 0.06 s previews
     1.1 m at racing speed but only 0.36 m at 0.5 m/s, less than a corner
     radius, so the controller could not see the corner at all. Each step now
     advances at least 0.10 m of arc; at Porto speeds `step_dt` is still the
     binding term, so that verified config is untouched.
   - **No stuck detection.** `self.speed` is the *commanded* profile speed, so
     it reads 1.5 m/s while the car sits motionless against a wall — the
     controller had no signal anything was wrong. Now detects it from actual
     pose travel and reverses out, the same escape FTG already uses. Raised
     ICRA throughput from 3 laps to 11 in the same 5.5 minutes even before the
     spur was fixed.

Note the sim's `lap_count` and `collision_count` are **cumulative across runs**
and only reset when the simulator container restarts. Two runs here were first
misread because of that; always difference them against the run's first row.

## September 12: the lateral limit, identified instead of guessed

`tools/lateral_id.py` (new, offline) fits the car's lateral response from the
Sept 11 race logs, ForzaETH On-Track-SysID style, and writes
`data/calibration/steer_map.csv` and `data/calibration/ggv.csv`. Nothing in the
shipping stack reads them yet — `optimize_raceline.py`, `mpc.py` and
`vehicle.py` are untouched.

Measurement chain: IPS x,y at ~11 Hz, local cubic fit over ±2 samples for
speed, path curvature and ay; `steering` in the same row is actuator feedback
from the same telemetry packet, so the 0.135 s command dead time is not in this
signal. Cross-checked against the IMU gyro in `data/bags/exp_ay50_d180_181558`:
the IPS-derived path yaw rate is **0.954x** the measured body yaw rate, so
every number below reads ~5% LOW at the peaks.

Identified (fit on six high-ay Porto runs, ICRA held out):

| quantity | value | how well determined |
|---|---|---|
| kinematic gain `g` | 1.000 | bounded; kappa and ay are 0.94 correlated, a free fit returns a 0.17 m wheelbase |
| understeer gradient `K_us` | 0.0011 rad/(m/s²) | weak; characteristic speed 17 m/s, i.e. nearly neutral |
| lateral pole `tau` | 0.085 s (bag: 0.045 s) | trades against a 0.050 s dead time at a 0.09 s sample rate; either way it is fast |
| saturation, hard clip | 5.94 m/s² (ICRA 7.00) | fits |
| saturation, tanh asymptote | 10.4 m/s² (ICRA 11.4) | **not identified** — 9 to 15 all fit within 3% |

Forward validation (model ay vs measured ay per speed×steer cell, quasi-steady
only) is within ±0.65 m/s², usually under 10%.

**Does the data support ay 8-9 in the hairpins? No — and it cannot rule it
out.** Nothing in the corpus holds more than **6.5 m/s²** for 0.27 s at any
speed (sustained p99 by speed band: 4.67 at 1.75 m/s, 5.44 at 2.25, 6.19 at
2.75, 6.54 at 3.25). In the hairpins (v 2.5-3.2, |delta| > 0.26) the car asks
0.901 1/m and gets 0.745, at a median ay of 5.70. The marginal return on lock
is already down to 50-60% of kinematic by ay 5.4. A kappa 1.0 apex is drivable
at **2.56 m/s** on the measured envelope, not the 2.9-3.0 m/s that ay 8.7 would
buy. But the corpus contains **4 quasi-steady samples above 0.33 rad of
steering** and **4 samples above 4.5 m/s with more than 0.15 rad** — the
controller never asks, so racing can never answer.

Independent corroboration of the promoted config: the measured sustained
envelope (6.5) sits exactly where the driven results put the cliff — planner
ay 6.0 runs 100 laps clean, 6.5 takes 3/100, 7.0 takes 5/32.

Also measured on the way: the combined |a| at p99 is **7.0-7.2 m/s² at every
lateral level**, i.e. a friction circle of ~7.1 m radius, and the usable
longitudinal axis shrinks as ay grows (ax p05 goes -5.9 → -0.8 as |ay| goes
0-1.5 → 5.5+). There is almost no hard-braking-while-cornering data (27
samples), so the ellipse coupling is assumed, not measured.

`data/calibration/ggv.csv` is in the TUM format `optimize_raceline.py` already
builds inline (`np.loadtxt(..., comments='#')` then `calc_vel_profile`;
verified it reproduces sqrt(ay/kappa) exactly). `ay_max` is written **flat** at
the measured 6.54: the per-speed rise in the envelope is demand, not grip.
`ax_max = 5.0 - 0.273v` comes from the longitudinal plant `tools/sim_servo.py`
already identified (ServoThrottle's measured defaults) — note the shipping
`--ax 6.5` is above what the servo can deliver.

To settle the 8-9 question, `src/pit_crew/sysid_run.py` (new, registered as
`ros2 run pit_crew sysid_run`) drives a steering staircase on the
**`-explore`** image: speeds 2.0-5.5 m/s, steering 0.05 rad to lock, 2.0 s
settle + 1.5 s hold per cell, sign alternating so the car weaves instead of
spiralling, cells above 14 m/s² of kinematic demand skipped, staircase stopped
when two steps add less than 0.2 m/s² of ay, LiDAR and 18 m radius guards. It
logs every telemetry tick to `/home/data/logs/sysid_<ts>.csv`, and
`tools/lateral_id.py --sysid <csv>` folds those holds into the same fit.
**Not yet driven.**

### Later on September 12: the first staircase run, and why it stopped

`data/logs/sysid_20260911_212223.csv`, 103 rows, 10 s. It measured **one
cell** — 2.0 m/s at 0.05 rad, 15 hold samples — and then tripped the 18 m
radius guard. That cell is a clean validation point and nothing more: achieved
curvature 0.154 against a kinematic 0.1535 (**ratio 1.003**), and the
identified model predicts it to within **1.8%**. At ay 0.60 it says nothing
about the limit, so `tools/lateral_id.py --sysid` on it changes no fitted
number; the calibration files stay the race-log fit.

Why it stopped: **geometry, not grip.** The spin-up held zero steering, so the
car drove 9.4 m straight before the staircase began, and the first two steering
steps are shallow arcs that keep going outwards — 18.2 m from the spawn 10 s
in. The LiDAR guard was never involved: all 103 rows read `lidar_min = inf`, so
in the explore world (no geometry within the 10 m LiDAR range) that guard
cannot fire at all.

Fixes in `src/pit_crew/sysid_run.py`, each simulated end to end against the
identified model before being written:

- **Circular spin-up**, radius `max(2.0, v²/4.5)`, so speed is gained inside a
  bounded disc. A closed circle, not a figure of eight — the eight ends up
  pointing outwards and simulates worse (22.0 m vs 18.6 m).
- **Drive home between cells** past 8 m: steer at the spawn point (bearing
  minus the course angle from the IPS trace — no yaw topic, no gyro
  integration) at 3 m/s until inside 4 m, then resume the pending cell.
  **At most one drive home per cell**: a 2 s settle at 5.5 m/s covers 11 m,
  more than the gap between the two radii, so without the limit the car goes
  home, settles, is outside again, and ping-pongs forever — that hung the
  simulation outright.
- **Radius guard becomes a backstop at 40 m**; the LiDAR guard defaults off
  (`lidar_stop_m: 0.0`), to be raised only for a world that has walls.
- **`max_demand` 14 → 20 m/s²**: at 5.5 m/s a 0.15 rad step already asks 14, so
  the old cap would have stopped the staircase one step before the answer if
  the skidpad's 9.9 m/s² is the truth. The knee rule (two steps adding less
  than 0.2 m/s² of ay) is the real terminator.
- `tools/lateral_id.py --sysid` no longer differentiates the pose for these
  logs — the logger writes a row per IMU message and the IPS repeats between
  them, which dropped 3 of 15 hold rows and then split the rest below the
  minimum segment length, so the loader reported zero. Speed now comes from
  distinct poses with the encoder as fallback, curvature from the gyro, and it
  warns when a staircase log contributes almost nothing.

Simulated on the identified model: **61 cells, ~420 s, 32 drives home, never
more than 17.7 m from the spawn**; on the pessimistic hard-clip grip model,
434 s and 16.2 m. The aborted run reached 18.2 m with no LiDAR return in any
direction, so that much floor is known clear. Still not driven at full length.
