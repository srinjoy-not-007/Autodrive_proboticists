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

All three DQ arms failed at **s 18.0-18.5, 0.35-0.56 m inside the line, at
1.0-1.5 m/s true speed** — crawling, and still steered into the inside wall.
The car is not arriving hot: it is **under** plan by 0.25-0.5 m/s into the entry.

With the brake on, the live entry cut is 0.05-0.07 m **better** than the harness
predicts, so the 0.058 s harness is pessimistic here — useful for ranking, not
for gating.

---

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
3. **`lon_v8b55a65`**, if a live slot remains after 1 and 2 (section 4).

TUM min-time optimisation: **no** — the corridor cannot pay for it.

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
