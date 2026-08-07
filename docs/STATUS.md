# Status — measured, not assumed

## Target: what winning actually looks like

Published results, **same Porto practice track we develop on**:

| Event | Qualification (10 laps) | Best lap | Finals best lap |
|---|---|---|---|
| CDC-TF 2025 | Mamba 65.41 s | **6.53 s** | VAUL 7.74 s |
| ICRA 2025 | AGV 67.86 s | 6.77 s | VAUL 11.28 s |

Qualification uses the **Porto practice track — the same one we develop on**,
so those best-lap figures are directly comparable to ours. The finals figures
are on each event's own unseen track and are not comparable. Podium teams post
**zero collisions**, so cleanliness is table stakes, not an edge. Our best clean lap is 32.43 s — about **5x off the winning pace**. Note
our own velocity profile's "ideal" lap was ~11 s, i.e. slower than their real
6.5 s: the grip assumptions (ax = ay = 3 m/s^2) are far too conservative. A
6.5 s lap on a 28.6 m loop is 4.4 m/s average, which needs roughly 10 m/s^2 of
lateral grip in the hairpins. Measuring the real limit is the first step.

Sources: [ICRA 2025 results](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-icra-2025/),
[CDC-TF 2025 results](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-cdc-tf-2025/)

## Steps only the human can do

- Register (done by user), push the image to DockerHub, and submit the Phase 1
  and Phase 2 Google Forms. Everything else in this document is automated.


Updated 2026-08-07. Every number here came from a run on the Porto practice
track; nothing is a design estimate.

## The stack

```
LiDAR ─┬─ localization ──── pose + score ──┬─ pure_pursuit ─┐
       │      (MCL vs map)                 │   (raceline)   │
       ├─ follow_the_gap ──────────────────┘                ├─ arbiter ─► car
       └─────────────────────────────────────── (LiDAR) ────┘
```

`arbiter` is the only node that publishes to the actuators. It picks between
RACELINE (pure pursuit), REACTIVE (Follow-the-Gap) and BRAKE.

Offline pipeline, all track-agnostic — this *is* the finals drill:

```
record_mapping_run.sh → build_map_from_bag.py → extract_centerline.py
                      → optimize_raceline.py  → run_race_stack.sh
```

## Measured results

| What | Result |
|---|---|
| **Porto, MPC, scale 1.0** | **22 laps, 0 collisions**, best 11.32 s ← shipping |
| **ICRA (unseen), MPC, scale 1.0** | **13 laps, 0 collisions**, 20.9 s/lap |
| Follow-the-Gap alone | **11 laps, 0 collisions**, best 33.43 s — clears the qualification bar |
| Localization, offline on an unseen bag | median 0.093 m, p95 0.299 m |
| Localization, **live** | median **0.092 m**, p95 0.298 m, within 10 cm 55% |
| Pure pursuit tracking error | median 0.12 m, p95 0.36 m |
| Pure pursuit lap time | 20.7 s (but not yet collision-free) |
| Optimized raceline | 27.0 m, ideal lap ~10.4 s |
| Centerline "safe" line | 29.7 m, ideal lap ~11.0 s |
| Throttle calibration | **0.040 throttle per m/s** |

## Findings that cost the most time (do not re-derive)

1. **Headless mode freezes the LiDAR.** It is GPU-raytraced; `-batchmode
   -nographics` serves a stale scan forever. The sim must render.
2. **Encoder speed is unusable** — it aliases at the publish rate, correlating
   with true speed at |r| < 0.2. It caused both the throttle surging and a
   relocalization storm. Speed now comes from the calibrated throttle curve.
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
| Stanley, safe line, ay 6 | 1352 collisions — grip limit is real near ay 3-4 |
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

## Open

- **Build the winners' stack and benchmark against it directly** — see NEXT
  below. This is the one thing standing between us and the 6.53 s target.
- The arbiter needs rebuilding or dropping (#11).

Closed since this section was written: `speed_scale` is 1.0 and clean on both
tracks, and the optimized line was driven and **rejected on measurement** (57
collisions, 6 zones) — it is not "left on the table", it does not work here.


## NEXT SESSION — START HERE

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

1. **Validate the benchmark first — cheapest and it may reset the target.**
   Confirm 6.53 s is the same track, same vehicle model, same sim version.
   Their average of 4.38 m/s over a line whose tightest corner is R = 0.98 m
   implies lateral accelerations of 9-20 m/s2; our car visibly slides at 5-7.
   If their sim grips differently, the gap is partly an artefact and the whole
   plan changes. Do not skip this.
2. **Get VAUL's stack running** (github.com/vaul-ulaval, 1st at ICRA 2025 and
   CDC-TF 2025). We already know from the earlier code comparison that every
   block matches ours except odometry (they run wheel odometry AND kiss-icp).
3. **Diff the artefacts, not the prose**: their raceline vs ours on the same
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

## What the winners do differently (VAUL, 1st at ICRA 2025 and CDC-TF 2025)

VAUL (Université Laval) publish their tooling at `github.com/vaul-ulaval`.
Comparing their component list against ours:

| Component | VAUL | Ours | Verdict |
|---|---|---|---|
| Raceline optimization | `global_racetrajectory_optimization` (TUM) | same TUM `tph` | **match** |
| Localization | `particle_filter` (fork of f1tenth/MIT, ray casting via `range_libc`) | own MCL, likelihood field | equivalent approach |
| Reactive baseline | `wall_follow_node` | Follow-the-Gap | **match** |
| Odometry | `wheel_odometry_node` **and `kiss-icp` (LiDAR odometry)** | none — throttle-derived speed only | **GAP** |
| Control | `norlab_controllers` | Stanley / MPC | comparable |

**The gap is odometry.** Every other block we have an equivalent of. VAUL carries
two independent odometry sources; we have neither — our encoders are unusable
(the signal aliases) so the particle filter's motion model runs on a
throttle-derived guess, which is a straight-line calibration that ignores
cornering drag. That is survivable on slow, small Porto and is exactly what
breaks on the larger, faster ICRA circuit, where the filter's p95 error is
0.54 m against 0.215 m of corner clearance.

**KISS-ICP is the specific missing piece.** It is scan-to-scan LiDAR odometry:
it estimates the vehicle's motion between consecutive scans by registering the
point clouds, with no reliance on wheel encoders or a throttle model. That is
precisely the input our motion model lacks, it is sensor-derived rather than
assumed, and it degrades gracefully on any track because it uses the same LiDAR
we already trust for the sensor model.

Also worth noting: VAUL's `wheel_odometry_node` treats the encoder `position`
field as **counts**, dividing by counts-per-revolution, where we treated it as
radians. Scale alone does not explain our near-zero correlation with truth (a
scale factor cannot destroy correlation), but their node also carries an
explicit max-acceleration outlier rejection — suggesting they hit noisy encoder
data too. Worth one re-test under the counts interpretation before concluding.

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

## Remaining gap on unseen tracks

ICRA localization is now stable (0 relocalizations, p95 0.549 m) but still
exceeds the 0.215 m corner clearance of its generated line. With odometry no
longer the suspect, the remaining lever is **clearance**: make the line
generator trade lap time for margin wherever the corridor allows, targeting
corner clearance comfortably above the measured localization p95. That is task
#24 and is now the single thing between us and a working unseen-track stack.

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
