# Status — measured, not assumed

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

## Open

- Find the highest collision-free `speed_scale`, then move from the safe line
  to the sweep-corrected optimized line (ideal lap ~10.4 s, so there is a lot
  left on the table).
- The arbiter needs rebuilding or dropping (#11).

## Next

1. Push the qualification image to DockerHub and submit Phase 1 early (#13).
2. Raise `speed_scale` to the clean limit; then switch to the optimized line.
3. Rehearse the new-track drill end to end (#14).
