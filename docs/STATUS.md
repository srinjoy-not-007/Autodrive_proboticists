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
| Follow-the-Gap alone | **3 laps, 0 collisions**, 33.6 s/lap |
| Localization vs IPS truth | median **0.09 m**, p95 0.31 m, max 0.53 m, 0 relocalizations |
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

## Open

- Pure pursuit is fast (20.7 s vs FTG's 33.6 s) but not yet collision-free.
  The footprint correction and the reworked arbiter are the current attempt.
- Qualification only needs 10 clean laps and ignores speed, so **FTG alone is a
  valid fallback entry** if the raceline stack is not clean in time.

## Next

1. Confirm the reworked arbiter + sweep-corrected raceline runs clean, then
   raise `speed_scale` gradually.
2. Containerize (#12) so a submittable image exists well before Sep 12.
3. Rehearse the new-track drill end to end (#14).
