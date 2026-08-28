# 2026-08-28 — braking authority, and the ceiling that was not a ceiling

Deepak G's document (`docs/Deepak G/25-08-26/`) is a full mathematical
formulation of the shipping MPC with a defect list. Its **B5** is the finding of
the project so far.

## B5: the controller could never brake

`ThrottleController.command` gated its braking branch behind `kp > 0`, and
nothing sets `kp`. Every throttle output was `0.040*v_target + 0.005`, which is
strictly positive. **Deceleration was coast-down and drag only** — while the
offline velocity profile was built assuming 4.0 m/s2 of braking the car could
not produce.

Fixed: braking now depends on `brake_gain` alone (default 0.0, so nothing
changes unless a caller opts in), and `mpc` feeds back the validated encoder
speed when `brake_gain > 0`.

## What that unlocked

Every run below: Porto, safe centerline, MPC, same session, 37-40 laps.

| config | median | best | collisions | 10-lap race |
|---|---|---|---|---|
| ay 3.5, no braking (old shipping) | 9.72 s | 9.59 s | 0 / 37 | 97.2 s |
| ay 3.5, brake_gain 0.3 | 9.89 s | 9.70 s | 0 / 43 | 98.9 s |
| ay 4.0, brake_gain 0.3 | 9.54 s | 9.40 s | 0 / 38 | 95.4 s |
| ay 4.5, brake_gain 0.3 | 9.31 s | 9.13 s | 0 / 39 | 93.1 s |
| **ay 5.0, brake_gain 0.3** | **9.14 s** | **9.01 s** | **0 / 40** | **91.4 s** |
| ay 6.0, brake_gain 0.3 | 9.44 s | 9.26 s | 0 / 38 | 94.4 s |

Two things to read carefully here.

**Braking alone is slower.** At the shipping ay 3.5 it costs 0.17 s/lap, because
the car had been overshooting a conservative profile and getting away with it.
The coast-down was acting as a free speed bonus. Braking only pays once the
profile is aggressive enough to need it.

**ay 4.0 and 5.0 were previously undrivable** — 23 and 31 collisions
respectively, and those numbers are why `docs/STATUS.md` recorded a grip
ceiling "demonstrated four independent ways". With braking authority both run
**perfectly clean**. The ceiling was substantially a missing-brakes artefact:
the car could plan a corner-entry speed it had no way to reach.

ay 6.0 turns back (9.44 s), so the optimum is near 5.0 and there is now a real
limit rather than an artificial one.
