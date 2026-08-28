# Review response — Vidit Gupta, 25 August

Source: `docs/Vidit Gupta/25 August/` (DEFECTS, IMPROVEMENTS, PLANNER,
PROPOSED_CHANGES, WORKING, RUNBOOK_MACOS).

**The review is against commit `8074658`, which is 25 commits behind the HEAD it
arrived at.** Several defects were fixed in between by work that had not been
communicated. That is not the reviewer's fault, and it does not make the review
less useful — two of the still-live items are real, and one of the *rejected*
performance claims turns out to be right and to overturn a conclusion of mine.

Every verdict below was re-derived against current `main`, not accepted from the
document. House rule 1 applies to the review as much as to us.

## Verdicts

| # | Claim | Verdict | Action |
|---|---|---|---|
| D1 | `icra2026_raceline.csv` undrivable and shipped | **TRUE, live** | fixed |
| D2 | `make_track_lines` never validates the raceline | **fixed since**, but a hole remained | fixed |
| D3 | drill bypasses `make_track_lines` | **fixed since** (Aug 23) | none |
| D4 | `stop_run.sh` zeroes actuators after killing nodes | **TRUE, live** | fixed |
| D5 | `--brake` is dead code | **fixed since** — and worth the ~1 s predicted | none |
| D6 | `m_veh=3.906` vs measured 3.47 kg | **TRUE, live** | fixed |
| D7 | drill clearance check reads the recon drive | **TRUE, live** | fixed |
| D8 | unpinned scipy pulls numpy 2.x | **fixed since** | none |
| D9 | `smoke_test.sh` tests a frozen LiDAR at the wrong rate | **TRUE, live** | fixed |
| D10 | retracted docstrings still state the retracted claim | **TRUE, live** | fixed |

## What was actually fixed

- **D1/D2** — `make_track_lines.py` validated the raceline but then promoted it
  to `<track>_raceline.csv` **regardless of the verdict**, which is how an
  undrivable ICRA line (2 direction reversals, min radius 0.327 m against a
  0.561 m lock) stayed on disk and in the submission image. A failed line is
  now deleted from the canonical name and kept as `_REJECTED.csv`.
  `submission/Dockerfile` also stopped copying the whole `data/racelines`
  directory — it now copies only the three canonical names per track, so no
  line nobody has cleared is reachable inside the image at all.
- **D4** — `stop_run.sh` killed the controller, slept 2 s, then zeroed. Because
  commands latch, the car drove its last command unsteered for those 2 s with
  the logger already dead. Zeroing *before* the kill does not work either (the
  live controller just overwrites it), so the zero publishers now start first
  and the nodes are killed underneath them.
- **D6** — `m_veh=3.906` → imported from `vehicle.MASS = 3.47`, the value
  `compare_sim_builds.py` read off the Unity rigidbody. Confirmed zero effect on
  lap time, exactly as the reviewer predicted; kept as a correctness fix and to
  remove the duplication that let the two drift apart.
- **D7** — the drill's clearance stage checked `<track>_traj.csv`, the recon
  drive, instead of the line about to be raced. Now checks `<track>_race.csv`.
- **D9** — `smoke_test.sh` ran the sim headless and expected 40 Hz, so it
  reported a healthy rate from a frozen sensor. Now runs windowed, expects
  ~10 Hz, and **fails** if consecutive scans are byte-identical.
- **D10** — the retracted "no trustworthy speed feedback" claim removed from
  `vehicle.ThrottleController` and `pure_pursuit`, and replaced with the real
  reason the loop is open (10 Hz sensor bus, not a bad encoder).

## The one that mattered: `--w-veh` had never been pushed past its default

`PROPOSED_CHANGES.md` is largely superseded — its Change 1 (`--brake` dead,
ax to 7.1) was found and shipped independently and delivered the ~1 s it
predicted, and its Change 3 (raise ay with a jerk limit) was tested to
destruction: ay 4.0 and 5.0 both collide, for reasons now understood.

**Change 2 is right, and it overturns a conclusion of mine.** `docs/STATUS.md`
carried "on Porto the geometry is settled — the best optimized line beats the
safe centerline by 0.05 s while giving up 44% of its clearance". That came from
`tools/sweep_lines.py`, whose grid **stopped at `w_veh 0.32`** and which also
ties `--ax` to `--ay` rather than using the shipping profile. Re-swept properly:

| `w_veh` | clearance | peak κ | ideal lap |
|---|---|---|---|
| safe centerline | 0.325 m | 1.019 | 9.59 s |
| 0.16 (old default) | 0.146 m | 1.717 | 9.56 s |
| 0.32 (old grid end) | 0.225 m | 1.506 | 9.44 s |
| **0.44** | **0.283 m** | **1.153** | **9.28 s** |
| 0.52 | 0.309 m | 1.266 | 9.90 s |

Non-monotonic in both clearance and time, so the value cannot be reasoned to —
it has to be swept per track. `make_track_lines.py` now sweeps it. On ICRA every
optimized line is still rejected on the steering lock at every `w_veh`, and the
safe centerline still wins, so this is a Porto-specific gain, not a general one.

## Driven result

All runs below are the **same session, same sim, same controller**, 37-40 laps
each. That matters: see the baseline note underneath.

| config | median lap | best | collisions | 10-lap race |
|---|---|---|---|---|
| **`safe`, ay 3.5 (shipping)** | **9.72 s** | 9.59 s | **0 / 37** | **97.2 s** |
| `w_veh 0.44` @ scale 0.97 | 9.69 s | 9.52 s | 1 / 40 | 99.4 s |
| `w_veh 0.44` @ scale 1.00 | **9.43 s** | **9.29 s** | 4 / 38 | 104.8 s |
| corrected LiDAR map + `w_veh 0.52` | 9.90 s | 9.74 s | 0 / 37 | 99.0 s |

**The `w_veh` line is genuinely faster per lap and still loses the race.** At
scale 1.00 it is 0.29 s/lap quicker than the shipping line and collides four
times; every collision is at (0.31, 5.35), its own tightest point, where the car
was ~0.5 m off its line. Backing off to scale 0.97 removes most of the
collisions and most of the gain with them.

The correct-LiDAR map was the obvious way to buy that clearance back — it cuts
lateral localization p95 from 0.155 m to 0.041 m. It did not help either: the
line it produces tracks *worse* (ideal 9.50 s, driven 9.90 s, a 4.2% gap against
the safe line's 1.4%).

So: **kept the sweep, did not ship the line.** The finding is real, the offline
conclusion I had recorded was wrong, and the driven answer is still the plain
centerline.

### The bug this exposed in my own pipeline

`make_track_lines.py` picks `race.csv` as *the fastest ideal lap that clears the
clearance floor*. On Porto that rule chose the 9.28 s-ideal line over the
9.59 s-ideal one — and the 9.28 s line drives **slower**. Offline lap time
mispredicted the driven ranking, so with the floor at 0.22 m the pipeline would
have shipped the worse line on a track nobody had driven.

`MIN_CLEARANCE` is now **0.30 m**, set from driven evidence rather than an error
budget: 0.283 m collides, 0.301 m and 0.325 m do not. Porto now selects `safe`
again, which is what actually wins.

### A baseline correction that affects every historical number

The shipping config measured **97.2 s** today against the **101.2 s** recorded in
`docs/STATUS.md` — same line (byte-identical), same controller, same
localization, no uncommitted changes to any of them. The difference is the
simulator: it now renders to an offscreen Xvfb display instead of competing with
the desktop compositor on the host X server.

Every lap time in this repo before 2026-08-28 was therefore measured under GPU
contention and is pessimistic by roughly 0.4 s/lap. This changes no ranking that
was measured A/B within a single session, but **cross-session comparisons
against older numbers are invalid**, which is exactly the mistake that made me
call the `w_veh` line a win an hour before disproving it.

## Checked and NOT a problem

- **"`alpha` is smoothed after the corridor is validated"** (IMPROVEMENTS).
  Plausible and worth checking — smoothing a constrained solution *can* violate
  the constraint. Measured on Porto at `w_veh 0.44`: smoothing moves `alpha` by
  at most **0.019 m** and leaves **zero** points outside the corridor the solver
  was given. A permanent check now prints this on every run, so if it ever does
  happen on another track it says so.

## Not adopted

- **`--w-veh 0.40` as a shipping default** — driven, slower. Superseded by
  sweeping the parameter and letting the gates choose.
- **PROPOSED_CHANGES' ay ladder to 8.0** — ay 4.0 and 5.0 were already driven
  and collide (23 and 31 times). The ladder's arithmetic is right and its
  physics is not: the tyre's 9.81 peak lives in a 0.01-slip window it cannot
  hold, and breakaway is abrupt against 10 Hz sensing.
- **The reviewer's 6.5 s projection.** It assumes the driven/ideal ratio holds
  at high ay. Measured today, the ratio *worsens* as the line gets tighter
  (1.4% on the safe line, 4.2% on the optimized one), which is the opposite of
  what the projection needs.
