# Proposed changes — planner

**Goal: a clean lap around 6.5–7.0 s on Porto**, against the shipping 11.32 s.
The published winner on this track and this simulator build is 6.53 s.

Every number below was computed on 2026-08-25 through the repo's own
`tph.calc_vel_profile` code path, on the committed maps and lines. The model
reproduces the shipping file to 4 ms (11.043 s computed vs 11.047 s committed),
so these are directly comparable to the numbers in `docs/STATUS.md`.

Nothing here has been driven. See [validation](#validation) and
[risks](#risks-and-what-would-falsify-this).

---

## Summary

| | Now | Proposed |
|---|---|---|
| Line | `track_v2_safe` (centerline) | optimized at `--w-veh 0.40` |
| Worst-corner clearance | 0.325 m | 0.250 m |
| Peak \|κ\| | 1.019 | **1.164** (raceline today: 1.717) |
| Steering-lock margin | 75% | **53%** (raceline today: 4%) |
| ay cap | 3.0 | **8.0**, lateral-jerk limited |
| ax / braking | 3.0 / 3.0 | **7.1 / 7.1** |
| Ideal lap | 11.05 s | **6.49 s** |
| Predicted driven | 11.32 s | **~6.65 s** |

Driven prediction uses the repo's own measured driven/ideal ratio of 1.024
(11.32 / 11.05).

---

## The ladder

Each row adds one change to the row above.

| Step | Ideal lap | Δ | Cumulative |
|---|---|---|---|
| 0. shipping: safe line, ay 3, ax 3 | 11.043 s | | |
| 1. `m_veh` 3.906 → 3.47 | 11.043 s | +0.000 | 0.000 |
| 2. **fix `--brake`, ax 7.1** | 10.000 s | **−1.043** | 1.043 |
| 3. **`--w-veh 0.40` line** | 9.571 s | −0.429 | 1.472 |
| 4. ay 3 → 4 | 8.515 s | −1.056 | 2.528 |
| 5. ay 4 → 5 | 7.791 s | −0.724 | 3.252 |
| 6. ay 5 → 6 | 7.253 s | −0.538 | 3.790 |
| 7. ay 6 → 7 | 6.832 s | −0.421 | 4.211 |
| 8. ay 7 → 8 | **6.492 s** | −0.340 | **4.551** |

Note the shape: **the first second is a bug fix**, and the ay ladder has
diminishing returns — each extra 1.0 m/s² of grip buys less than the one before,
while the risk grows. Stopping at ay 7 still gives ~7.0 s driven, which meets
the goal.

---

## Change 1 — fix `--brake` so braking uses the real tyre limit

**What.** Wire `args.brake` into the `ggv`, and raise the longitudinal limits
from 3.0 to the measured tyre capability.

**Where.** [`tools/optimize_raceline.py:69`](../tools/optimize_raceline.py#L69)
and [`:151`](../tools/optimize_raceline.py#L151).

**Why.** `--brake` is declared and **never read** — see
[DEFECTS #5](DEFECTS.md#5---brake-is-dead-code). Braking deceleration silently
equals `--ax`, i.e. 3.0. The Unity tyre asset gives longitudinal μ 0.72, i.e.
**ax up to 7.1 m/s²**, so the profile is braking at 42% of what the car can do.
Because a velocity profile is mostly braking distance, this is the single
largest lever in the whole proposal.

**Effect.** 11.043 → 10.000 s. **−1.043 s, from a one-line bug.**

**Risk.** Low on paper, untested in practice. Braking at 7.1 m/s² has never been
driven; the open-loop throttle model may not deliver it, in which case the car
arrives at corners faster than planned. **This is the change most worth driving
in isolation first.**

---

## Change 2 — generate the line with `--w-veh 0.40`

**What.** Ship the optimized raceline generated at `--w-veh 0.40` instead of the
plain centerline.

**Where.** `tools/optimize_raceline.py` invocation; the default is currently
0.16.

**Why.** This is the discovery that makes the rest work. `docs/STATUS.md`
records the `--w-veh` sweep as "trades clearance for time along a nearly flat
frontier" and drops it. Nobody checked what it does to **peak curvature**:

| `--w-veh` | clearance | peak \|κ\| | lock margin | ideal (ay 3) |
|---|---|---|---|---|
| 0.16 (default) | 0.146 m | 1.717 | **4%** | 10.408 s |
| 0.32 | 0.225 m | 1.506 | 18% | 10.469 s |
| **0.40** | **0.250 m** | **1.164** | **53%** | 10.453 s |
| 0.48 | 0.292 m | 1.293 | 38% | — |
| ≥0.56 | QP fails — corridor exhausted | | | |

Widening the vehicle margin pushes the optimizer off the corridor edges, which
is precisely what stops min-curvature from spiking locally. **Clearance +71% and
peak curvature −32%, for 0.045 s.**

This matters beyond lap time. `docs/STATUS.md` attributes the ay-4.0 failure to
steering saturating 20% of samples; the current raceline sits at a **4% margin**
on the steering lock, so saturation is almost guaranteed. At 53% margin there is
actual room.

**Effect.** −0.429 s on top of change 1, and it is what makes high ay survivable.

**Also fixes.** This supersedes the planned "bound peak curvature" work
(PLANNER.md Phase 2) — an existing parameter achieves the same goal.

**Risk.** Clearance drops 0.325 → 0.250 m against the shipping line. Porto
localization p95 is 0.298 m, so by the error-budget logic `STATUS.md` applies to
ICRA, **0.250 m does not fit the error budget.** Hence the alternative below.

---

## Change 3 — raise ay with a lateral-jerk limit

**What.** Constrain `d(ay)/dt` when building the velocity profile, then raise the
ay cap from 3.0 toward 8.0. **Both together — not ay alone.**

**Where.** [`tools/optimize_raceline.py:149-158`](../tools/optimize_raceline.py#L149).

**Why.** The tyre peaks at ay 9.81 at 0.01 slip and halves to 4.91 by 0.10 slip,
so the ~5 m/s² ceiling this project kept hitting is the **asymptote, not the
limit**. But raising ay alone is already measured to fail: ay 4.0 gave 23
collisions, and the failures were at **curvature transitions** (κ 0.379 and
0.699, against a track maximum of 1.019) — one zone reverses from −0.611 to
+0.379 within 1.5 m while accelerating.

That is a **lateral jerk** problem, and the profile currently has no notion of
it: `ggv` is a constant ay at every speed, so the planner will happily schedule
that reversal.

**Effect.** ay 3 → 8 is worth 3.079 s on the change-2 line.

**Open question.** Whether `tph.calc_vel_profile` supports a jerk limit natively
or this needs a post-pass over `vx`. The library is installed and importable but
has not been read yet.

**Risk.** Highest of the three. Nothing above ay 4.0 has ever been driven here.
The hypothesis that jerk limiting is what unlocks the extra grip is **untested**.

---

## Change 4 — correct `m_veh` to 3.47

**What.** [`tools/optimize_raceline.py:158`](../tools/optimize_raceline.py#L158),
`m_veh=3.906` → 3.47, ideally read from `src/pit_crew/vehicle.py`.

**Why.** `compare_sim_builds.py` read 3.47 kg directly off the Unity rigidbody;
3.906 has no source. See [DEFECTS #6](DEFECTS.md#6-vehicle-mass-contradicts-the-measured-value).

**Effect on lap time: none** (+0.000 s — with `drag_coeff=0.0` the mass cancels
in a friction-limited profile). Include it as a correctness fix, not a
performance one.

---

## Two candidate configurations

Carry both to the track and let the drive decide.

### A — fast

```bash
.venv/bin/python tools/optimize_raceline.py data/racelines/track_v2_centerline.csv \
    --w-veh 0.40 --ay 8.0 --ax 7.1 --brake 7.1 --out data/racelines/track_v2_fast
```

Clearance 0.250 m · peak κ 1.164 · **ideal 6.49 s → driven ~6.65 s**

### B — safe

```bash
.venv/bin/python tools/optimize_raceline.py data/racelines/track_v2_centerline.csv \
    --w-veh 0.48 --ay 7.0 --ax 7.1 --brake 7.1 --out data/racelines/track_v2_safe2
```

Clearance 0.292 m · peak κ 1.293 · **ideal 6.98 s → driven ~7.14 s**

B keeps clearance close to the shipping line's 0.325 m and still meets the
"7 s is fine" bar. **If A collides, B is the fallback, not a slower A.**

---

## Validation

The repo's house rule is *drive it before you believe it*, and offline metrics
here have been wrong in both directions. Proposed order, so a failure is
attributable:

1. **Change 1 alone** (braking) on the existing safe line — isolates whether the
   car can actually brake at 7.1 m/s². Expect ~10.0 s.
2. **Changes 1+2+4** at ay 3.0 — isolates the new line geometry. Expect ~9.6 s.
3. **Ramp ay** 4 → 6 → 8 with jerk limiting, ten laps each.
4. Full 10-lap runs of A and B at the chosen setting.

Gates at every step: **0 collisions**, worst-corner clearance ≥ 0.25 m, peak κ
below the 1.783 steering lock, 0 direction reversals.

Before any of this, fix [DEFECTS #2](DEFECTS.md) — `make_track_lines.py` does
not validate the raceline it writes, and every candidate above flows through
that path.

---

## Risks, and what would falsify this

- **ay 8 has never been driven.** If jerk limiting does not unlock it, the
  ladder stops around ay 5–6 (7.8 s ideal, ~8.0 s driven). Still a large
  improvement, short of the goal.
- **0.250 m clearance is under the 0.298 m localization p95.** Candidate B
  exists for this reason.
- **Braking at 7.1 m/s² may not be deliverable** by an open-loop throttle model
  calibrated at 0.040 per m/s. If not, change 1's second disappears and
  everything downstream shifts.
- **The driven/ideal ratio of 1.024 was measured at ay 3.0.** At ay 8 the
  tracker is working far harder and the ratio will likely worsen — the ~6.65 s
  prediction is optimistic in a way that is not currently quantified.

**6.0 s is not on this list, because it is not reachable.** Every line that can
be generated from this corridor needs ay ≥ 9.85 for a 6.0 s lap, against a tyre
peak of 9.81 that exists only in a 0.01-slip window. The target is 6.5 s.

---

## Not doing, and why

- **Iterative reweighting to bound peak curvature** — change 2 achieves it with
  an existing parameter.
- **Re-sweeping `--kappa-bound`** — 0.8 / 1.0 / 1.2 / 1.6 give identical lines.
- **Re-tuning the centerline centering strength** — harder centering makes
  clearance *worse* (0.325 → 0.152 m).
- **Touching the controller** — it tracks its own plan to 2.4%. Not the
  bottleneck.
