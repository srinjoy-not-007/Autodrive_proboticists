# Planner work

Working document for improving the offline planner — the `tools/` pipeline that
turns an occupancy grid into a racing line and a velocity profile.

Scope is deliberately the **plan**, not the tracker. The repo's own measurement
is that the controller holds its own plan to within 2.4% (11.32 s driven against
11.05 s ideal), so remaining lap time is in the plan.

---

## 1. Baseline — reproduced 2026-08-25

Recomputed from the committed maps and lines. Every figure matches
`docs/STATUS.md` to three decimals, so the published numbers are trustworthy and
these are the values to beat.

| Line | clearance | peak \|κ\| | κ p99 | v max | ideal lap |
|---|---|---|---|---|---|
| `track_v2_safe` **(shipping)** | **0.325 m** | **1.019** | 1.012 | 5.39 m/s | **11.05 s** |
| `track_v2_raceline` | 0.146 m | 1.717 | 1.433 | 4.76 m/s | 10.41 s |
| `icra2026_safe` | 0.564 m | 1.712 | 1.438 | 6.36 m/s | 19.88 s |
| `icra2026_raceline` | 0.079 m | 3.057 | 2.698 | 5.94 m/s | 18.89 s |

`icra2026_raceline` is **undrivable** — see [DEFECTS.md #1](DEFECTS.md).

Driven results for context:

| Config | Best lap | Collisions |
|---|---|---|
| safe line + MPC, ay 3.0 (shipping) | **11.32 s** | 0 over 22 laps |
| safe line + MPC, ay 4.0 | 10.11 s | 23 over 21 laps |
| optimized raceline, ay 3.0 | 10.65 s | 57 over ~25 laps |
| **published winner, same track & sim build** | **6.53 s** | 0 |

The 6.53 s benchmark was verified comparable asset-by-asset with
`tools/compare_sim_builds.py` — byte-identical track mesh, identical physics,
identical vehicle. The gap is real.

---

## 2. The hypothesis

Three measured facts, which together point somewhere specific.

**The tyres have roughly 3× the grip we use.** Read out of the Unity asset:
lateral μ peaks at **1.00 (ay 9.81 m/s²) at 0.01 slip**, and halves to 0.50
(ay 4.91) by 0.10 slip. The velocity profile caps at **ay 3.0**. The ~5 m/s²
ceiling the project kept hitting is the tyre's *asymptote*, not its limit.

**Raising the cap alone fails at transitions, not at the tightest corners.** At
ay 4.0 the failures were at κ 0.379 and 0.699 against a track maximum of 1.019.
One zone reverses from −0.611 to +0.379 within 1.5 m while accelerating
2.38 → 3.25 m/s. Steering sat at full lock **20%** of samples, versus ~5% in the
clean run.

**Min-curvature spikes locally.** The optimized line's *peak* κ is 1.717 —
**worse** than the plain centerline's 1.019 — because `opt_min_curv` minimises
the integral of κ² and is free to spike anywhere.

> **Hypothesis: the car is not grip-limited, it is slip-window limited.**
> The plan asks lateral force to appear and reverse faster than the tyre can
> build it inside the 0.01–0.10 slip window, so the tyre drops onto its
> asymptote exactly where the line changes direction. If that is right, the
> lever is not "more ay" — it is bounding **how fast the plan demands lateral
> force to change**, in both the geometry and the speed profile.

This predicts something falsifiable and cheap to test: raising ay *together
with* a transition bound should behave differently from raising ay alone.

---

## 3. Work plan

### Phase 0 — instruments and headroom · half a day

Nothing here changes the planner; it builds the means to judge it.

1. **Add two metrics that do not exist yet.** Peak/percentile **lateral jerk**
   (d(ay)/ds along the line, and d(ay)/dt at profile speed) and **steering
   saturation fraction**. The hypothesis above is stated in these quantities and
   currently cannot be checked. See [IMPROVEMENTS.md](IMPROVEMENTS.md).

2. **Measure the actual lateral freedom.** Porto's minimum corridor is 0.875 m;
   `corner_sweep()` consumes up to 0.60 m, and `w_veh` takes more on top. If
   what remains is a few centimetres, then **raceline optimisation cannot help
   on this track** and the entire win must come from the velocity profile. One
   number, and it redirects the rest of the week.

   This also settles the standing claim that `--kappa-bound` "does nothing
   because the corridor binds first".

3. **Fix [DEFECTS #2](DEFECTS.md)** first — validation coverage is a known hole,
   and every candidate line produced below flows through it.

### Phase 1 — lateral-jerk-limited velocity profile · 1–2 days

**Highest expected value, and independent of Phase 0's answer.**

The profile currently has no notion of how fast lateral acceleration changes:
`ggv` is a constant `ay` at every speed
([`optimize_raceline.py:151`](../tools/optimize_raceline.py#L151)), so the
planner will happily schedule −0.611 → +0.379 curvature in 1.5 m while
accelerating.

The change: constrain `d(ay)/dt` when building the profile, then raise the ay
ceiling above 3.0. **Both must move together** — raising ay alone is already
measured to crash, and the hypothesis says the jerk bound is what makes the
extra grip usable.

Open question to settle first: whether `tph.calc_vel_profile` supports a jerk
limit natively or whether this needs a post-pass over `vx`. The library is
installed but has not been read.

Worth folding in here: **[DEFECTS #5](DEFECTS.md)** — braking deceleration is
currently pinned to `--ax` (3.0) because `--brake` is dead code, while the tyre
model gives longitudinal μ 0.72, i.e. **ax up to 7.1 m/s²**. Braking distance is
most of what a velocity profile is made of, so this may be free lap time sitting
behind a one-line bug.

### Phase 2 — bound peak curvature in the line · 1–2 days

**Conditional on Phase 0 showing real lateral freedom.**

The objective minimises Σκ² and spikes locally; what the car needs is a bound on
*max* κ and on how fast κ changes along the path.

Approaches, cheapest first:

1. **Iterative reweighting** — solve, find the spikes, add local weight there,
   re-solve. This reuses the existing QP and fits the code as written: the
   `sweep_iters` loop at [line 107](../tools/optimize_raceline.py#L107) is
   already an iterate-and-re-solve structure.
2. An explicit minimax term.
3. Post-smooth κ subject to a clearance constraint.

Target: beat 11.05 s ideal without exceeding peak κ 1.019 or dropping below
0.325 m clearance.

### Phase 3 — drive it · half a day

Offline metrics have been wrong in this repo **in both directions**: a line with
healthy measured clearance that crashed every lap because it doubled back on
itself, and a sensor written off as junk that was fine. Nothing counts until it
has run.

MPC on the new line at `speed_scale 1.0`, ten-plus laps, lap times and
collisions from `race_logger.py`. Zero the throttle before stopping or the
counter lies — [DEFECTS #4](DEFECTS.md).

---

## 4. Scorecard

Every candidate line reports these before anyone argues about it. All computable
offline from data already in the repo.

| Metric | Baseline | Rule |
|---|---|---|
| Ideal lap time | 11.05 s | lower is the point |
| Worst-corner clearance | 0.325 m | **hard floor** — never trade below |
| Peak \|κ\| | 1.019 | must not increase |
| κ p99 | 1.012 | catches spikes the max hides |
| Min-radius margin vs 0.561 m lock | 75% | new; see IMPROVEMENTS |
| Peak lateral jerk | — | **new**; the Phase 1 quantity |
| Steering saturation fraction | ~5% clean / 20% at ay 4.0 | **new**; separated the clean run from the crashing one |
| Direction reversals | 0 | disqualifying |

---

## 5. Do not re-derive

Already swept and measured. Re-running these is time spent confirming
`docs/STATUS.md`.

- **`--kappa-bound` does nothing on Porto.** 0.8 / 1.0 / 1.2 / 1.6 give
  identical lines; the corridor binds first.
- **`--w-veh` trades clearance for time along a nearly flat frontier:**
  0.146 m → 10.41 s against 0.292 m → 10.67 s.
- **Raising ay alone:** ay 4.0 → 10.11 s and 23 collisions. ay 5.0 was never
  driven because steering already saturated 20% of the time at 4.0.
- **The optimized min-curvature line loses to the plain centerline on Porto** —
  faster in theory (10.41 s vs 11.05 s) but 0.146 m clearance and ~57
  collisions. Better tracking did not rescue it.
- **Do not re-tune the centerline centering strength.** Harder centering makes
  the line jagged and clearance *worse* (Porto 0.325 → 0.152 m).
- **`skidpad.py`'s Porto numbers are contaminated** — the car grinds the walls
  once it slides. Do not cite them.

---

## 6. Honest caveat

Nothing here promises 6.53 s. The gap is **1.73×**, the repo's own swept
frontier tops out around 10.5 s, and it remains possible the winning teams do
something structurally different that none of this reaches.

This is the best-evidenced attack on the gap, not a guarantee of closing it. A
*measured* negative result — "lateral-jerk limiting buys nothing on Porto, here
are the numbers" — is a legitimate outcome in this project's culture, and more
useful than a tuned parameter with no explanation.
