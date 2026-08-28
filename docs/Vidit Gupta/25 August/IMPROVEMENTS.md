# Improvements

Things that are **not broken** but could be better. Separated from
[DEFECTS.md](DEFECTS.md) on purpose: nothing here is producing a wrong result
today, so none of it should jump the queue ahead of a real bug.

Ordered roughly by value per hour. Line references are against commit `8074658`.

---

## Planner

### Both "optimized" racelines sit within 4% of the steering lock

| Line | peak \|κ\| | min radius | lock |
|---|---|---|---|
| `track_v2_raceline` | 1.717 | **0.582 m** | 0.561 m |
| `icra2026_safe` | 1.712 | **0.584 m** | 0.561 m |
| `track_v2_safe` (shipping) | 1.019 | 0.981 m | 0.561 m |

The first two pass `drivability()` but with almost no margin — a few centimetres
of tracking error puts them past what the steering can physically produce. The
check is a hard pass/fail at the lock; it would be more useful as a **margin
report**, so a line that only just qualifies is visibly different from one that
comfortably does.

**Suggested:** have `drivability()` return the margin, and warn below (say) 20%.

### The corridor is shrunk in two places

[`optimize_raceline.py`](../tools/optimize_raceline.py) reduces the usable
corridor twice: once via `w_veh=0.16` passed into `tph.opt_min_curv`
([line 110](../tools/optimize_raceline.py#L110)), and again by subtracting
`corner_sweep()` — which already includes the body half-width — at
[line 116](../tools/optimize_raceline.py#L116).

Both are deliberate (the `--w-veh` help text says "extra safety on top of the
swept footprint"), so this is not a bug. But it makes the *effective* corridor
hard to reason about, and STATUS notes that `--kappa-bound` "does nothing
because the corridor binds first" — which is a claim about exactly this
quantity. Worth computing and printing the effective half-corridor per point.

### `alpha` is smoothed after the corridor is validated

[`optimize_raceline.py:124-127`](../tools/optimize_raceline.py#L124) smooths the
lateral-shift solution *after* the shrink-and-re-optimize loop has finished. The
smoothed line is what gets written; its clearance is never re-checked against
the corridor the solver was constrained by.

Smoothing a constrained solution can push it outside the constraint. Whether it
actually does here is **untested** — worth a single assertion after smoothing
rather than a rewrite.

### The body-sweep shrink is symmetric

[`optimize_raceline.py:116`](../tools/optimize_raceline.py#L116) subtracts the
same `sweep` from both the left and right widths. In a corner the body sweeps
*outward* at the front-outer corner and *inward* at the rear-inner — the
geometry is asymmetric. Symmetric shrinking is conservative, so it costs
available corridor rather than safety. On a track where "the corridor binds
first", that cost may be material.

### Line selection is single-objective

[`make_track_lines.py:98`](../tools/make_track_lines.py#L98) picks
`max(clearance)` and ignores lap time entirely. For a line called "safe" that is
defensible, but it means the tool cannot express "take the 5 cm less clearance
for 0.4 s". Once the raceline is validated (DEFECTS #2), a two-objective choice
becomes possible.

---

## Measurement and tooling

### No lateral-jerk metric exists

The measured failure at ay 4.0 was at **curvature transitions**, not the tightest
corners (STATUS: κ 0.379 and 0.699 against a max of 1.019; steering at full lock
20% of samples). Nothing in the toolchain measures the rate of change of lateral
acceleration, so the quantity the failure is described in cannot be checked.

This is the single most useful thing to add before doing planner work — see
[PLANNER.md](PLANNER.md).

### Steering-saturation fraction is computed ad hoc

The "20% of samples at full lock" figure is quoted in STATUS but no tool
produces it. [`steering_smoothness.py`](../tools/steering_smoothness.py) reports
reversals, rate and mean lock — adding the saturated fraction is a two-line
change and makes the clean/crashing comparison reproducible.

### Free-space carving is sparse

[`build_map_from_bag.py`](../src/tools/build_map_from_bag.py) subsamples rays
`[::6]` and steps 3 cells along each, leaving **2,413 single-cell unknown
speckles** inside the drivable area on the ICRA map (median blob size: 1 cell).

Nothing currently depends on confirmed-free cells, so this has no effect today.
It would matter for any future check that requires positive evidence of free
space, and it makes the map look worse than the recon actually was.

### `pf_core.py` and `localization.py` duplicate the filter

[`pf_core.py`](../src/pit_crew/pf_core.py) is the ROS-free copy used for offline
replay, and it reimplements `predict`, `update`, `resample` and `relocalize`
alongside the live node. The two are currently in agreement, but there is
nothing enforcing that — a fix applied to one can silently miss the other.

The live node could import the core rather than restate it.

---

## Controller

### MPC scores a first step four times larger than it applies

[`mpc.py:227`](../src/pit_crew/mpc.py#L227) searches candidates across
`steer_rate × control_dt × 4` = ±0.256 rad, then
[line 237](../src/pit_crew/mpc.py#L237) applies at most
`steer_rate × control_dt` = ±0.064 rad. So the rollout evaluates a first step
that is then clipped to a quarter of its size, and the applied action is not the
optimum of the cost that selected it.

It works — MPC is the only controller measured clean on an untuned track — so
this is a design note, not a bug. But the module docstring claims it "never asks
for a change the actuator cannot deliver", and the cost function does exactly
that. Either the search range or the docstring should change.

### `ThrottleController` idles at nonzero throttle

[`vehicle.py:85`](../src/pit_crew/vehicle.py#L85) computes
`u = ff * v_target + idle` with `idle=0.005`. At `v_target = 0` the command is
0.005, not zero — the car creeps rather than stopping. Harmless while racing,
but it interacts badly with the latch behaviour in DEFECTS #4.

### Encoder glitch rejection is far too loose

[`vehicle.py:58`](../src/pit_crew/vehicle.py#L58) rejects a speed sample only if
`abs(v) > TOP_SPEED * 1.1` = 25.2 m/s. The car races at ~5 m/s. A teleport
producing a spurious 20 m/s passes the filter unchallenged and enters the
particle filter's motion model.

A threshold tied to plausible vehicle speed (say 3× the profile maximum) would
actually catch what the comment says it catches.

---

## Repo hygiene

### `scan_odometry.py` has no entry point

It is a library (no `main()`), correctly absent from
[`setup.py`](../src/setup.py)'s `console_scripts`. Fine — but it reads as a node
alongside twelve modules that are nodes. A one-line note at the top of
`setup.py`, or moving it, would save the next reader the check.

### Five scripts do not work outside Linux+NVIDIA

`sim_restart_connect.sh`, `run_sim.sh`, `run_devkit.sh`, `smoke_test.sh` and
`new_track_drill.sh` all assume the Dockerised simulator with `--gpus all` and
an X11 socket. On macOS the simulator runs natively instead — see
[RUNBOOK_MACOS.md](RUNBOOK_MACOS.md). Worth a note in the script headers so the
next person does not debug the wrong layer.

### `docs/PLAN.md` is historical

Written 2026-08-06 and framed as "today = Aug 6" with a week-by-week timeline
that has since been overtaken. It is useful as a record of the original strategy
but reads as current. A dated header would fix it.
