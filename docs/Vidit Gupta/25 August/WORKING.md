# Working notes

Living reference for how the planner's data actually works — the file formats,
what every number means, and how the car consumes them.

**Keep this updated.** Any time the pipeline, the file formats, the column
meanings, or the way a consumer reads them changes, edit this document in the
same commit. See the [changelog](#changelog) at the bottom.

Numbers below are from `track_v2` (Porto) at commit `8074658`.

---

## 1. There are two CSV formats, not one

People say "the raceline CSV" as if there is one file. There are two, with
different shapes and different jobs.

### Centerline CSV — the planner's *input*

Written by [`extract_centerline.py`](../tools/extract_centerline.py).

```
# x_m,y_m,w_tr_right_m,w_tr_left_m
0.6429,3.1680,0.4000,0.5000
0.6399,3.0910,0.4250,0.4750
```

400 points, 4 columns. This is TUM's `reftrack` format: a point, plus **how much
room there is on each side of it**.

| Property | Porto value |
|---|---|
| point spacing | 0.0747 m (median) |
| loop length | 29.745 m |
| corridor width (`w_r + w_l`) | min **0.875 m**, median 1.200, max 2.275 |
| closure gap | 0.0772 m — one spacing |

The widths are measured by casting a normal from each point out to the corridor
boundary. That boundary is the **sealed, flood-filled** drivable region (walls
dilated by 3 cells ≈ 7.5 cm), which is why
[`optimize_raceline.py`](../tools/optimize_raceline.py) notes that `w_veh` only
needs to cover the car plus a small margin — some safety is already baked in.

### Raceline CSV — the planner's *output*

Written by [`optimize_raceline.py`](../tools/optimize_raceline.py). **This is the
only file the car ever reads.**

```
# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2
0.0000,0.6429,3.1680,3.1009,0.0527,2.9377,2.5452
0.0997,0.6391,3.0682,3.1065,0.0527,3.0228,2.5185
```

299 points, 7 columns, uniform 0.0997 m spacing — that is the `--stepsize 0.10`
flag, after resampling onto a spline.

Both files are **open lists describing a closed loop**: there is no duplicated
closing point, so every consumer indexes with `% len()`.

---

## 2. A worked corner

The tightest corner on Porto, straight out of `track_v2_safe.csv`. Everything in
section 3 can be read off this table.

```
     s_m     x_m     y_m     psi   kappa    R(m)     vx      ax   ay=v²κ
   9.177   0.258  -5.982   2.826  -0.293    3.41  2.290  -1.415     1.538
   9.675   0.063  -6.441   2.643  -0.460    2.17  1.982  -1.129     1.809
  10.174  -0.232  -6.840   2.339  -0.791    1.26  1.752  -0.401     2.429
  10.473  -0.471  -7.018   2.068  -1.012    0.99  1.716   0.000     2.977
  10.573  -0.561  -7.061   1.965  -1.019    0.98  1.716   0.000     3.000  ← APEX
  10.673  -0.655  -7.094   1.861  -1.019    0.98  1.716   0.000     3.000
  10.772  -0.751  -7.117   1.758  -1.019    0.98  1.716   0.000     3.000
  10.872  -0.850  -7.131   1.659  -0.924    1.08  1.716   0.281     2.719
  11.072  -1.049  -7.133   1.507  -0.629    1.59  1.773   1.023     1.977
  11.371  -1.346  -7.090   1.358  -0.393    2.55  1.967   1.480     1.520
```

A straight, for contrast:

```
   2.095   0.644   1.074  -3.120   0.001    1667  4.429   2.989
```

---

## 3. What each column means

### `s_m` — arc length

**Distance travelled *along the path*, from the start point.** Not straight-line
distance from an origin — an odometer reading that only counts up as you drive
the line. Two points 5 m apart in `s` are 5 m apart *as driven*, even if the path
loops back and they are 30 cm apart in space.

It is the natural coordinate for a closed track: "where am I round the lap"
becomes one number instead of two. Range 0 → 29.724 m, constant 0.0997 m step.

**Used for:** lap time (`Σ ds / vx`), and converting "look 0.35 s ahead" into
"look N points ahead". Nothing steers with it.

### `x_m, y_m` — position

**Where the point is in the map frame**, metres. Same frame as the occupancy grid
and the localization output, which is what lets `footprint_analysis.py` look up
wall distance directly at a line point.

This is the position of the **rear axle**, not the car's centre. The car is
0.5 m long, so the body sweeps well outside this path in a corner — the entire
reason `corner_sweep()` exists, and why a line with 0.335 m of centre clearance
once had 0.000 m of *corner* clearance.

**Used for:** every controller does `argmin(distance)` against these to locate
itself on the plan.

### `psi_rad` — heading

**Which way the car points at that location**, radians. Through the corner above
it falls 2.826 → 1.358, about 1.5 rad (85°), because the car is turning right.

Heading is the integral of curvature: **dψ/ds = κ**. Check it on the table —
between rows 1 and 2, `s` advances 0.498 m at mean κ ≈ −0.37, predicting
Δψ ≈ −0.18. Observed: 2.826 → 2.643 = −0.183.

> **Gotcha.** TUM writes this **north-zero** — 0 means +y, not +x — so it sits
> −90° from the `atan2(dy, dx)` convention. Measured median offset: **−89.7°**.
> Every consumer in this repo ignores the column and recomputes heading from the
> geometry. Column 3 is effectively dead weight.

### `kappa_radpm` — curvature

**How sharply the path bends**, radians per metre. Reciprocal of turn radius:
**κ = 1/R**. So κ = −1.019 means R = 0.98 m.

- `κ = 0` → straight (the straight row shows κ = 0.001, R = 1667 m)
- large `|κ|` → tight
- **sign is direction**: negative = turning right, positive = left

The most important column. It sets the steering feedforward — `atan(L·κ)`, so κ
*is* the steering command in disguise — and it sets the speed via the friction
circle below.

**Hard limit:** the steering lock allows R ≥ 0.561 m, so **|κ| must stay under
1.783**. Porto's shipping line peaks at 1.019. `icra2026_raceline.csv` reaches
3.057, which is why it is undrivable — see [DEFECTS.md #1](DEFECTS.md).

### `vx_mps` — target speed

**How fast to be at that point**, m/s. In the table it falls 2.290 → 1.716,
holds flat through the apex, then climbs back.

It is not chosen — it falls out of the curvature. Cornering generates lateral
acceleration:

```
ay = vx² · κ            (circular motion: v²/R)
```

The planner takes the largest `vx` at each point that keeps `ay` under the cap.
Read the last column of the table: it climbs to exactly **3.000** at the apex,
which is the `--ay 3.0` flag. Speed is whatever the curvature permits.

Across the whole file: **7%** of points sit at ≥99% of the cap (genuine
corners), **48%** sit below half the cap (straights and braking zones, where
`--vmax` or the braking ramp binds instead).

### `ax_mps2` — longitudinal acceleration

**Braking or accelerating**, m/s². Negative slows, positive speeds up. The most
physically revealing column:

```
entry   ax = -1.415, -1.129, -0.401     braking into the corner
apex    ax =  0.000,  0.000,  0.000     ← nothing left
exit    ax = +0.281, +1.023, +1.480     back on the power
```

**Three rows of exactly zero at the apex is the friction circle.** A tyre has one
grip budget shared between turning and changing speed. At the apex `ay` is at the
full 3.0, so nothing remains for `ax` — you cannot brake or accelerate while
cornering at the limit without exceeding total grip.

It is also why braking begins ~1.4 m *before* the corner. Throttle is open loop
on a ~10 Hz sensor loop, so the car cannot brake reactively at corner entry; it
has to already be slow when it arrives.

Verified derived: `ax` correlates **0.9938** with `v·dv/ds` recomputed from
column 5.

**Used for:** nothing online. No controller reads column 6. Diagnostic only.

---

## 4. How the columns interlock

Only the geometry is independent. Everything else follows from it:

```
x, y  ──►  s      (integrate distance along the path)
      ──►  psi    (direction of travel)
      ──►  kappa  (rate of change of psi)
                  ──►  vx   (from ay = v²κ ≤ cap)
                            ──►  ax   (rate of change of vx)
```

Which is why the planner's only real freedom is **`alpha`**: one number per
point, how far to slide sideways along the local normal. Move the line, and
curvature, speed and lap time all fall out of that single choice.

`--centerline-only` sets `alpha = 0` — follow the centerline exactly, and only
compute a speed profile. **That is what currently ships.**

---

## 5. How the planner builds the output

[`optimize_raceline.py`](../tools/optimize_raceline.py), in order:

1. **Thin** the centerline to ~160 points (`step = len // 160`) so the QP stays
   tractable.
2. **Fit splines** — `calc_splines` returns coefficients plus **normal vectors**
   at each point.
3. **Solve for `alpha`** — `opt_min_curv` (a QP minimising Σκ²), or zeros under
   `--centerline-only`. For the optimized line this runs inside the body-sweep
   loop: solve, measure how far the body sweeps at the resulting curvature,
   shrink the corridor by that, re-solve. It must iterate because the sweep
   depends on the solution's own curvature.
4. **Smooth `alpha`** with a 5-point circular kernel — isolated single-point
   solutions become curvature spikes that crater the velocity profile.
5. **Rebuild** — `create_raceline` applies `alpha` and resamples at
   `--stepsize`, producing `x, y, s`.
6. **Differentiate** — `calc_head_curv_an` gives `psi` and `kappa` analytically
   from the spline; a 5-point circular median filter then removes
   spline-junction spikes (unfiltered these cost ~6 s/lap).
7. **Velocity profile** — `calc_vel_profile` takes `kappa` plus a friction-circle
   `ggv` and returns `vx`; `calc_ax_profile` differentiates that into `ax`.

Lap time, the number everyone quotes, is simply:

```
Σ (ds / vx) = 11.047 s
```

---

## 6. Three surprises in how the car consumes it

### `psi_rad` is written and then ignored

See the gotcha above. Everything recomputes heading from geometry.

### `kappa` is re-smoothed by the controller, and it matters

[`mpc.py:92`](../src/pit_crew/mpc.py#L92) does not use column 4 as written:

```python
span = max(1, int(round(0.5 / spacing)))          # = 5 at 0.0997 m spacing
self.kappa = ndimage.uniform_filter1d(rl[:, 4], size=2*span+1, mode='wrap')
```

An 11-point window ≈ **1.10 m of arc**. Effect on the shipping line:

```
peak |kappa|   1.019  →  0.882      (−13%)
```

Porto's tightest corner has R = 0.98 m, so **the smoothing window is wider than
the corner itself.** The feedforward `atan(L·κ)` therefore commands about 13%
less steering at the apex than the plan asks for, and cross-track feedback makes
up the difference.

It is deliberate — at 0.1 m spacing, centimetre-scale spline wiggle dominates the
heading estimate and goes straight into the steering as visible oscillation. But
the consequence is that **the peak curvature optimised offline is not the peak
curvature the car feedforwards.** Directly relevant to any peak-κ bounding work;
a curvature feature narrower than about a metre of arc is partly smoothed away
before it becomes steering.

### `vx` is read as a forward minimum, not a point lookup

[`mpc.py:242`](../src/pit_crew/mpc.py#L242):

```python
horizon = max(0.2, self.speed * self.brake_horizon)   # brake_horizon = 0.35 s
n_ahead = max(2, int(horizon / spacing))
idx = (i0 + np.arange(n_ahead)) % len(self.v_prof)
self.speed = float(self.v_prof[idx].min()) * self.speed_scale
```

It takes the **slowest speed in the next ~0.35 s of travel**, not the speed at
the current point, because open-loop throttle on a 10 Hz loop cannot brake
reactively at corner entry.

The horizon is deliberately **short**: the profile already contains its own
braking distance, so a long horizon double-counts it. Measured: 52 s/lap at a
1.2 s horizon versus 11 s at 0.35 s.

---

## 7. Practical implications for planner work

- Changes reach the car through **columns 4 and 5 only**. Nothing else is read
  at runtime.
- Column 4 is **low-passed over ~1.1 m** on the way in.
- The only geometric freedom is **`alpha`**, one scalar per point along the
  normal.
- `ay = vx²·κ` is the whole speed model. Raising the cap raises speed everywhere
  the corner binds — 7% of points on Porto.
- The apex `ax = 0` rows show there is **no spare grip mid-corner**. Any scheme
  that wants to brake or accelerate there must first reduce `ay` demand.

---

## Changelog

| Date | Change |
|---|---|
| 2026-08-25 | Created. CSV formats, per-column meanings, worked corner, consumption gotchas — all measured from `track_v2` at commit `8074658`. |
| 2026-08-25 | Verified the velocity model: reproducing `tph.calc_vel_profile` with the committed `kappa` gives 11.043 s against the committed 11.047 s, and 10.408 s against 10.41 s for the raceline. The pipeline is exactly reproducible offline. |
| 2026-08-25 | Measured Porto corridor headroom: after body-sweep shrink and `w_veh`, lateral freedom is min **0.370 m**, median 0.721 m, and 82% of points have >0.50 m. This contradicts the standing claim in `STATUS.md` that "the corridor binds first" — see [PROPOSED_CHANGES.md](PROPOSED_CHANGES.md). |
