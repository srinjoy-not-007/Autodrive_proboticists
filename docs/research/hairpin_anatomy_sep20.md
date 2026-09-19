# The two hairpins at 0.25 m — where the 0.58 / 1.13 s/lap actually goes

Sep 20, agent, branch `hairpin-profiles`. Geometry fixed
(`data/racelines/finals_mesh_apex_f42k_race.csv`, ideal 10.282 s); everything
here is a SPEED-PROFILE question.

Sources: the two v9 centuries, both 100 laps / 0 collisions —
`data/logs/race_20260919_170814.csv` (real display, dt_p50 **0.0580**, median
10.84) and `data/logs/race_20260919_190223.csv` (Xvfb, dt_p50 **0.0960**,
median 11.15). Truth pose (IPS `x,y`), truth speed from the IPS path, rack
angle from `steering`, published command from `cmd`, yaw rate differentiated
from the truth heading. Per-bin times are the median over the 100 passages of
the time between the plan's own gate planes at 0.25 m spacing, differenced
against the plan's `ds/v` over the same interval.

Reproduce: `/tmp/.../anatomy2.py` in the session scratchpad; the builder for
the candidate lines is `scripts/hairpin_profile.py` (committed).

---

## 1. The headline: the PLAN's own yaw-rate demand is past the breakaway

`tools/sysid_breakaway.py` (Sep 19, `-explore` staircase) put breakaway at a
**2.71 rad/s** asked yaw rate held steady, ~3.5 for excursions under 0.3 s.
The shipped profile asks for more than that over **0.8 m of each hairpin**:

| s | kappa | R | v_plan | **v·kappa asked** | ay = v²·kappa |
|---|---|---|---|---|---|
| 18.79 | 1.62 | 0.62 | 1.69 | **2.74** | 4.63 |
| 18.89 | 1.62 | 0.62 | 1.77 | **2.87** | 5.08 |
| 18.99 | 1.62 | 0.62 | 1.92 | **3.12** | 5.98 |
| 19.09 | 1.49 | 0.67 | 2.06 | **3.06** | 6.30 |
| 19.19 | 1.44 | 0.69 | 2.12 | **3.05** | 6.45 |
| 19.29 | 1.30 | 0.77 | 2.17 | **2.82** | 6.11 |
| 19.39-19.49 | 1.22 | 0.82 | 2.21-2.24 | **2.70-2.74** | 5.97-6.15 |
| 43.28-43.38 | 1.76 | 0.57 | 1.57 | **2.77** | 4.35 |
| 43.48 | 1.76 | 0.57 | 1.75 | **3.08** | 5.37 |
| 43.58 | 1.63 | 0.61 | 1.95 | **3.19** | 6.23 |

Whole-line maxima: **3.19 rad/s** asked (s 43.58), against an ay that never
exceeds 6.5 under a cap of 9.0. **The ay cap never binds in a hairpin.** No
`reprofile.py` rung can reach this, because the binding quantity is `v·kappa`,
not `v²·kappa`; that is why the ay ladder (ay 6.75, 9.0, h2/h4 blends) has
always been a wash in these two corners. The other points above 2.5 rad/s are
s 24.1-24.6 and 37.4-37.6 — the corner explicitly closed on Sep 19 — and are
left untouched here.

The safe reference (`_safe_ref.csv`) is identical to the race line through both
hairpins (the shipped blend holds them at alpha 0), so "put it back to safe
speeds" has already been tried and does not address this: safe asks 3.19 too.

## 2. HP1 (s 18.5-20.5): the car apexes a metre late because it turns in too tight

Display, per 0.25 m (cumulative loss in the right column):

```
   s   kappa  v_plan v_drv   dv | loss    cum  | thr==0 off(+=inside) rack_p90 yaw_p50 yaw_p90 a_drv
 17.50  0.07   3.43   3.70  +0.26| +0.000 -0.024|  47%   -0.000      0.111    0.96    1.31   -5.16
 18.25  0.05   2.39   2.47  +0.08| -0.001 -0.030|  34%   +0.076      0.175    0.77    1.20   -4.31
 18.50  0.44   1.96   1.99  +0.03| -0.006 -0.036|  10%   +0.143      0.213    1.15    1.53   -3.04
 18.75  1.58   1.77   1.77  -0.01| -0.075 -0.111|   0%   +0.177      0.319    1.81    2.24   -1.57
 19.00  1.49   2.06   1.71  -0.35| -0.003 -0.114|   0%   +0.107      0.455    2.63    2.89   +0.78
 19.25  1.26   2.20   1.81  -0.38| +0.062 -0.052|   4%   -0.098      0.519    2.91    3.04   -0.53
 19.50  0.97   2.32   1.51  -0.81| +0.097 +0.045|   0%   -0.245      0.523    2.37    2.67   -1.16
 19.75  0.64   2.54   1.47  -1.07| +0.107 +0.152|   0%   -0.297      0.521    1.75    2.21   +2.05
 20.00  0.40   2.82   1.95  -0.87| +0.057 +0.209|   0%   -0.279      0.222    1.17    1.29   +3.99
 20.25  0.27   3.13   2.44  -0.68| +0.024 +0.233|   0%   -0.236      0.154    0.83    1.07   +4.17
 20.50  0.18   3.44   2.91  -0.53| +0.017 +0.250|   0%   -0.180      0.076    0.28    0.47   +5.38
 21.00  0.04   4.07   3.69  -0.39| +0.010 +0.267|   0%   -0.079      0.049    0.87    1.04   +4.70
```

Total over s 15.5-23.0: **+0.281 s/lap** (display), **+0.333** (Xvfb). Reading
it:

* **Entry is not the problem.** Through s 15.5-18.5 the car is 0.2-0.36 m/s
  ABOVE plan (display) or 0.25-0.57 (Xvfb) and is *ahead* of the plan's clock by
  0.036 s. The planned brake works: `throttle == 0.0` on 36 % of s 16-18.25
  samples (display) / 46 % (Xvfb) and it sheds 0.67 m/s per two samples against
  0.48 coasting. The car arrives at the plan's apex on the plan's speed.
* **It then cuts too far inside.** At s 18.75 the car is **+0.177 m inside** an
  R 0.62 m arc, i.e. it is trying to drive an actual radius of ~0.45 m. The
  minimum turn circle is **0.561 m**. It cannot hold that, so it saturates:
  rack p90 goes 0.319 → 0.455 → **0.523 rad (full lock)** over s 18.75-19.5 and
  stays there to s 20.0, and the truth yaw rate sits at **2.63-2.91 p50 /
  3.04 p90** — exactly the measured cliff.
* **The whole loss is the unwind.** s 18.75-19.0 *gains* 0.078 s (the shortcut
  is genuinely shorter), then s 19.25-20.5 loses **+0.364 s** with the car
  0.24-0.33 m **wide** at full lock and 0.8-1.1 m/s under plan. The real apex
  (slowest point, 1.47 m/s) is at **s 19.75 — a metre past the plan's 18.79**.
* Xvfb is the same shape one notch worse: full lock from s 19.0, 0.32 m wide,
  loss +0.333.

## 3. HP2 (s 42.5-45.5): identical mechanism, half the amplitude

Display, per 0.25 m:

```
   s   kappa  v_plan v_drv   dv | loss    cum  | thr==0 off(+=inside) rack_p90 yaw_p90 a_drv
 41.75  0.24   3.31   3.33  +0.02| -0.002 +0.046|  89%   +0.104      0.181   1.68   -5.36
 42.00  0.37   2.98   2.85  -0.13| +0.008 +0.054|  80%   +0.147      0.196   1.55   -6.18
 42.25  0.52   2.73   2.43  -0.30| +0.010 +0.064|   6%   +0.196      0.199   1.44   -1.00
 42.75  0.79   2.22   2.15  -0.07| -0.011 +0.046|  14%   +0.271      0.239   1.76   -3.67
 43.00  1.01   1.76   1.76  +0.00| -0.044 +0.001|   8%   +0.281      0.365   2.08   -3.13
 43.25  1.74   1.61   1.61  +0.00| -0.118 -0.117|   0%   +0.259      0.424   2.22   -1.89
 43.50  1.41   1.97   1.49  -0.48| +0.019 -0.097|   0%   +0.179      0.500   2.84   +1.00
 43.75  0.77   2.24   1.73  -0.50| +0.093 -0.004|   0%   -0.063      0.521   3.09   +0.41
 44.00  0.35   2.55   1.70  -0.85| +0.049 +0.046|   0%   -0.212      0.521   2.66   +0.13
 44.25  0.05   2.95   1.90  -1.05| ~+0.026      |   0%   -0.251      0.305   1.58   +4.29
 44.50  0.00   3.41   2.45  -0.96| +0.024      |   0%   -0.237      0.153   1.13   +5.70
 45.00  0.08   4.16   3.43  -0.73| +0.013      |   0%   -0.140      0.049   0.32   +6.53
```

Total over s 40.0-47.25 ≈ **+0.16 s/lap** display, **+0.25** Xvfb (the
s 44.25-44.5 bin straddles the lap counter; its loss is interpolated, every
other bin is a clean 100-passage median). Same story: the car is up to **0.28 m
inside** at s 43.0-43.25, goes to full lock at s 43.5, then unwinds to 0.25 m
**wide** by s 44.25 with the apex a metre late (slowest 1.70 m/s at s 44.0
against the plan's 1.57 at 43.28).

## 4. Does the brake fire inside the turn? Yes at HP2, and it hurts

`throttle == 0.0` means locked wheels (`VehicleController.cs`), and a locked
wheel carries no lateral force.

| window | display | Xvfb | median rack there |
|---|---|---|---|
| hp1 entry s 16.0-18.25 | 35.9 % | 45.8 % | 0.038-0.039 rad |
| **hp1 turn s 18.25-20.25** | **5.6 %** | **16.6 %** | 0.098 / 0.223 rad |
| hp2 entry s 40.5-42.25 | 33.1 % | 36.5 % | 0.084-0.177 rad |
| **hp2 turn s 42.25-44.5** | **3.5 %** | **11.0 %** | 0.194 / 0.274 rad |

Effect, median change over the next two samples, brake samples vs coasting
samples in the same window:

| window | d|cte| brake | d|cte| coast | d|yaw rate| brake | coast |
|---|---|---|---|---|---|
| hp1 turn, display | **+0.0585 m** | +0.0150 | +0.110 | −0.076 |
| hp1 turn, Xvfb | **+0.0417 m** | +0.0122 | +0.170 | −0.199 |
| hp2 entry, display | **+0.0596 m** | +0.0007 | −0.122 | +0.152 |
| hp2 entry, Xvfb | **+0.0632 m** | +0.0417 | +0.136 | +0.252 |

So a brake pulse inside the turn costs **3-4x** the cross-track drift of a
coasting sample. It is a real, measured cost — but on the display it is only
3.5-5.6 % of turn samples, so it is a second-order effect there; at 0.096 s it
reaches 11-17 % and matters more. **Peak offenders: s 41.75-42.00 (89 % / 80 %
of samples braking on the display) and s 42.75 (58 % on Xvfb), i.e. the brake
into hp2 is still running at kappa 0.24-0.79.**

## 5. Plan-limited or tyre-limited, per quarter metre?

* s 15.5-18.5 (both hairpins' approaches): **plan-limited**. The car is faster
  than the plan and the brake is what brings it down.
* s 18.75-19.5 / 43.25-43.75: **tyre-limited and saturated**. Rack at
  0.50-0.523 rad (full lock, 0.5236 max), truth yaw rate 2.6-3.1 rad/s, i.e. at
  or above the 2.71 cliff. The controller has no authority left.
* s 19.75-20.5 / 44.0-44.75: **still tyre-limited** — rack 0.52 → 0.22 while the
  car is 0.25-0.33 m wide; it is spending its remaining grip on getting back to
  the line, not on speed.
* s 20.5 onward / 45.0 onward: **plan-limited again**, but starting from
  0.5-1.1 m/s down.

## 6. Exit acceleration is NOT the deficit

Measured median longitudinal acceleration against the plan's own `ax` column:

| s | plan ax | driven (display) | driven (Xvfb) |
|---|---|---|---|
| 19.99-20.49 | 3.2-4.1 | +3.99 … +5.38 | +3.21 … +4.48 |
| 20.5-21.0 | 4.3-5.1 | +4.70 … +5.38 | +4.86 … +5.08 |
| 44.75-45.5 | 5.0-6.1 | +6.06 … +6.53 | +5.48 … +6.22 |
| 0.0-1.0 (hp2 exit) | 5.9-6.1 | +5.9 (v1 table) | +5.9 |

**The car delivers the planned exit acceleration, and at hp2 exceeds it
(6.5 m/s²).** The plan's ramp out of hp1 is only 3.2-5.1 m/s² anyway, not 6.5 —
6.5 appears only once the line is straight. So a steeper exit ramp buys
nothing: the exit is slow because it *starts* 1.0-1.1 m/s down, not because the
car cannot accelerate.

---

## 7. Four candidate profiles on the same geometry

Built with `scripts/hairpin_profile.py` (yaw-rate cap / flat lock phase /
advanced brake / exit release, then the same `limit_long_accel` reachability
pass at ax 6.5, brake 4.6 that `scripts/partial_blend.py` uses). Windows are
restricted so s 24-27 and s 36-40 never move.

| line | what it changes | points moved | ideal |
|---|---|---|---|
| shipped `_race` | — | — | **10.282** |
| **`_hpA_race`** | `--cap 2.55`: v ≤ 2.55/kappa in s 18.2-20.2 and 42.7-44.5 | **13 / 444**, s 18.8-19.5 and 43.3-43.7; max cut 0.39 m/s | **10.373** (+0.092) |
| `_hpB_race` | hpA + `--flat 1.20`: hold v at the zone minimum wherever kappa ≥ 1.2 (no acceleration while the rack is at lock) | 15 / 444, same s; max cut 0.67 m/s | 10.447 (+0.166) |
| `_hpC_race` | hpA + `--advance 0.3`: v(s) ← min(v(s), v(s+0.3)) on the approaches, so the planned brake finishes 0.3 m earlier and is over before turn-in | 114 / 444, **s 10.7-19.5 and 41.2-43.7** | 10.633 (+0.352) |
| `_hpD_race` | hpB + `--release` to the ay-9.0 profile on the exits (s 20.2-21.5, 44.2-2.0): spend the corner saving on the exit | 50 / 444; max raise **+0.28 m/s**, max cut 0.67 | 10.406 (+0.124) |

What hpA does concretely at hp1: 1.69/1.77/1.92 → **1.57** at s 18.79-18.99 and
2.06/2.12/2.17/2.21 → 1.71/1.77/1.96/2.08 at s 19.09-19.49; asked yaw rate
falls from 2.74-3.12 to a flat **2.55**. At hp2: 1.57/1.75/1.95 → 1.45/1.45/1.56
at s 43.28-43.58, asked yaw 2.77-3.19 → 2.55.

### Surrogate verdicts

`tools/surrogate_race.py --laps 60 --seeds 3` (= 180 laps/cell), shipping flags
plus `-r barrier_margin=0.34 -r servo_period_max=0.100 -r model_cmd_tau=1.0
-r model_cmd_tau_period_min=0.080`.

**(a) Default plant** (the validated one — and note it has **no yaw-rate cliff**:
`--plant-breakaway` defaults to 0, so this plant charges each candidate its
ideal-time cost and gives it no credit for the mechanism it is built to fix):

| line | 0.058 s | 0.098 s |
|---|---|---|
| shipped | 0 coll, **11.010** | 0 coll, **11.570** |
| hpA | 0 coll, 11.040 (+0.03) | 0 coll, **11.560 (−0.01)** |
| hpB | 0 coll, 11.050 (+0.04) | 0 coll, 11.630 (+0.06) |
| hpC | 0 coll, 11.200 (+0.19) | 0 coll, 11.860 (+0.29) |
| hpD | 0 coll, 11.040 (+0.03) | 0 coll, 11.590 (+0.02) |

Clean sheets prove nothing (the harness has zero false collides but many false
cleans). The useful reading is that hpA gives back essentially all of its
0.092 s of ideal cost even on a plant that models none of its benefit.

**(b) Breakaway plant** (`--plant-breakaway 1 --plant-r-crit 2.71` — the sysid
cliff; it is over-pessimistic, the **shipped line collides on it too** while
being 100/0 live at both periods, so its collisions are not a veto, only its
ranking is informative):

| line | 0.058 s | 0.098 s |
|---|---|---|
| shipped | **2 coll**, 11.680 | 1 coll, 12.030 |
| hpA | 1 coll, 11.660 | 2 coll, 12.070 |
| hpB | 1 coll, 11.550 | **0 coll**, 12.080 |
| hpD | 1 coll, **11.520** | **0 coll**, 12.040 |

Every collision on this plant, for every line including the shipped one, is at
**s 0.3-0.8 — the hp2 exit**, which is the same corner the live failures of
`fin_exm10`, `fin_e18a10` and `fin_ay90` hit.

### Notes on the losers

* **hpC is out.** `--advance` propagates backwards through the reachability
  pass: a 0.3 m shift over a 1.2 m approach window drags the plan down from
  s 10.7, costing 0.35 s of ideal for a mechanism worth at most a few
  hundredths on the display. If the brake-in-turn effect is to be attacked it
  should be attacked with `thr_brake_lead_s` in the controller, not with the
  profile.
* **hpD's risk is named.** Its release raises s 20.2-21.5 and 44.2-2.0 by up to
  0.28 m/s toward the ay-9.0 profile, and *those exact exits* are where
  `fin_exm10_x32` (9 collisions, s 44.1-44.3), `fin_e18a10_disp` (4, s 44.35-44.55,
  at full lock) and `fin_ay90_disp` (4, s 19.9-20.2) failed. A variant that
  moved the release windows later (20.4 / 44.8) was **worse** on the breakaway
  plant (3 collisions at 0.098 against 0), and a variant that released only back
  to the shipped speeds is byte-identical to hpB. So hpD is the high-upside /
  high-risk rung, not the first thing to drive.

## 8. Ranking, and what to drive first

1. **`finals_mesh_apex_f42k_hpA_race.csv` — drive this first.** It is the
   minimum honest test of the yaw-rate hypothesis: **13 of 444 points move**,
   nothing is ever raised, no corner outside the two hairpin apexes is touched,
   it costs 0.092 s of ideal, and both surrogate plants say that cost comes
   back. If the breakaway story is right it should recover a visible slice of
   the measured +0.281 / +0.333 s of hp1 loss; if the apex speed was never the
   binding thing, it will read as a ~0.03 s/lap loss and the story is closed
   cheaply. Watch: rack p90 and truth yaw rate at s 19.0-19.5 (should drop off
   the 0.523 rad rail and under 2.7 rad/s), and the offset at s 19.5-20.0
   (−0.30 m today).
2. **`_hpB_race`** — the stronger dose (also removes the plan's
   mid-corner acceleration, which is what asks for 3.12 rad/s at s 18.99).
   Drive it if hpA moves the corner but under-delivers.
3. **`_hpD_race`** — only after hpA/hpB are clean and faster, and only with the
   hp2 exit watched; it re-opens a corner with three prior live failures.
4. **`_hpC_race`** — kept for the record, not recommended.

Gate rule unchanged: 100 laps at both periods, the display is the gate, and no
run is evidence without its `dt_p50`.
