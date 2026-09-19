# The display cut at hairpin 2 under `model_cmd_tau` (Sep 20, agent)

Why `model_cmd_tau 1.0` — the shipped 0.10 s command filter modelled inside the
mpc3 rollout — is −0.19 s/lap at the organisers' period and 10 collisions in 41
display laps, all at s 42.8-42.9. Everything below is measured from the driven
logs (`data/logs/`), the map and the harness; nothing is estimated unless it
says so.

Logs used: `race_20260919_192247` (mct, display, 41 laps / 10 collisions,
dt_p50 **0.0583**), `race_20260919_170814` (v8, display, 100 / 0, 0.0583),
`race_20260919_190223` (mct, Xvfb, 100 / 0, 0.0964), `race_20260919_121531`
(v8-config, Xvfb, 100 / 0), `race_20260919_193350` (v9 gate, display, 32 / 0).
Line `finals_mesh_apex_f42k_race.csv`, map `finals_mesh`.

## 1. The cut is a real vehicle path, not a localization error

All ten collisions are the same event: s 42.79-42.88, truth offset **+0.376 to
+0.380 m INSIDE** the line (one outlier at +0.323), truth speed 1.45-1.66 m/s,
and the controller's own `cte` at that moment **+0.32 to +0.36** — i.e. the
estimator reports the cut it is making, to within 2-4 cm.

Estimate minus truth, with truth put into the map frame by a Procrustes fit over
every clean sample of the run (fit residual p50 0.082-0.091 m over the whole
lap, rotation +0.35 deg, translation 3.5 cm — the same transform in all four
runs), truth interpolated at `pstamp`, decomposed in the line frame:

| run | whole lap lat p50 / \|p95\| | s 41-43.5 lat p50 / \|p95\| | s 41-43.5 yaw p50 / \|p95\| |
|---|---|---|---|
| mct display 0.058 | −0.004 / 0.084 | **−0.024 / 0.094** | −1.39 / 2.29 deg |
| v8 display 0.058 | −0.003 / 0.090 | −0.022 / 0.083 | −1.32 / 2.23 deg |
| mct Xvfb 0.096 | −0.004 / 0.090 | −0.024 / 0.075 | −1.45 / 2.44 deg |
| v9 display 0.058 | −0.004 / 0.092 | −0.030 / 0.100 | −1.31 / 2.30 deg |

The localization error at the hairpin-2 entry is **the same at both periods**
(lateral bias 2-3 cm toward the outside, p95 under 0.10 m, heading bias 1.4 deg)
and is an order of magnitude smaller than the 0.38 m cut. It is not the variable.

## 2. What actually differs — and it is not "earlier on the display"

Median profiles through the entry, by run (offset + = inside; heading = truth
minus line heading):

| s | off v8 disp | off mct disp (clean) | off mct disp (hit) | off v8 Xvfb | off mct Xvfb | Δmct disp | Δmct Xvfb |
|---|---|---|---|---|---|---|---|
| 42.0 | 0.122 | 0.146 | 0.189 | 0.066 | 0.126 | +0.024 | +0.060 |
| 42.5 | 0.222 | 0.264 | 0.335 | 0.122 | 0.198 | +0.042 | +0.075 |
| 43.0 | 0.275 | 0.331 | — | 0.147 | 0.232 | +0.056 | +0.085 |
| 43.5 | 0.244 | 0.309 | — | 0.102 | 0.188 | +0.065 | +0.087 |

heading at s 41.5: v8 disp 3.05, mct disp 8.75 (hit laps 10.94), v8 Xvfb −0.05,
mct Xvfb 6.07 deg — **Δ +5.70 deg on the display, +6.12 deg in band.**

So modelling the filter does **not** turn the car in earlier on the display
specifically: it adds the same 2-3.5 deg of yaw and, if anything, slightly *less*
inside offset there (+0.056 m at s 43.0) than in band (+0.085 m). What is
period-specific is the **baseline**: at the same corner, in every config, the
display runs **0.10-0.14 m further inside and ~5.5 deg more rotated** than Xvfb
(v8: 0.275 vs 0.147 at s 43.0). The display simply has no margin left to spend.

The budget, from the per-lap maxima of the inside excursion over s 41.5-42.9:
v8 display p50 0.266, p90 0.291, **max 0.311**; mct display p50 0.324 (clean
laps), and the wall bites at **0.36-0.38**. The corridor here is 0.675-0.750 m
to the inside wall at s 42.0-42.5, and a car at +0.38 m with 16-19 deg of yaw
puts its front-left corner at ~0.58-0.60 m plus body; that is the contact.

Within the mct run the hit laps are the early ones: rack first exceeds 0.10 rad
at s 41.49 (hit) against 41.79 (clean) and v8's 41.59, and `cmd` at s 41.0 is
+0.121 rad (hit) against +0.058 (clean) and +0.073 (v8). The distribution has a
0.4 m spread and only its early tail collides.

## 3. The lag arithmetic — where the constant comes from

Constants: `step_dt` 0.060, `delay_s` 0.180 fixed (`delay` is not in
`RATE_TERMS`) so `nd` = 3; `apply_ahead_s` = 0.090·T/0.104, floored at one step;
`Actuator.dead_time` 0.135; `steer_tau` 0.04; `cmd_tau` 0.10; `RATE_NOMINAL`
0.104.

| term | T = 0.058 | T = 0.096 |
|---|---|---|
| `apply_ahead_s` | 0.0505 s = 0.84 step -> **floored to 1.00** | 0.0834 s = 1.39 step |
| readout knot `_f·dt` | 0.240 s | 0.263 s |
| model's command -> rack lag (`steer_tau` only) | 0.040 | 0.040 |
| ... with `model_cmd_tau 1.0` | 0.140 | 0.140 |
| TRUE node filter ramp lag `T(1−a)/a`, `a = 1−e^{−T/τ}` | 0.0737 | 0.0594 |
| TRUE publish -> rack (identified `0.056 + 0.5T`) | 0.0852 | 0.1042 |
| TRUE total solver-decision -> rack | **0.1589** | **0.1636** |
| net anticipation without the model | +0.121 s = **2.08 frames** | +0.140 s = 1.45 frames |
| net anticipation with `model_cmd_tau 1.0` | +0.221 s = **3.79 frames** | +0.240 s = 2.49 frames |

Two facts fall out:

1. **The true chain is period-invariant, by cancellation.** The ZOH-sampled
   filter lags a ramp by `τ − T/2` (exactly `T(1−a)/a`), and the identified
   transport carries `+0.5T`; the halves cancel, leaving `τ + 0.056 ≈ 0.16 s`
   at both periods. So the model's constant 0.175 s (without the modelled
   filter) is a good match — +0.016 s at 0.058, +0.011 s at 0.096 — which is
   why v8 is clean at both. Turning the model on makes the model's own chain
   0.275 s against a true 0.16 s.
2. **The double-count is real but is bigger in band, so it is not the
   explanation.** The rollout models a continuous 0.10 s lag where the real
   filter's ramp lag is `τ − T/2`; the missing half-frame is already inside the
   `0.5T` of the publish->rack chain. That over-compensation is `R·T/2` =
   0.029 s of ramp at 0.058 and 0.048 at 0.096. Wrong direction.

What *is* wrong with the period is the **unit**: the lead this term adds is a
constant in SECONDS (`cmd_tau` of command ramp: the published value is read from
the pre-filter plan `dc`, which leads the rollout's rack plan `d` by
`τ_c + τ_a` on a ramp), while every other lead here is a constant in FRAMES —
`apply_ahead_s` is rate-scaled, and at 0.058 s it is additionally floored at one
step. Adding 0.10 s is **+1.72 frames at the display against +1.04 in band**,
onto a loop that already anticipates 2.08 frames there against 1.45. That is
the term that does not scale, and it lands on the one corner where the display
has 5 cm of margin.

## 4. Candidates, screened in the harness

`tools/surrogate_race.py`, 102 laps (3 seeds x 34) at `--frame 0.058`, v9
arguments: every cell is **0 collisions** and within 0.07 s of median lap
(`mct 0` 11.030, `mct 1.0` 11.030, `mct 0.6` 10.990, `mct 1.0 + steer_tau_k 1.0
ref 0.095` 10.960). The harness cannot fail this corner — as recorded, its clean
sheets are not evidence.

It does, however, **reproduce the traces**. 12 laps at 0.058 s, median inside
error through the entry (`e` from `--dump-rec`), against the live medians:

| arm | e at s 43.0 | s 43.5 | Δ vs no model | live counterpart |
|---|---|---|---|---|
| `model_cmd_tau 0` (v8) | 0.285 | 0.297 | — | live v8 0.275 / 0.244 |
| `model_cmd_tau 1.0` | 0.352 | 0.365 | **+0.067** | live mct 0.331 / 0.309 (+0.056) |
| `model_cmd_tau 0.6` | 0.322 | 0.329 | +0.037 | not driven |
| `model_cmd_tau 1.0` + barrier 0.44 over s 41.2-43.4 | 0.323 | 0.333 | +0.038 | not driven |
| `model_cmd_tau 0.6` + the same barrier window | 0.316 | 0.323 | +0.031 | not driven |

The harness's level (0.285) and its mct delta (+0.067) match the live display
numbers (0.275, +0.056) to within 1 cm, so the peak inside error over s 42-43.5
is a usable screening metric even though the collision itself never appears.
Both partial arms remove about half of the added excursion, and they are **not
additive** — together they give +0.031 m against +0.037 / +0.038 alone, so the
remaining 3 cm is the model-free part of the display's own bias, not something
either knob reaches.

`model_cmd_tau` is now a FRACTION of `cmd_tau` (1.0 and 0.0 are byte-identical
to before), and the period gate has two new inert knobs,
`model_cmd_tau_fade` (0.004) and `model_cmd_tau_floor` (0.0 = the committed v9
gate: fade to nothing below `model_cmd_tau_period_min`). A floor keeps the full
model in band and a fraction of it on the display.

## 5. What to drive, in order (32 display laps each)

1. **`-p model_cmd_tau:=1.0 -p model_cmd_tau_period_min:=0.080 -p
   model_cmd_tau_floor:=0.60`** — in band byte-identical to v9/mct (the gate is
   1.0 at and above 0.080 s), on the display the rollout models 0.060 s of the
   filter, the frame-matched share (0.058/0.096 = 0.61). Expect ~60 % of the
   0.14 s/lap display gain (10.70 median against v8's 10.84) and about half of
   the 0.06 m extra cut. Fail-safe direction: lower the floor, not raise it.
2. **`-p model_cmd_tau:=1.0 -p model_cmd_tau_period_min:=0.0 -p
   param_schedule:=/home/data/racelines/finals_mesh_apex_f42k_hp2.json`**
   (`barrier_margin` 0.34 -> 0.44 over s 41.2-43.4, 0.8 m blend; the file is in
   `data/racelines/`). This keeps the whole lag model at both periods — the
   10.55 s laps — and buys the margin back where it is spent. The corridor
   allows it: 0.44 + 0.135 of body = 0.575 against 0.675 m of inside wall at
   s 42.5. Harness: same −0.038 m of cut as arm 1, and the in-band cost of the
   window is **+0.02 s/lap** (102 laps at 0.096 s: 11.580 with it, 11.560
   without, 0 collisions both) — i.e. it keeps the organisers'-period gain.
3. **`-p model_cmd_tau:=0.60 -p model_cmd_tau_period_min:=0.0 -p
   param_schedule:=...hp2.json`** — both together, for when arm 1 or 2 alone is
   still inside v8's own p90. Gives up the most pace of the three.

Rejected as arm candidates: `steer_tau_k 1.0 / steer_tau_ref 0.095` (it trims
0.037 s of the lead at the display and nothing in band, but it does it by
telling the compensator the rack is instantaneous, against the identified
`steer_tau_k = −0.5`, and it moves `Actuator` as well as the rollout);
`dead_tick_c` / `delay_tick` (they move `nd` and therefore every corner);
`lead_mode` variants (measured at 198-210 collisions, `mpc3_core` DEFAULTS).

**Do not read a harness clean sheet as a pass.** The gate is 32 display laps,
then 100, with `dt_p50` recorded.
