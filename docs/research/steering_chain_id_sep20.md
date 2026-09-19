# The publish -> rack steering chain, identified at both sensor periods

Sep 20, agent. Offline only; no simulator or container was touched.

Tool added: `tools/steer_chain_id.py` (joint dead-time / first-order-lag /
slew fit with a free static gain, evaluated on a mask so the same trace can be
fitted per regime). Scratch estimators used alongside it: a per-lap, per-corner
pure-shift alignment and an exact "is the rack the command of k rows ago"
count. Every number below is from a driven log.

Logs used (all `finals_mesh_apex_f42k_race` family, IROS finals track):

| period | log | what it is |
|---|---|---|
| 0.058 | `race_20260919_170814` | v8, display, 100 laps / 0 |
| 0.058 | `race_20260919_193350` | v9, display, 32 / 0 |
| 0.058 | `race_20260919_192247` | v8 + ungated `model_cmd_tau`, display, 41 / **10** |
| 0.058 | `race_20260918_162222` | frozen finals config, display, 32 / 0 |
| 0.095-0.098 | `race_20260919_121531` | v8, Xvfb, 100 / 0 |
| 0.096 | `race_20260919_190223` | v9 (`model_cmd_tau`), Xvfb, 100 / 0 |
| 0.098 | `race_20260918_163645` | frozen finals config, Xvfb, 32 / 0 |
| 0.096 | `race_20260919_204700` | open-hairpin `f42i`, Xvfb, 32 / 2 |

---

## 1. The identified chain

### 1.1 The slew limit is inert. The chain is linear.

Rack slope over whole logs: p99 **2.04 rad/s** at 0.058 and **1.68 rad/s** at
0.096, against the 3.2 rad/s rate limit; the published command's own slope has
the same p99 (2.00 / 1.63), i.e. **the rack is never rate-limited, it is
following a command that `cmd_tau` + `w_dcmd` have already slowed below the
limit.** Samples above 3.0 rad/s are 0.2 % at 0.058 and 0.03 % at 0.096, and
they sit on rows with a short timestamp gap (measurement, not plant).

Consequence: the 3.2 rad/s clamp in `Actuator.advance` and in the rollout is
not doing anything on the shipping tune, and the "0.14 rad behind the command"
at the open-hairpin turn-in is **pure transport lag, not saturation**.

Static gain rack/cmd is 0.99-1.00 wherever the command is large
(`|cmd| >= 0.30`), and 0.88-0.99 on the straights, where the errors-in-
variables of the flutter attenuate it. There is no steady-state gain error.

### 1.2 The effective lag: `0.056 + 0.50 * T`, confirmed by a third estimator

Per-lap pure-shift alignment of the `cmd` column onto the `steering` column,
by corner window, median over laps (IQR in brackets):

| window | 0.058 s (`_170814` / `_193350`) | 0.096 s (`_190223` / `_121531`) |
|---|---|---|
| hairpin 1 entry, s 17.6-19.6 | **0.084** [0.082, 0.086] / 0.086 | **0.104** [0.102, 0.110] / 0.106 |
| hairpin 1 exit, s 19.6-21.5 | 0.038 / 0.038 | 0.083 / 0.080 |
| hairpin 2 entry, s 41.8-43.6 | 0.068 / 0.068 | 0.073 / 0.072 |
| s 23-28 (mid corner) | 0.088 / 0.088 | 0.108 / 0.104 |
| straight, s 5-12 | **0.058** = 1.00 T | **0.096 / 0.088** = 1.00 / 0.93 T |

The turn-in and mid-corner windows land on **0.085 s at T = 0.058 and 0.105 s
at T = 0.096**, reproducing `tools/steer_residual_id.py`'s `0.056 + 0.50 * T`
to 0.001 s on a completely different estimator (per-lap shift alignment instead
of a global lag fit), on a different line, on different logs.

### 1.3 The decomposition, and the discrete structure

Joint (dead, tau) fit with the gain free, restricted to the turn-in window
s 17.4-19.8 (`tools/steer_chain_id.py --line ... --s-window 17.4,19.8`):

| log | T | Td | tau | Td+tau | gain | rms |
|---|---|---|---|---|---|---|
| `_193350` | 0.058 | 0.052 | 0.026 | 0.078 | 0.990 | 0.0161 |
| `_170814` | 0.058 | 0.050 | 0.028 | 0.078 | 0.992 | 0.0172 |
| `_190223` | 0.096 | 0.068 | 0.018 | 0.086 | 0.998 | 0.0213 |
| `_121531` | 0.095 | 0.060 | 0.026 | 0.086 | 1.000 | 0.0183 |

The RMS valley is flat along `Td + tau`, so the split is soft and the **total**
is the reliable quantity. The two estimators bracket it at
**0.078-0.085 s at 0.058** and **0.086-0.105 s at 0.096**. Take the shift
number (`0.056 + 0.50 T`) as the effective lag and `(Td, tau) ~ (0.05, 0.027)`
at 0.058 and `(0.064, 0.022)` at 0.096 as the shape.

Discrete structure, assumption-free: the fraction of rows where the rack
equals the command of *k* rows ago to within 0.001 rad —

| log | k = 1 | k = 2 | k = 1, slow cmd | k = 1, fast ramp |
|---|---|---|---|---|
| `_170814` (0.058) | 0.634 | 0.114 | 0.624 | 0.588 |
| `_190223` (0.096) | **0.888** | 0.074 | 0.910 | 0.652 |

So the chain is **one telemetry frame of latch** (exactly, most of the time)
**plus a residual that only shows when the command ramps**. At 0.096 the
latch accounts for 89 % of all rows on its own; at 0.058 only 63 %, which is
the same statement as `0.5 * T + 0.056` being 1.47 frames at 0.058 and 1.08
frames at 0.096. The physical reading is a ZOH pickup at the sim side (average
half a period of wait) on top of a fixed 0.056 s of WebSocket + Unity +
actuator + telemetry return.

### 1.4 Is the chain the same small and large?

**No, and the difference is in the direction that hurts.** In seconds, the
straight runs at exactly one period (0.058 / 0.096 — i.e. the fixed 0.056 s
costs nothing there) while a hairpin turn-in runs at 0.085 / 0.105. The excess
of turn-in over straight is **+0.027 s at 0.058 and +0.009 s at 0.096**. The
fitted `tau` behaves the same way: 0.006-0.012 on the `|cmd| < 0.10` band
against 0.028 at `|cmd| >= 0.30` (whole-log fits, all four logs).

A lag identified on straights therefore **understates the hairpin lag by half a
frame at the display period**, which is exactly where every display collision
of this campaign has been.

---

## 2. What the solver assumes, against what is there

Two separate modelled lags live in this stack and they do not agree with each
other:

1. **the actuator model** — `Actuator(dead_time=cmd_dead_time, steer_tau)`,
   used to predict the rack at the solve instant and to build the `nd`
   committed steps, and mirrored by the rollout's `tau_a`. Shipping:
   `0.135 + 0.04 = 0.175 s, constant` (`delay_tick` is not in `rate_terms`).
2. **the readout lead** — the published angle is read at plan index
   `nd + max(apply_floor_steps, apply_ahead_s/dt)` = `3 + 1.000` at T = 0.058
   (the floor binds) and `3 + 1.384` at T = 0.096, i.e. **0.240 s / 0.263 s**
   of plan time, 4.14 frames / 2.74 frames.

And one real lag that until Sep 20 was in **neither**: the node's own
`cmd_tau` 0.10 output filter, which sits between the plan readout and the
published command.

### The arithmetic that matters

The honest quantity is the lag from *the plan knot the angle is read at* to
*the rack*, because that is what the solver is implicitly compensating:

```
true   =  cmd_tau (0.100, in the node)  +  0.056 + 0.50 T
       =  0.185 s at T = 0.058          =  0.205 s at T = 0.098
```

| configuration | modelled | at 0.058 | at 0.098 |
|---|---|---|---|
| v8 (`steer_tau` 0.04, no filter model) | `0.135 + 0.040` | 0.175, **0.010 too FAST** | 0.175, **0.030 too FAST** |
| v9 in band (`model_cmd_tau` 1.0) | `0.135 + 0.100 + 0.040` | 0.275, **0.090 too SLOW** | 0.275, **0.070 too SLOW** |
| chain-matched (below) | `0.016 + 0.50 T + 0.100 + 0.040` | 0.185, exact | 0.205, exact |

**This answers the question the run log posed.** The v8 model was not too
slow — it was marginally too FAST, and by 3x more at the organisers' band
(0.030 s) than on the display (0.010 s), because the modelled lag is a
constant while the true one grows with the period. That is the whole reason
`model_cmd_tau` is +0.19 s/lap at 0.095 and a DQ at 0.058: adding the filter
to the model adds a flat 0.100 s of anticipation, which at 0.098 covers a
0.030 s shortfall (and fixes the *shape*, see below) and at 0.058 covers a
0.010 s shortfall and over-anticipates by 0.090, which is what cuts hairpin
2's entry 0.38 m inside.

### The shape, not just the phase

The lag columns above understate what `model_cmd_tau` does. In v8 the rollout
has no stage at all between the plan's command and the plan's rack except
`steer_tau` 0.04, so the solver plans a rack ramp it cannot buy: whatever it
publishes is low-passed by 0.10 s before it leaves the node, so the real rack
ramp starts later **and shallower** than planned. With `model_cmd_tau` the
plan pre-emphasises (`dc` is raised early so that `fl` — the post-filter
command — has the wanted shape) and the publish is read from the PRE-filter
plan, so the published command itself carries the inverse filter. That is a
genuine correction and it is why it is worth 0.19 s/lap in band — not the
extra phase, which by the table above is 0.070 s more than the period needs.

**The right model at 0.058** is therefore: keep the filter modelled (the shape
is wrong at every period) and take the 0.090 s of surplus phase back out of
the dead time, so that the modelled total lands on 0.185 s. That is
`cmd_dead_time` 0.016 + 0.50 T, i.e. the `delay` rate term with a
`cmd_dead_time` default of **0.068** (`RateAdapter.bridge = 0.068 - 0.5 *
0.104 = 0.016`).

Checks on that set: `delay_s = cmd_dead_time + est_lag` with
`est_lag = 0.180 - 0.068 = 0.112`, giving 0.157 s at 0.058 and 0.177 s at
0.098, so `nd = round(delay_s / 0.060) = 3` at **every** period in the band —
the committed-step count that made `nd_mode:='period'` collide does not move.
Add `-p nd_delay_s:=0.180` to pin it anyway; it is byte-identical at 0.058 and
0.098 and only guards a period under ~0.050 s.

---

## 3. Harness validation

`tools/surrogate_race.py --line finals_mesh_apex_f42k_race`, 3 seeds x 16 laps
= **48 laps per cell**, with the v9 flags
(`barrier_margin=0.34 servo_period_max=0.100 model_cmd_tau=1.0
model_cmd_tau_period_min=0.080 straight_pp_k=0.5 straight_d=3.0
straight_kappa=0.10 straight_pp_period_max=0.075`) plus the shipping set.

The harness plant is the right one to test this against: `--plant-fit` already
latches the command at a frame boundary and adds `FIT_DEAD = 0.056`, i.e. it
*is* `0.056 + 0.50 T`, identified from the same logs.

Every cell below is **clean — 0 collisions**, at both periods. Ranked by the
modelled total lag of section 2 (`cmd_dead_time_eff + modelled cmd_tau +
steer_tau_eff`), against the true 0.185 s (0.058) / 0.205 s (0.098):

| cell | modelled total | 0.058 s | 0.098 s |
|---|---|---|---|
| gated + `steer_tau=0.0` (filter NOT modelled at 0.058) | 0.135 | 96/0, 11.290 (+0.30) | — |
| **v9 baseline** (gated at 0.058, `mct` on in band) | 0.175 / 0.275 | 96/0, **10.990** | 96/0, **11.570** |
| `mct` ungated, nothing else | 0.275 | 48/0, 11.040 (+0.03) | = baseline |
| **ungated `mct` + `steer_tau=0.0`** | 0.235 | 96/0, **10.950 (-0.04)** | 96/0, **11.550 (-0.02)** |
| ungated `mct` + `cmd_dead_time=0.105` + `delay` term | 0.222 / 0.242 | 48/0, 11.130 (+0.14) | 48/0, 11.560 (-0.01) |
| chain-matched: ungated `mct` + `cmd_dead_time=0.068` + `delay` | **0.185 / 0.205, exact** | 48/0, 11.180 (+0.19) | 48/0, **12.080 (+0.51)** |
| the same with the gate kept (isolates the dead time at 0.058) | 0.085 + 0.040 | 48/0, 11.140 (+0.15) | = chain-matched |

Three things fall out of that table.

1. **The two lag stages are substitutes, and the total is what the car feels.**
   `steer_tau=0.0` is +0.30 s/lap when the filter is not modelled (total 0.135,
   0.050 too fast) and **-0.04 s/lap when it is** (total 0.235, 0.050 too
   slow). Same knob, opposite sign, and the sign flips exactly where section
   2's arithmetic says it should.
2. **The best cell of the whole sweep is `model_cmd_tau` ungated with
   `steer_tau` 0.0** — fastest at 0.058 *and* at 0.098, 96 clean laps each.
   It is also the closest of the tested sets to the identified chain that does
   not go through `cmd_dead_time`.
3. **Matching the chain exactly, through `cmd_dead_time`, is clean but
   slower** — and the cost is not proportional to the size of the correction
   (+0.14 at 0.058 for `0.105`, +0.15 for `0.068` with the gate kept), which
   says most of it is not the steering model at all: `mpc3.py` also hands
   `cmd_dead_time` to the **speed servo** (`tau=v['cmd_dead_time']`) and to
   `self._bridge`. At 0.098 the cost *is* proportional (-0.01 at `0.105`,
   +0.51 at `0.068`), so there the steering model is genuinely over-corrected.
   **`cmd_dead_time` is not a steering-only knob; do not use it as one.**

### What the harness cannot decide

`model_cmd_tau` ungated at 0.058 — the configuration that took **10 collisions
in 41 live display laps** (`race_20260919_192247`, all at s 42.8, +0.38 m
inside) — is **48/0 clean here**. The harness does not reproduce the display
failure, exactly as `docs/FINALS_TUNING.md` says of every display arm. So the
table above ranks pace and vetoes nothing; the display is still the gate.

---

## 4. What this closes and what it leaves open

* **Closed:** the 3.2 rad/s slew limit is not a constraint on the shipping
  tune at either period; do not look for saturation in a rack trace again.
  The static gain is unity. The chain is `0.056 + 0.50 T` on a turn-in and
  exactly one frame on a straight, reproduced now by three independent
  estimators.
* **Closed:** matching the modelled chain to the identified chain *exactly*,
  through `cmd_dead_time`, is clean but 0.19-0.51 s/lap slower, and most of
  the 0.058 s cost is the speed servo that shares the parameter. Do not
  re-derive; if the dead time is ever to be corrected alone it needs its own
  parameter.
* **The one set worth driving** (section 5).
* **Open:** the period gate `model_cmd_tau_period_min` is the right shape of
  answer but a blunt one — it throws away the *shape* correction at 0.058 to
  avoid the *phase* it drags with it. The smallest honest fix is to make
  `model_cmd_tau` a SCALE on the modelled filter tau rather than a flag
  (`tau_c = cmd_tau * model_cmd_tau`), so 1.0 stays byte-identical and the
  display can model, say, 0.3 of the filter — 0.030 s of extra phase instead
  of 0.100 — and keep most of the pre-emphasis. Not implemented here: it is a
  code change to `mpc3_core._rollout` plus the `tools/sim_tracker.py` mirror,
  and the finals image is frozen.

---

## 5. The sets to drive, in order

All three keep the publish path untouched — `w_dcmd` 50 and `cmd_tau` 0.10
still filter the published command exactly as they do today, so the Sep 12
display flutter fix is unchanged. None of them moves `nd` (3 at every period
in the band) and none touches the readout lead.

**1. In band first (Xvfb 0.095-0.098), then the display:**

```
-p steer_tau:=0.0
```

The rack pole is the stage the identification says is smallest and least
period-correct, and once `model_cmd_tau` puts the 0.10 s filter in the
rollout, keeping 0.04 on top of it is 0.050 s of surplus phase. With v9's
gate as shipped this is `mct` + `steer_tau` 0 in band (harness 96/0,
**11.550** vs 11.570) and **`steer_tau` 0 with the filter NOT modelled on the
display, which the harness says is +0.30 s/lap** — so on the display drive it
only together with set 2 or 3, never alone.

**2. The same, ungated, once 1 is clean in band:**

```
-p steer_tau:=0.0 -p model_cmd_tau_period_min:=0.0
```

The fastest cell of the whole sweep at both periods (96/0, **10.950** at 0.058
and **11.550** at 0.098). Modelled total 0.235 s against the true
0.185 / 0.205. This is the arm that tests whether removing 0.040 s of surplus
phase is enough to stop the hairpin-2 inside cut that ungated `model_cmd_tau`
took live on the display; if it still cuts, the surplus is the remaining
0.050 s and set 3 is next.

**3. The full chain match, display only, if 2 still cuts:**

```
-p model_cmd_tau_period_min:=0.0 -p rate_terms:=apply,thr,delay,predict,enc,pose \
-p cmd_dead_time:=0.068 -p nd_delay_s:=0.180
```

Modelled total lands exactly on the identified 0.185 s (0.058) / 0.205 s
(0.098). Harness 48/0 at 11.180 on the display (+0.19) — the price is real and
it is mostly the speed servo, which this set also retunes. `nd_delay_s` pins
the committed-step count at 3 so nothing steps if the period lottery draws
short. **Do not drive this in band**: 48/0 but +0.51 s/lap at 0.098.
