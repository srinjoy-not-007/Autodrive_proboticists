# How peer race stacks keep steering smooth (research, Sept 12 2026)

## 1. ForzaETH race stack (the closest peer to us)

**Paper:** [arXiv:2403.11784](https://arxiv.org/abs/2403.11784) · [JFR version](https://onlinelibrary.wiley.com/doi/10.1002/rob.22429) · **Code:** [github.com/ForzaETH/race_stack](https://github.com/ForzaETH/race_stack)

Rates (from `controller/controller_manager.py`):

```python
self.loop_rate = 40 # rate in hertz
...
def control_loop(self):
    rate = rospy.Rate(self.loop_rate)
```

- LiDAR 40 Hz, **EKF state estimation 50 Hz** fusing wheel odom + IMU, localization 50 Hz (SynPF) / 200 Hz (Cartographer), **control node 40 Hz**, **MPC `MPC_freq: 20`** (`stack_master/config/NUC2/kinematic_mpc_params.yaml`). Note the layering: nothing consumes a raw per-scan pose. The paper: the EKF allows "fusion of multiple noisy measurements, with customizable variances, in a nonlinear state estimator", and "Cartographer performs smoother and more accurately under nominal conditions" while "SynPF ... tends to be more robust towards wheel-slippage".
- **MAP controller** ([README](https://github.com/ForzaETH/race_stack/blob/main/controller/map/README.md), paper [arXiv:2209.04346](https://arxiv.org/abs/2209.04346)) — geometric L1 guidance, lateral accel → steering via an offline lookup table. Smoothing is *structural*, in four places:
  - **Lookahead scales with speed:** `L1_distance = self.q_l1 + self.speed_now * self.m_l1` (tuned `m_l1 0.6`, `q_l1 -0.18`), clipped `t_clip_min 0.8` … `t_clip_max 5` m. README: *"To prevent oscillations when driving fast and to be able to track small curvatures, the L1 is scaled with delay compensated speed."*
  - **Lower bound tied to the error:** `lower_bound = max(self.t_clip_min, np.sqrt(2)*lateral_error)` — *"clip lower bound to avoid ultraswerve when far away from mincurv"*.
  - **Explicit per-step steering slew clip:**
    ```python
    # limit change of steering angle
    threshold = 0.4
    steering_angle = np.clip(steering_angle, self.curr_steering_angle - threshold,
                                             self.curr_steering_angle + threshold)
    ```
  - **Delay compensation by propagating the pose/speed forward**, separately for speed and steering: `speed_lookahead` 0.25 s ("Lookahead time in seconds to account for actuation and computation delay"), `speed_lookahead_for_steer` [0, 0.2, **0**] ("Can increase laptime by cutting more corners" — they ship 0 for the steering lookup).
  - Plus steer down-scaling above 7–8 m/s (`downscale_factor` 20%) and accel/decel scalers (1.2 / 0.9).
- **Their MPC** (`kinematic_mpc_params.yaml`): `N: 40`, `MPC_freq: 20`, `t_delay: 0.125`, `steps_delay: 3`, cost `qddelta: 1e-1` on steering rate, and **hard input-rate bounds `ddelta_min/max: ±0.2 rad/s`** (single-track version: `qddelta: 5e-1`, `±3.2 rad/s` — exactly our rack figure). Crucially, in `kinematic_mpc.py` the published command is the **state at the delay node, not `u0`**:
  ```python
  time_delay_step = 3
  self.u0 = self.acados_solver.get(0, "u")
  self.pred_x = self.acados_solver.get(time_delay_step, "x")
  self.steering_angle = self.pred_x[4]
  ```
  Steering *angle* is a state, steering *rate* the bounded input, so the commanded angle sequence is continuous by construction and cross-solve jumps are bounded by `ddelta_max * dt`.
- On 10 vs 40 Hz sensing: they never run control at sensor rate — control is 40 Hz over a 50 Hz fused estimate, MPC is 20 Hz. No result in the paper on degrading LiDAR to 10 Hz.

## 2. AutoDRIVE / RoboRacer ports

- [AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-ForzaETH-Race-Stack](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-ForzaETH-Race-Stack) — **both branches (`ROS-1`, `ROS-2`) contain only `README.md` + `LICENSE`**. Announced, not published. No port-specific tuning to learn from.
- [vaul-ulaval/autodrive_roboracer_ws](https://github.com/vaul-ulaval/autodrive_roboracer_ws) — the bridge `ackermann_to_autodrive_node.cpp` is a pure unit converter: clip to `max_steering_angle 0.5236`, divide, publish. **No rate limit, no filter, no tick synchronisation.** Their control is a wall-follower that latches `self.last_steering` when data is stale.
- Other upstreams in `upstream/`: `3omd4_CDC-TF2025` publishes pure-pursuit steering off a **fixed 100 Hz timer** (`create_timer(0.01, ...)  # 100 hz`) with a constant lookahead; `uci-f1tenth_race_stack` uses a fixed `lookahead: 1.5` m with a curvature speed factor. Nobody in the upstream corpus solves an optimisation per telemetry frame — they all run a geometric law with a spatial lookahead, which is inherently rate-invariant and low-pass by construction.
- Nothing anywhere handles the one-command-per-tick latching explicitly; our `docs/STATUS.md` finding ("the devkit bridge consumes one command per telemetry tick; a publish off the tick grid is dropped") appears to be original.

## 3. University stacks

| Stack | Controller | Control rate vs sensing | Smoothing / delay |
|---|---|---|---|
| **ForzaETH (ETH)** | MAP (L1 geometric) / KMPC / STMPC | 40 Hz control, 20 Hz MPC, over a **50 Hz EKF**; LiDAR 40 Hz | speed-scaled lookahead; 0.4 rad/step clip; `speed_lookahead` 0.25 s; `t_delay`/`steps_delay` in MPC; `qddelta` + `ddelta` bounds |
| **TUM Autonomous Motorsport (IAC)** | feedforward/feedback + LQR, later **Tube-MPC** ([arXiv:2205.15979](https://arxiv.org/abs/2205.15979), [mod-control docs](https://mod-control.readthedocs.io/), [TUMFTM/mod_vehicle_dynamics_control](https://github.com/TUMFTM/mod_vehicle_dynamics_control)) | control loop runs far faster than the planner (planner ~10 Hz, controller hard real-time Simulink) | tube = explicit robustness to estimation error, which is the principled version of a deadband |
| **KAIST (IAC)** | LQR primary with an **offline gain lookup table**; MPCC only in sim ([arXiv:2303.09463](https://arxiv.org/abs/2303.09463)) | *"Control command was calculated strictly at 100 Hz and passed to the drive-by-wire system"* — decoupled from perception entirely | gain scheduling offline; no per-frame optimisation in the loop |
| **UPenn F1TENTH course** | pure pursuit with speed-dependent lookahead; survey [arXiv:2402.18558](https://arxiv.org/html/2402.18558v2) | benchmark: planning 25 Hz, sim dynamics 100 Hz | *"a control regularisation term penalises steering angles, encouraging smooth behaviour"*; lookahead is the single tuning knob |
| **Adaptive-lookahead PP** ([arXiv:2111.08873](https://arxiv.org/pdf/2111.08873)) | PP | — | the canonical statement: small lookahead → agile corners but **oscillation on straights**; large → smooth but cuts corners |
| **MIT RACECAR / VAUL / Team Mamba** | pure pursuit / wall-follow / reactive; VAUL's public repos are the devkit + a wall follower + a particle filter (no tracker); no public "Team Mamba" RoboRacer repo under `hr1799` | — | VAUL's PF (`vaul_pf`) is a port of the MIT racecar particle filter |

The pattern is unanimous: **every serious stack runs its tracker at a fixed rate over a filtered state estimate, and none re-optimises once per raw localization update.**

## 4. Chatter remedies, with citations

- **Input/rate weighting is the first fix.** [Autoware issue #486 "MPC oscillation issue and input weight"](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/486): *"the input weight is set to 0 (not cared). Therefore, the current mpc seems to be acting like PD-control with infinite gain."* The [Autoware MPC lateral controller docs](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/) state the mechanism plainly: *"When weight_lat_error is large, the steering moves significantly to improve accuracy, which can cause oscillations."* Knobs: `mpc_weight_steer_rate`, `mpc_weight_steer_acc` (1e-6, jerk), `mpc_weight_lat_jerk` (0.1), and `weight_steering_input_squared_vel` — *"If you want to adjust the effect only in the high-speed range."*
- **Command filtering at actuator bandwidth.** Autoware `steering_lpf_cutoff_hz`, **default 3 Hz**: *"Adjusting the steering_lpf_cutoff_hz value can also be effective to forcefully reduce computational noise… The smaller the cutoff frequency, the stronger the noise reduction, but it also induce operation delay."*
- **Deadband at the noise floor.** Autoware `mpc_zero_ff_steer_deg` = 0.5° — *"threshold that feed-forward angle becomes zero"*; plus reference smoothing `curvature_smoothing_num_traj` = 15: *"In case your target curvature appears jagged… A larger value yields a smooth curvature calculation which reduces noise but can cause delay in feedforward computation."*
- **Delay model in the plant, not just the pose.** Autoware carries **both** `input_delay` (0.24 s, *"steering input delay time for delay compensation"*) **and** `vehicle_model_steer_tau` (0.3 s, first-order steering lag) and warns: *"If the onset of steering in curves is late, it's often due to incorrect delay time and time constant in the steering model."* Our `Actuator` models dead time + slew but no first-order lag.
- **Move blocking** — fix the input over blocks of steps: [arXiv:2002.06835](https://arxiv.org/pdf/2002.06835), [arXiv:2408.08020](https://arxiv.org/pdf/2408.08020). Reduces DOF and therefore high-frequency content in the solution; also cited as a remedy for *"input chattering"* at low QP sampling frequencies.
- **Dead-time compensation / Smith predictor in path tracking:** [arXiv:2507.06935](https://arxiv.org/pdf/2507.06935) (nonlinear dead-time compensation for path tracking). Key discrete-time fact reused there: *"The sample and hold process contributes a dead time of one half of a sampling period (Ta/2)"* plus a full period for compute — i.e. **the effective dead time is period-dependent**, exactly what our `rate_terms=delay` models.
- **Rate-limited actuator + delay ⇒ limit cycle.** This is the classical PIO literature: *"When actuators are rate-saturated… the effective delay in the control loop may increase. This effective delay increase manifests itself as a phase shift between the commanded and actual system signals and can instigate PIOs"* ([JSASS, Pilot-Induced Oscillation Analysis with Actuator Rate Limiting](https://www.jstage.jst.go.jp/article/tjsass/44/143/44_143_48/_article)); describing-function analysis of rate/acceleration limits: [Nonlinear Dynamics, 2026](https://link.springer.com/article/10.1007/s11071-026-12449-0) — rate saturation *"creates a limit-cycle isola, coexisting multiple solutions… and chaotic motions."* Damping remedies in that literature: reduce loop gain before the rate limit (our `w_lat`), phase-compensate the limiter, or allocate/filter so the command never demands more than the rate limit (CAPIO, [academia.edu/5701801](https://www.academia.edu/5701801/)).

## 5. Rules / Unity / telemetry period

No public spec predicts the period. The [IROS 2026 page](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/) and the [sim-racing repo](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing) only require *"set graphics to `Ultra` quality and vehicle in `Autonomous` mode"*; no frame-rate cap, vsync setting, or telemetry-rate statement, and no hardware tier. The AutoDRIVE architecture description ([summary](https://www.emergentmind.com/topics/autodrive-simulator), Samak et al.) says physics/actuation run in Unity `FixedUpdate` with **sensor rates configurable per sensor** and telemetry pushed as JSON over a WebSocket from Unity's single-threaded main loop. That matches our own measurements better than the asset-audit note ("telemetry every 90 ms") does: the period is set by the render/main thread's per-frame cost, which is why we see 0.058 s on a real display, 0.088–0.106 s on Xvfb, and the organisers 0.097 s. **Conclusion for us: assume nothing; the stack must be invariant over ~0.05–0.11 s.**

---

## (a) Why a per-sensor-frame MPC on a slew-limited, dead-time rack flutters on straights

Three things compound, and only the third is unusual to us.

1. **The cost function makes noise worth chasing.** In `mpc3_core.DEFAULTS`, `w_lat=24.0` against `w_rate=0.60` applied to `u*dt` — i.e. the penalty on a steering *change* of Δδ is `0.6·Δδ²`. A 0.05 m lateral error is worth `24·0.0025 = 0.060`; a 0.10 rad steering step costs `0.6·0.01 = 0.006`. The optimiser will spend 0.1 rad of lock to erase 5 cm. Our MCL lateral error floor is ~0.037 m p50 (measured) and is essentially white frame-to-frame, so **every frame the solver is handed a fresh ±0.04 m of pure noise and is told it is ten times more expensive than a violent steering step.** That is verbatim Autoware's *"When weight_lat_error is large, the steering moves significantly to improve accuracy, which can cause oscillations"*, and verbatim their issue #486: with no real input weight the MPC degenerates into a very high-gain PD.

2. **Nothing couples this solve's published command to the last one.** The warm start is only `u = np.clip(np.roll(self.u,-1)[:nf], -rate, rate)`; `w_rate`/`w_jerk` penalise smoothness *within* a plan, never plan-to-plan. The published number is an interpolation of the plan at `nd + apply_ahead_s/step_dt` ≈ 0.27 s in — a point far enough down the horizon that a small change in the initial lateral error rotates the whole early plan and moves it a lot. Two consecutive plans can each be internally smooth and still differ by 0.1 rad at the published index. ForzaETH avoids this structurally: they publish the steering **state** at the delay node of an MPC whose input is a rate hard-bounded at ±0.2 rad/s (KMPC), and their geometric controller clips the applied angle to ±0.4 rad of the previous one. We have neither bound at publish time.

3. **The rack and the dead time turn chatter into a limit cycle, and the plant is per-tick.** The bridge latches one command per telemetry tick, dead time ≈ 0.135 s ≈ 1.3–2.3 ticks, and the rack slews 3.2 rad/s. A command reversal is therefore acted on 1–2 frames late, while the rack is still travelling toward the *previous* command — so the error the next solve measures is partly the consequence of a command it can no longer see. That is the textbook rate-limit/dead-time PIO condition: rate saturation raises effective delay, which adds phase lag, which sustains the oscillation. A useful diagnostic: **a delay-driven limit cycle has a roughly rate-invariant frequency near 1/(4·τ_eff) ≈ 1.3–1.9 Hz for τ_eff 0.135–0.19 s; pure measurement noise flips near Nyquist and scales with the sensor rate.** Your 3.8–5.5 flips/s at 10–17 Hz works out to ~1.9–2.75 Hz cycles at *both* rates — i.e. mostly the delay-driven mode, with the amplitude (not the frequency) growing with rate.

**Why higher rate is worse**, specifically:
- Noise power injected per second rises linearly with the rate while the plant is unchanged — the loop gain in Hz-terms goes up, the phase margin against the fixed 0.135 s dead time goes down.
- The rack covers less angle per frame (`3.2·0.058 = 0.19 rad` vs `0.34 rad`), so the same 0.10–0.14 rad p90 step is a far larger fraction of a frame's slew authority: at 0.058 s you measured 12–28 % of commands exceeding the rack rate through s 13–16, 0 % at 0.094 s. Once you are rate-saturated, the actuator is a nonlinear phase-lag element and the loop is in the PIO regime.
- Every *fixed-in-seconds* constant becomes a different number of frames. `apply_ahead_s = 0.090` was 0.87 of a period at 0.104 s and 1.02 (and, below 0.069 s, floored away entirely by `apply_floor_steps`) at 0.058 s. **Command lead grows relative to the period as the rate rises → turn-in comes early → the hairpin.** That is your measured 0.054–0.084 m of extra plan lead, same sign and same corner as the collisions.
- And peers simply do not expose themselves to this: ForzaETH's controller sees a 50 Hz EKF output whose smoothness is independent of the LiDAR; KAIST computes "strictly at 100 Hz"; no one's lateral gain is a function of how fast the sensor happened to tick.

The honest summary: **their line is smooth because the smoothing lives in the estimator and in a fixed-rate, low-DOF control law with a spatial lookahead; ours lives nowhere.** We put a high-gain, noise-coupled optimiser directly on raw per-scan MCL output and then hand its answer to a rack that cannot keep up.

## (b) Ranked recommendations for mpc3

1. **Anchor the published command to the previous one (plan-to-plan consistency term) — highest value, lowest risk.**
   Add a residual `sqrt(w_cmd)·(δ_pub − δ_pub_prev)` on the published index (or, equivalently, constrain `|δ_pub − δ_pub_prev| ≤ steer_rate · measured_period` as a hard clamp, ForzaETH's `threshold = 0.4` per step generalised to be rate-invariant). Start `w_cmd` so that a 0.10 rad flip costs about what a 0.05 m error costs, i.e. `w_cmd ≈ 6` against `w_lat = 24`.
   *Expected:* sign-flip rate on straights falls several-fold; the >3.2 rad/s commands at 0.058 s go to zero by construction, not by accident of `apply_floor_steps`. *Risk:* low — but it does add phase lag at the hairpin, so A/B against the s 11.3 collision specifically, and note this is a *softer* version of what you already found (`apply_floor_steps:=0.0` killed the slew saturation and changed nothing at the hairpin) — so expect it to fix the flutter and **not** the turn-in.

2. **Deadband / Huber the lateral error at the localization noise floor.**
   Replace `sw_lat·e` with `sw_lat·max(0, |e| − e_dead)·sign(e)`, `e_dead ≈ 0.035–0.04 m` (your measured de-staled MCL p50 is 0.037 m). Precedent: Autoware's `mpc_zero_ff_steer_deg`; principled version: TUM's Tube-MPC, which treats estimation error as a set rather than a setpoint.
   *Expected:* straights go quiet without touching corner gain at all — this is the single change that targets the actual noise source rather than its symptom. *Risk:* a small steady-state offset inside the deadband (≤4 cm, well inside your 0.28 m clearance gate); watch for it biasing the ILC prior if you ever re-enable ILC.

3. **Make the lead spatial, not temporal — and model the rack as a first-order lag as well as dead time.**
   Two coupled changes: (i) define the publish point as a *distance* ahead with a floor (ForzaETH: `L1 = 0.6·v − 0.18`, `t_clip_min 0.8 m`) rather than `apply_ahead_s` seconds, which removes the whole class of rate bugs `rate_adaptive` is patching one term at a time; (ii) add `steer_tau` (Autoware's `vehicle_model_steer_tau`, default 0.3 s; ours is probably 0.03–0.08 s) to `Actuator` so the rollout knows the rack is a lag, not a teleport.
   *Expected:* rate invariance over 0.05–0.11 s for free, and the early turn-in at high rate should shrink because the lead stops growing with the period. *Risk:* medium — this re-tunes the corner entry that the current lag was accidentally providing (same trap as the lateral-jerk limiter), so it needs a full 100-lap Ultra gate plus the 0.058 s display run.

4. **Filter the published command at actuator bandwidth, with the filter inside the model.**
   First-order LPF on `δ_pub` with a **time constant in seconds** (τ ≈ 0.08–0.10 s ⇒ 1.6–2.0 Hz cutoff; Autoware ships 3 Hz), and add the same τ to the actuator model so the MPC compensates its own filter.
   *Expected:* kills whatever flutter survives (1) and (2); directly attacks the rate-saturation→phase-lag PIO mechanism. *Risk:* highest of the four — *"it also induce operation delay"*; an unmodelled filter is pure added dead time on a loop that is already delay-limited. Only do this after (1)/(2), and only with τ in the model.

5. **Decouple publish rate from solve rate — but by re-evaluating the plan, not by re-solving on a timer.**
   You already rejected `control_mode:=timer` (20 Hz solve, 0.25 s/lap slower, collisions at every period) — correctly, because it threw away estimate freshness. The untested variant: keep solving per pose (information-optimal on this bridge), but publish each tick by evaluating the *existing* plan at true elapsed wall time. Between solves the command moves along a smooth plan instead of jumping to a new one.
   *Expected:* the command stream becomes C¹ by construction and rate-invariant; matches how every peer stack is layered. *Risk:* medium-high implementation cost, and on this bridge (one command per tick) it only helps when a solve and a tick are out of phase — measure the phase first before building it.

**Not recommended:** move blocking (a 20-step horizon does not have the DOF problem it solves), and raising `w_rate` alone (it smooths the plan, not the plan-to-plan jump, so it costs corner-entry authority without addressing the mechanism).

**Sources:** [ForzaETH paper](https://arxiv.org/abs/2403.11784) · [ForzaETH code](https://github.com/ForzaETH/race_stack) · [MAP controller README](https://github.com/ForzaETH/race_stack/blob/main/controller/map/README.md) · [MAP paper](https://arxiv.org/abs/2209.04346) · [AutoDRIVE ForzaETH port (empty)](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-ForzaETH-Race-Stack) · [vaul-ulaval ws](https://github.com/vaul-ulaval/autodrive_roboracer_ws) · [TUM IAC](https://arxiv.org/abs/2205.15979) · [TUM mod-control](https://mod-control.readthedocs.io/) · [KAIST IAC](https://arxiv.org/abs/2303.09463) · [F1TENTH survey](https://arxiv.org/html/2402.18558v2) · [Adaptive-lookahead PP](https://arxiv.org/pdf/2111.08873) · [Autoware MPC lateral controller](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/) · [Autoware issue #486](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/486) · [Move blocking QP](https://arxiv.org/pdf/2002.06835) · [Move blocking horizons](https://arxiv.org/pdf/2408.08020) · [Nonlinear dead-time compensation for path tracking](https://arxiv.org/pdf/2507.06935) · [PIO with actuator rate limiting](https://www.jstage.jst.go.jp/article/tjsass/44/143/44_143_48/_article) · [Describing-function analysis of rate limits](https://link.springer.com/article/10.1007/s11071-026-12449-0) · [IROS 2026 rules page](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-iros-2026/)
