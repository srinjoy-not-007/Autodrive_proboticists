"""Pure-python core of the mpc3 tracker: a speed-aware Gauss-Newton horizon.

ROS-free on purpose, exactly like ``mpc2_core``: ``tools/sim_tracker.py`` drives
these same objects, so what is measured offline is the law that runs live.

What mpc3 keeps from mpc2, because it was measured to work
---------------------------------------------------------
``RefPath``   (raceline, smoothed curvature, corridor half-widths marched along
              the path normal from ``pf_core.load_map``'s distance field),
``Actuator``  (command history + ``/autodrive/roboracer_1/steering`` feedback,
              0.135 s dead time, 3.2 rad/s slew, interpolated across the samples
              the feedback covers), ``PoseFilter`` and ``wrap`` are imported
              from ``mpc2_core`` unchanged.  They are not re-derived here, and
              mpc2_core is not edited.

What is new, and why
--------------------
1. **Speed is part of the state.**  Every tracker before this one drove a fixed
   offline velocity profile: the steering solver could see that it was 0.25 m
   wide at a hairpin exit and could do nothing about the fact that it was also
   going too fast to rejoin.  That is the mechanism behind every s 13.6 / s 27.7
   collision in ``docs/STATUS.md``.  Here the horizon carries ``v`` and the
   speed the throttle is asked for comes out of the same plan the steering does:

   * the profile speed is the ceiling, never the floor;
   * it is cut by the *curvature the plan is actually asking for* --
     ``v <= sqrt(ay_budget / |kappa_cmd|)`` -- so running wide (which needs more
     curvature to rejoin) automatically buys less speed;
   * it is cut again in proportion to the predicted lateral error itself;
   * and the result is made feasible with a backward brake pass and a forward
     acceleration pass at the measured servo authority, so the plan is a speed
     trajectory the car can really follow rather than a wish.

2. **The lateral model is the measured one.**  Yaw rate is
   ``0.956 * v * tan(delta) / L`` (measured this session at every steering
   angle), and asked curvature above ``ay_deliver / v^2`` is not delivered --
   a smooth knee consistent with the skidpad, where the car holds ~0.9 of asked
   curvature to 9.9 m/s2 and then lets go.  This is a *saturation*, not the
   static understeer table the project notes forbid in a rollout: below the knee it is
   the identity, so it can never tell the controller to wind on more lock in the
   linear region.  Above the knee it makes extra lock worthless, which is the
   only thing that stops the solver asking for it.

3. **The dead time is handled in time, not by moving the pose.**  The first
   ``round(delay/dt)`` steps of the horizon are *committed*: their steering is
   whatever the already-published commands will produce, replayed through
   ``Actuator``.  The decision variables start where the new command can first
   bite, and the reference they are judged against is the reference at that
   TIME along the path.  Nothing is shifted by hand.

4. **No cut term.**  mpc2 needed ``cut_offset_m`` to match the legacy tracker's
   lap time because the legacy tracker cut corners by 0.139 m and the safe
   centreline did not describe that shorter path.  Tuning it is tuning a
   geometry knob inside a tracker, and the offline optimum (0.27) drove to the
   clearance floor and collided live.  mpc3 tracks the line it is given,
   honestly, and the geometry is the *line's* job -- run it on a min-curvature
   line if a shorter path is wanted.
"""

import math
import os

import numpy as np

try:
    from pit_crew import vehicle
    from pit_crew.mpc2_core import Actuator, PoseFilter, RefPath, wrap
except ImportError:                                  # running from src/pit_crew
    import vehicle
    from mpc2_core import Actuator, PoseFilter, RefPath, wrap

__all__ = ['Actuator', 'PoseFilter', 'RefPath', 'wrap', 'MPC3', 'DEFAULTS',
           'RateEstimator', 'RateAdapter', 'RateControl', 'ILC',
           'corridor_clearance', 'RATE_NOMINAL', 'RATE_TERMS',
           'RATE_ALL_TERMS', 'GapRider', 'gap_feedforward']


DEFAULTS = dict(
    # horizon ---------------------------------------------------------------
    horizon=20,
    step_dt=0.060,            # 1.20 s of preview
    min_step_m=0.10,          # never preview less than this per step
    # actuator --------------------------------------------------------------
    steer_rate=vehicle.STEER_RATE,
    delay_s=0.180,            # 0.135 dead time + 0.045 estimation, measured
    apply_ahead_s=0.090,      # which plan point the actuator is handed
    # --- where `nd` comes from (OPT-IN) -----------------------------------
    # `nd` is the number of horizon steps whose steering is already in flight.
    # 'fixed' is the shipping behaviour: `round(delay_s / step_dt)`, a constant
    # (3 at every period the clamp band admits, 0.052-0.156 s).
    #
    # 'period' derives it instead from the MEASURED pose-to-rack delay,
    # `period + dead_tick_c`, which is the law the live logs identify (see
    # `dead_tick_c`).  It is NOT the identity at the nominal period -- at
    # 0.104 s it gives round(0.149 / 0.060) = 2 against the shipping 3 -- so it
    # is opt-in and has to be driven before it is believed.  `delay_tick` in
    # `rate_terms` puts the SAME law on `Actuator.dead_time`, so with both on
    # the rollout's committed steps and the actuator model agree by
    # construction; with only one on they do not, which is why the pair is the
    # configuration to test.
    nd_mode='fixed',
    # OPT-IN (0.0 = off, byte-identical): the delay, in seconds, that `nd` is
    # derived from, PINNED regardless of what `set_rate`/`rate_terms` do to
    # `delay_s`.  It exists because the two things `delay_s` controls pull in
    # opposite directions once the measured dead-time law (`delay_tick`,
    # period + `dead_tick_c`) is used.  The ACTUATOR compensation wants the
    # true, period-dependent delay -- the fixed 0.180 s over-compensates a
    # 0.103 s dead time at a 0.058 s period and only 0.140 s at 0.095 s, which
    # is the measured rate-dependent turn-in phase (0.12-0.22 m of arc, see
    # docs/STATUS.md).  But `nd` derived from that same number steps 2/3/4
    # across the band, and moving `nd` is what made `nd_mode:='period'`
    # collide on the tight circuits.  Pinning `nd_delay_s` at the validated
    # 0.180 lets the dead-time law be corrected without touching the solver's
    # structure.
    nd_delay_s=0.0,
    # The constant of the measured dead-time law, in seconds.  Fitted offline
    # (slew-limited rack model, 3.2 rad/s, per-20 s-segment median) on the six
    # display/Xvfb race logs that carry both `cmd` and `steering`:
    # pose-stamp-to-rack = period + c with c = 0.042 s at a 0.058 s period and
    # 0.051-0.059 s at 0.094-0.100 s.  0.045 is the value the closed-loop
    # timing hunt reported and is kept as the default so the two agree.
    dead_tick_c=0.045,
    # Floor, in HORIZON STEPS, under which `apply_ahead_s` is not believed.
    # The publish index is `nd + apply_ahead_s/step_dt` and this floors the
    # second term, so at the shipping tune (0.090/0.060 = 1.5 steps) it does
    # nothing at all and 1.0 is exactly the old `max(self.dt, apply)`.
    #
    # It matters only BELOW a 0.069 s sensor period, and there it matters a
    # lot.  `rate_adaptive` scales `apply_ahead_s` to 0.87 of the measured
    # period, so at the 0.058 s this host's real display produces it asks for
    # 0.050 s -- under `step_dt`, where the old floor silently discarded it and
    # left the whole `apply` term (the only one ever swept against the period)
    # inert.  Worse, the floor makes the publish index an exact INTEGER, so the
    # published angle is a raw plan knot rather than an interpolation between
    # two, and successive publications step by a whole plan step: measured in
    # `tools/sim_tracker.py` at frame 0.058, 24.9% of the commands through
    # s 12-17 slew faster than the 3.2 rad/s the rack can follow (0.0% at every
    # frame >= 0.065), and the same signature is in the live 0.058 s log
    # (12-28% of samples at s 13-16, 0.0% in the 0.094 s 100-lap run).
    # Set 0.0 with `rate_adaptive` on; the rate clamp keeps the index off `nd`.
    apply_floor_steps=1.0,
    # `apply_lead_steps` (OPT-IN, -1.0 = off, byte-identical when off).
    # WHY it exists: `nd` does two unrelated jobs.  It is the number of horizon
    # steps whose steering is already in flight (a property of the machine's
    # dead time, and the rollout needs it right), and it is also the base of
    # the index the published angle is READ AT, `nd + max(apply_floor_steps,
    # apply_ahead_s/step_dt)`.  The readout is a phase LEAD on a ramping
    # reference: on a hairpin entry the plan's angle one step further on is a
    # tighter angle, so a lead longer than the true pose-to-rack delay turns in
    # early and the car settles INSIDE the line (measured: CDC-TF s 36-39.5,
    # Porto s 8-11).  Shortening it via `delay_s` also corrupts `nd`; this
    # parameter moves the readout ALONE.  When >= 0 the publish index is
    # exactly `nd + apply_lead_steps` (in horizon steps, fractional allowed),
    # `apply_floor_steps` and `apply_ahead_s` are both unread, and so is
    # `rate_adaptive`'s `apply` term -- every other rate term still applies.
    # -1.0 keeps the shipping readout untouched.
    apply_lead_steps=-1.0,
    # measured lateral model ------------------------------------------------
    yaw_gain=0.956,           # measured yaw rate / (v tan(delta)/L)
    ay_deliver=9.5,           # asked curvature above this/v^2 is not delivered
    sat_knee=0.80,            # fraction of it where the knee starts
    # speed plan ------------------------------------------------------------
    ay_budget=7.5,            # grip the SPEED plan may spend on curvature
    k_excess_max=0.15,        # 1/m of CORRECTION curvature the speed cut may see
    a_brake=4.0,              # m/s2 the plan may ask the servo to shed
    a_accel=5.0,              # ...and to add (measured servo authority)
    # `e_free` MUST sit above the tracker's own normal error or the emergency
    # brake becomes a permanent tax.  At 0.05 it sat BELOW the live median
    # (0.072 m, p95 0.179), so the cut was active essentially all the time:
    # measured by replaying the 100-lap live log through this solver, it alone
    # held the speed ceiling 0.188 m/s under the profile for the whole lap --
    # every other term contributed 0.000.  0.18 is just above the measured live
    # p95, so it is silent in normal running, and `k_e` is doubled so that it
    # still bites hard when the car is genuinely off line.
    e_free=0.18,              # lateral error tolerated before slowing, m
    k_e=2.0,                  # speed cut per metre of error beyond e_free
    # OPT-IN (1.0 = off, byte-identical to the shipping tune).  Scales the
    # `e_free`/`k_e` cut at the horizon steps where the OFFLINE PROFILE is
    # accelerating (`a_prof > 0`), i.e. on a corner EXIT.  The cut exists as an
    # emergency brake for a car that is off line going INTO a corner; on the
    # way out it is the opposite of useful -- measured on the IROS finals line
    # (harness, 0.058 s, frozen args), it is the ONLY term of this function
    # that binds anywhere on the lap, and it holds the throttle ceiling 0.21
    # m/s under the profile through the s 19 hairpin exit and 0.34 m/s under it
    # through the s 37.6 exit, exactly where the car is already 0.5-0.9 m/s
    # below plan.  The grip cap and the backward brake pass contribute 0.000.
    # 0.0 removes the cut on exits only; the entry cut is untouched.
    e_cut_accel_scale=1.0,
    # Companion gate for `e_cut_accel_scale` (0.0 = no extra condition).  When
    # > 0 the relaxation additionally requires the REFERENCE curvature at that
    # step to be below this value, so it starts only once the corner has opened
    # up.  Measured: with the scale alone, `a_prof` turns positive a few tenths
    # of a metre BEFORE the s 37.6 apex, the backward brake pass carries the
    # raised ceiling back into the entry, and the harness takes 9 collisions in
    # 36 laps, all at s 37.4-37.5.
    e_cut_accel_kappa=0.0,
    v_mod_min=0.55,           # never cut the profile speed below this fraction
    thr_lead_s=0.225,         # plan speed at the time the command is in force
    # How the lead is taken.  'point' is what raced: the ceiling AT
    # `thr_lead_s`, which both brakes early and ACCELERATES early.  Measured on
    # our own logs, 61% of the speed error is a pure spatial lag of 1.45 m
    # (0.33 s) at 9.6 Hz and 1.15 m at 11.4 Hz -- the tune was validated with
    # that lag acting as a free corner-entry brake, and at 11 Hz it shrinks,
    # the peak speed rises 6.01 -> 6.18, and the car arrives at the hairpin too
    # fast (corr(speed excess 3.25 m earlier, inside offset) = -0.82).  The
    # honest fix is a LONGER lead that is brake-only: 'min' takes the MINIMUM
    # of the ceiling over [0, thr_lead_s], so a drop ahead is anticipated and a
    # rise ahead is not.  A symmetric 0.30 s lead already failed live (7/32).
    thr_lead_mode='point',
    # WHICH speed curve the lead is taken from.  'ceiling' is what raced: the
    # speed plan's ceiling `v_lim`.  Note what that actually is -- `v_lim`
    # STARTS as `P.v_prof[idx]` and every later term is a `min` (ILC scale,
    # the `e_free` error cut, the grip cap, the backward brake pass), so with
    # the ILC off the ceiling can never exceed the offline profile at the same
    # index and 'min' is exactly 'ceiling'.  Measured Sept 12 (offline,
    # `MPC3.solve` on a braking approach): ceiling 6.925 against profile 6.951
    # at the same lead point.  The entry overspeed the display logs show is
    # therefore NOT a setpoint above the plan at the lead point; it is the
    # point lead itself plus a plant that delivers -1.7 where the profile asks
    # -3.0.  'profile' ignores the cuts and is the FASTER source of the three;
    # 'min' only differs from 'ceiling' when `ILC.v_scale` is above 1.
    thr_source='ceiling',
    # OPT-IN brake-phase lead, s, ADDED to `thr_lead_s` only where the offline
    # profile is braking (`a_prof < 0` anywhere inside the base lead window).
    # A straight therefore keeps the current point lead exactly; only a corner
    # approach looks further ahead, and when the extension is active the value
    # is the MINIMUM over the extended window, so it can only brake early and
    # never accelerate early.  0.0 = off, byte-identical to the shipping tune.
    thr_brake_lead_s=0.0,
    thr_enc_tau=0.19,         # encoder-filter lag, s, for the 'sum' rate rule
    # cost ------------------------------------------------------------------
    w_lat=24.0,
    w_yaw=3.0,
    w_steer=0.30,
    w_rate=0.60,              # on the applied steering change per step
    w_jerk=0.05,
    w_lat_T=40.0,
    w_yaw_T=8.0,
    w_barrier=400.0,
    barrier_margin=0.24,      # corridor the barrier defends, beyond the body.
                              # Raised with the speed: a faster car spends more
                              # of the corridor, and 0.16 put the offline worst
                              # footprint clearance at 0.100 m on seed 3.
    # output smoothing (OPT-IN; every one of these is 0.0 = off, and 0.0 is
    # byte-identical to the tune that raced) --------------------------------
    #
    # The flutter these exist for is NOT noise: with `pose_noise` 0.0 in
    # `tools/sim_tracker.py` the published command still alternates by exactly
    # `steer_rate * step_dt` = 0.192 rad between consecutive frames.  It is a
    # relay limit cycle -- the free steps of the horizon start at `nd` (0.18 s
    # of dead time), the FIRST of them carries no jerk cost at all
    # (`du = u[i] - (u[i-1] if i else u[0])` is identically 0 for i = 0) and
    # only a small rate cost, so the cheapest way for the solver to answer a
    # small lateral error is a one-step flick at the slew limit.  The car never
    # executes the flick -- the command is a zero-order hold for one sensor
    # period -- so the next solve sees the same error and flicks the other way.
    # The faster the sensor, the more solves per dead time and the stronger the
    # cycle: measured at frame 0.058 vs 0.104 in the harness and live.
    #
    # `w_dcmd`   the causal fix: a Gauss-Newton residual on (the angle about to
    #            be published) - (the angle published last frame).  It is the
    #            missing move-suppression term of a receding-horizon controller
    #            whose plan is only ever executed one frame at a time.  It is
    #            NOT a lag: a genuine turn-in ramp is planned over the horizon
    #            and pays the penalty once per frame, the same as before.
    # `cmd_tau`  a rate-invariant first-order low pass on the published
    #            command, `alpha = 1 - exp(-period / tau)` with the period the
    #            one `RateControl` measures, so its lag is fixed in SECONDS at
    #            any sensor rate.  Pure output filtering: it adds `tau` of lag.
    # `cmd_notch` a 2-tap smoother, `cmd <- (1 - b) * cmd + b * prev`.  b = 0.5
    #            is an exact zero at the Nyquist frequency, which is where the
    #            relay cycle lives, for a group delay of only half a frame.
    # `e_dead`   a deadband on the LATERAL residual only: inside it the cost
    #            and its gradient are exactly zero, so an error the size of the
    #            localization noise floor (live lateral est-truth p95 0.034-
    #            0.047 m) buys no steering at all.  Outside it the residual is
    #            `e - e_dead` with the same weight, so a real excursion is
    #            tracked with the same gain, only offset -- which is why it
    #            costs clearance far less than lowering `w_lat` does.
    w_dcmd=0.0,
    cmd_tau=0.0,
    cmd_notch=0.0,
    e_dead=0.0,
    # --- the rack inside the rollout (OPT-IN, research R3) -----------------
    # `steer_tau`      first-order lag of the rack, in seconds, ON TOP of the
    #                  dead time and the 3.2 rad/s slew the rollout already
    #                  models.  The plan's steering angle then lags the plan's
    #                  COMMAND, which is what the identified rack does (joint
    #                  fits of the display staircase: dead 0.100 / tau 0.04 /
    #                  slew 3.2 at a 0.058 s period, dead 0.14-0.16 / tau
    #                  0.06-0.08 at 0.100 s).  Autoware ships the same term as
    #                  `vehicle_model_steer_tau` and warns that a wrong one is
    #                  the usual cause of late turn-in.  Set the SAME value on
    #                  `Actuator(steer_tau=...)` -- the node and the harness
    #                  both do -- or the delay compensation and the rollout
    #                  disagree about the same rack.
    # `model_cmd_tau`  1.0 puts the `cmd_tau` output filter INSIDE the rollout
    #                  as a first stage of the same lag chain, so the plan
    #                  stops asking for steps the filter will not pass on.
    #                  Inert while `cmd_tau` is 0.0.  The published angle is
    #                  then read from the PRE-filter plan (`solve` applies the
    #                  filter itself, exactly once).
    # At `steer_tau` 0 and `model_cmd_tau` 0 the rollout takes the original
    # branch untouched, so every committed number is reproduced byte for byte.
    steer_tau=0.0,
    model_cmd_tau=0.0,
    # --- where the published angle is read out of the plan (OPT-IN) --------
    # `lead_mode` 'temporal' is the shipping behaviour: the publish index is
    # `nd + max(apply_floor_steps, apply_ahead_s/step_dt)`, i.e. a lead in
    # SECONDS, which is exactly the constant `rate_adaptive` has to scale with
    # the sensor period.  'spatial' replaces it with a lead in METRES,
    # `max(lead_min_m, lead_m0 + lead_k * v)`, walked forward along the warm
    # speed plan from step `nd`, which is rate-invariant by construction
    # because a distance ahead of the car does not know what the sensor period
    # is.  The defaults are the DIRECT spatial equivalent of the shipping
    # `apply_ahead_s` 0.090 s (`lead_k` = that lead in seconds) with a floor
    # that stops it collapsing onto the `nd` knot at walking pace.
    # NOT ForzaETH's L1: their `0.6 v - 0.18`, floored at 0.8 m, is a
    # pure-pursuit LOOKAHEAD POINT, a different object from the plan knot the
    # published angle is read at.  Measured here (pole plant, 12 laps x 3
    # seeds, frames 0.058 and 0.104): 198 and 210 collisions, |e| p95 0.59 m.
    # Do not re-derive.
    # Interaction with `rate_adaptive`: in 'spatial' mode the `apply` term is
    # INERT (nothing reads `apply_ahead_s`).  Every other term still applies --
    # `delay` (the actuator dead time and `nd`), `thr`, `predict`, `enc`,
    # `pose` -- because they are properties of the machine, not of the lead.
    # `apply_floor_steps` still floors the index off the `nd` knot.
    lead_mode='temporal',
    lead_m0=0.0,
    lead_k=0.09,
    lead_min_m=0.30,
    # --- no-anticipation publish gate (OPT-IN, 0.0 = off) -----------------
    # WHY: the published angle is read `nd + apply` steps into the plan, and on
    # this track the plan knot it lands on can already be ON the curvature step
    # into the s 19 hairpin (kappa 0.09 -> 1.60 in 0.5 m).  Measured live
    # (`race_20260918_162222.csv`, display, 0.058 s): the published angle
    # averages +0.067 rad over s 16-18 where the line asks 0.03, i.e. the
    # turn-in is applied 1-2.5 m early -- the "swivel before the turn".
    # WHAT: while no corner is within `pub_gate_d` metres AHEAD of the car
    # (`|kappa_ref| < pub_gate_kappa` over that window), the published angle is
    # clipped to the LINE-MATCHING angle there plus `pub_gate_corr` radians of
    # correction authority.  It is a clip on the OUTPUT only: the plan, the
    # rollout and every cost are untouched, and the gate releases entirely as
    # soon as the corner is inside the window, so a real turn-in is never
    # delayed by more than `pub_gate_d` metres of travel.
    # `pub_gate_corr` 0.0 disables it and is byte-identical to the tune that
    # raced.  Sensible band: 0.05-0.12 rad of correction, 1.0-3.0 m of window.
    # `pub_gate_e` is the RELEASE: the gate is off whenever the car is further
    # than this from the line, because a bounded correction is not enough to
    # rejoin from a collision reset.  Measured (harness, `--teleport-every 4`,
    # 12 laps x 3 seeds x 2 frames): without the release the gate takes 702
    # collisions at 0.058 s against the ungated 43 -- the car is put down off
    # line on the straight and the clip will not let it come back.
    pub_gate_corr=0.0,
    pub_gate_d=1.5,
    pub_gate_kappa=0.30,
    pub_gate_e=0.25,
    # solver ----------------------------------------------------------------
    iters=6,
    outer=2,                  # speed-plan / steering-plan fixed point passes
    lam0=1e-3,
)


def _sat_curvature(delta, k_max, knee, gain):
    """Delivered curvature and its derivative w.r.t. the steering angle.

    Identity (times ``gain``) until ``knee * k_max``, then an exponential
    approach to ``k_max``.  Smooth, monotone, and with a derivative that decays
    to zero -- which is what makes extra lock worthless to the optimiser instead
    of merely expensive.
    """
    k = math.tan(delta) / vehicle.WHEELBASE
    dk = (1.0 + math.tan(delta) ** 2) / vehicle.WHEELBASE
    a = abs(k)
    kn = knee * k_max
    if k_max <= 0.0 or a <= kn:
        return gain * k, gain * dk
    span = max(k_max - kn, 1e-6)
    e = math.exp(-(a - kn) / span)
    f = kn + span * (1.0 - e)
    return gain * math.copysign(f, k), gain * dk * e


# --------------------------------------------------------------------------
# rate adaptivity  (OPT-IN: `rate_adaptive`, default off everywhere)
# --------------------------------------------------------------------------
#
# mpc3's constants are not all in seconds by accident.  Several of them are
# really "some number of SENSOR PERIODS", and the sensor period is not a
# property of the tracker at all -- it is a property of the machine the
# simulator happens to be running on.  Measured on the same line
# (`porto_mesh_mc0.63_ay6.75`), same map, same tracker:
#
#   logger dt 0.104 s  ->  100 laps, 0 collisions,  |cte| p50 0.068  p95 0.208
#   logger dt 0.088 s  ->   20 laps, 11 collisions, |cte| p50 0.093  p95 0.305
#
# and the offline harness driven at those two periods reproduces the p50
# almost exactly (0.067 / 0.090) with nothing else changed.  So the tracker is
# not "tuned for 9.6 Hz" by choice; it is tuned for 9.6 Hz by accident, and the
# organisers' machine measured 0.097 s.
#
# The dominant term, isolated in the harness at fixed 0.088 s frames, is
# `apply_ahead_s`: the command that is published is held for exactly one sensor
# period, so the angle worth publishing is the plan's angle about one period
# after the command bites.  Sweeping it at three periods, the optimum tracks
# the period (best p50/p95 at 0.070-0.078 s for a 0.088 s frame, 0.085 s for
# 0.097, 0.090 s for 0.104) -- i.e. a constant 0.87 of a period, which is
# exactly what the validated 0.090 s is at the 0.104 s rate it was validated
# at.  Everything else here is the same argument applied to the other
# implicitly-per-period constants, and each is separately switchable so the
# isolation can be redone.
RATE_NOMINAL = 0.104          # the period mpc3's defaults were validated at
# `apply` is the only term whose optimum was actually SWEPT against the period,
# and it was swept with `delay` held fixed.  `delay` is therefore NOT in the
# default set, for two reasons that compound:
#
#   * it double-counts the half-frame.  `delay` puts 0.5p into
#     `cmd_dead_time`, and `apply` then adds 0.87p on top of it, so the total
#     lead handed to `_indices` scales as `bridge + est_lag + 1.37p` where the
#     argument (dead time + half a hold window) only supports 1.0p; and
#   * today that is INVISIBLE rather than wrong, which is worse.  The solver
#     lead is `nd * step_dt + apply`, and `nd = round(delay_s / step_dt)` is 3
#     for every period from 0.060 s to 0.160 s -- so in the whole realistic
#     band `delay` moves only `Actuator.dead_time`, and at 0.200 s `nd` goes
#     3 -> 4 and the lead jumps 0.060 s in one step.
#
# Pass `rate_terms=apply,thr,delay,predict,enc,pose` to re-enable it, but
# isolate it first: `rate_terms=delay` alone against `rate_terms=apply` alone.
RATE_ALL_TERMS = ('apply', 'thr', 'delay', 'predict', 'enc', 'pose')
# `delay_tick` is an ALTERNATIVE to `delay`, not an addition -- both write
# `cmd_dead_time` and `delay_tick` wins when both are listed -- so it is
# deliberately not in `RATE_ALL_TERMS`.  Ask for it by name:
# `rate_terms=apply,thr,predict,enc,pose,delay_tick`.
RATE_TERMS = ('apply', 'thr', 'predict', 'enc', 'pose')


class RateEstimator:
    """Robust online estimate of the pose period.

    Median of the last ``window`` inter-arrival times, which is what makes it
    survive the thing a mean does not: a dropped pose doubles one interval, and
    a burst of two poses in one frame halves the next.  Intervals outside
    ``[lo, hi]`` are not evidence about the rate at all and are dropped.
    """

    def __init__(self, nominal=RATE_NOMINAL, window=20, lo=0.03, hi=0.40,
                 min_samples=8):
        self.nominal = float(nominal)
        self.window = int(window)
        self.lo, self.hi = float(lo), float(hi)
        self.min_samples = int(min_samples)
        self._prev = None
        self._d = []
        self.period = float(nominal)
        self.ready = False
        self.n_seen = 0

    def update(self, t):
        """One pose stamp.  Returns the current period estimate."""
        if not math.isfinite(t):
            return self.period
        if self._prev is None:
            self._prev = t
            return self.period
        dt = t - self._prev
        if dt < self.lo:
            # A duplicate or an out-of-order stamp. Do NOT advance the anchor:
            # the estimator would then measure the NEXT interval short too,
            # which is the same trap `vehicle.SpeedEstimator` documents.
            return self.period
        self._prev = t
        if dt <= self.hi:
            self._d.append(dt)
            self.n_seen += 1
            if len(self._d) > self.window:
                self._d.pop(0)
            if len(self._d) >= self.min_samples:
                self.period = float(np.median(self._d))
                self.ready = True
        return self.period


class RateAdapter:
    """The period-dependent constants, as ratios of the validated period.

    Every formula here is the identity at ``nominal``, so switching adaptivity
    on at the rate the tracker was validated at changes nothing -- that is the
    property that makes this safe to ship behind a flag.  ``terms`` selects
    which families adapt, so any one of them can be isolated.
    """

    def __init__(self, cmd_dead_time=0.135, delay_s=0.180, apply_ahead_s=0.090,
                 thr_lead_s=0.225, max_predict_s=0.35, stale_pose_s=0.35,
                 enc_alpha=0.35, pose_blend=1.0, enc_tau=0.19,
                 dead_tick_c=0.045, nominal=RATE_NOMINAL, terms=RATE_TERMS):
        self.nominal = float(nominal)
        self.terms = tuple(terms)
        self.dead0 = float(cmd_dead_time)
        self.delay0 = float(delay_s)
        self.apply0 = float(apply_ahead_s)
        self.thr0 = float(thr_lead_s)
        self.predict0 = float(max_predict_s)
        self.stale0 = float(stale_pose_s)
        self.enc0 = float(enc_alpha)
        self.blend0 = float(pose_blend)
        self.enc_tau = float(enc_tau)
        # The bridge's own dead time, i.e. what is left of the measured
        # command-to-movement delay once the half-frame the latched command
        # waits at the pickup boundary is taken out.
        self.bridge = self.dead0 - 0.5 * self.nominal
        self.est_lag = self.delay0 - self.dead0
        # The constant of the `delay_tick` law (see DEFAULTS' `dead_tick_c`).
        self.dead_c = float(dead_tick_c)

    def values(self, period):
        p = float(period)
        r = p / self.nominal
        v = dict(cmd_dead_time=self.dead0, delay_s=self.delay0,
                 apply_ahead_s=self.apply0, thr_lead_s=self.thr0,
                 max_predict_s=self.predict0, stale_pose_s=self.stale0,
                 enc_alpha=self.enc0, pose_blend=self.blend0)
        if 'delay_tick' in self.terms:
            # ONE-TICK LATCH + a fixed actuator delay, which is what the live
            # logs identify.  The controller's command is not picked up by the
            # bridge until the emit at the END of the next telemetry callback,
            # so the wait is a whole period, not half of one; what is left over
            # is constant.  Measured (six race logs with both `cmd` and
            # `steering`, slew-limited rack fit, 20 s segments): pose-to-rack =
            # period + 0.042 s at 0.058 and + 0.051-0.059 s at 0.094-0.100,
            # residuals <= 0.010 s against this law and up to 0.026 s against
            # `bridge + period/2`, which the 0.058 s point rejects.
            #
            # Note what this is measured FROM: the pose stamp.  `Actuator`
            # records its commands at the PUBLISH instant, which is one solve
            # later, so setting it here charges the controller's own latency
            # twice.  That is deliberate and bounded (~0.02-0.04 s, i.e. under
            # one horizon step) -- `dead_tick_c` is the knob to take it back
            # out -- but it is why this is opt-in and has to be driven.
            v['cmd_dead_time'] = p + self.dead_c
            v['delay_s'] = v['cmd_dead_time'] + self.est_lag
        elif 'delay' in self.terms:
            v['cmd_dead_time'] = self.bridge + 0.5 * p
            v['delay_s'] = v['cmd_dead_time'] + self.est_lag
        if 'apply' in self.terms:
            v['apply_ahead_s'] = self.apply0 * r
        if 'thr_sum' in self.terms:
            # The transport lag of the speed loop, built rather than scaled:
            # command dead time + one command hold + the encoder filter's own
            # lag.  ~0.43 s at 0.104 s, which is only usable with
            # `thr_lead_mode` 'min' -- as a symmetric point lead it accelerates
            # early out of every corner.
            v['thr_lead_s'] = v['cmd_dead_time'] + p + self.enc_tau
        elif 'thr' in self.terms:
            v['thr_lead_s'] = self.thr0 * r
        if 'predict' in self.terms:
            # ONE-SIDED, and that asymmetry is the whole point: these two are
            # safety gates, not tuning constants.  A faster sensor earns a
            # tighter open-loop dead-reckon and a tighter stale gate; a slower
            # one must not earn a looser one.  Unclamped, a 0.39 s period (the
            # most `RateEstimator.hi` admits) gave max_predict_s = 1.31 s --
            # 7.9 m of open-loop extrapolation at 6 m/s handed to the solver,
            # and 1.31 s before the watchdog coasts.
            v['max_predict_s'] = self.predict0 * min(r, 1.0)
            v['stale_pose_s'] = self.stale0 * min(r, 1.0)
        if 'enc' in self.terms and 0.0 < self.enc0 < 1.0:
            # A per-SAMPLE low pass has a per-sample memory; holding the same
            # alpha at a different rate silently changes its lag in seconds.
            # Matching `(1 - alpha)` per unit TIME keeps the lag fixed.
            v['enc_alpha'] = 1.0 - (1.0 - self.enc0) ** r
        if 'pose' in self.terms and 0.0 < self.blend0 < 1.0:
            v['pose_blend'] = 1.0 - (1.0 - self.blend0) ** r
        return v


class RateControl:
    """Estimator + adapter + the objects that have to follow them.

    One object so that the node and ``tools/sim_tracker.py``'s ``MPC3Sim``
    cannot drift apart: what the harness measures is literally what races.
    Disabled (``enabled=False``) it touches nothing at all -- ``update`` still
    measures the period, so the rate can be LOGGED without being acted on.
    """

    def __init__(self, mpc, act, enabled=False, filt=None, speed_est=None,
                 thr=None, nominal=RATE_NOMINAL, window=20, terms=RATE_TERMS,
                 min_change=0.002, rate_lo=0.5, rate_hi=1.5, rate_slew=0.0,
                 **nominals):
        self.mpc = mpc
        self.act = act
        self.filt = filt
        self.speed_est = speed_est
        # `ServoThrottle.hold` is "how long one command stays in force", i.e.
        # one sensor period -- the same machine property everything else here
        # follows. Inert while the caller passes `lead_max=0.0` (mpc3 does), and
        # correct rather than accidentally 0.09 s the moment it does not.
        self.thr = thr
        self.enabled = bool(enabled)
        self.est = RateEstimator(nominal=nominal, window=window)
        self.adapter = RateAdapter(nominal=nominal, terms=terms, **nominals)
        self.min_change = float(min_change)
        # Trust band on the MEASURED period, as a ratio of the validated one.
        # Every adapted constant is linear or exponential in the period, so an
        # estimate that is wrong (a stalled sim, a clock change, a bridge
        # hiccup that survives the median) moves all of them together and in
        # the same, dangerous direction.  Nothing outside [0.5, 1.5] x nominal
        # has ever been measured on any machine we have run on -- 0.088, 0.097
        # and 0.104 s -- so outside that band the right answer is the tune that
        # was validated, not an extrapolation of it.
        self.rate_lo = float(rate_lo) * float(nominal)
        self.rate_hi = float(rate_hi) * float(nominal)
        # OPT-IN bound on how fast the APPLIED period may follow the measured
        # one, in seconds of period per second of wall clock.  0.0 (default)
        # is the historical behaviour and is byte-identical to it.
        #
        # Why it exists: `RateEstimator` is a median over `window`, so it does
        # not follow a drift smoothly -- it holds the old value for half a
        # window and then steps.  Measured here on a 0.092 -> 0.110 s step at
        # window 20: the estimate is 0.092 for 9 samples (0.83 s), then 0.101
        # for one, then 0.110, i.e. the whole 0.018 s arrives in two samples.
        # Every adapted constant is linear in the period, so that is a step in
        # `apply_ahead_s` (0.0156 s here) and in `thr_lead_s` (0.039 s) inside
        # one frame.  A slew spreads it: at 0.01 s/s the same step takes 1.8 s.
        self.slew = float(rate_slew)
        self._t_prev = None
        self.clamped = 0
        self.applied = float(nominal)
        self.values = self.adapter.values(nominal)

    def update(self, t_stamp):
        """One pose stamp.  Returns the values in force (adapted or nominal)."""
        p = self.est.update(t_stamp)
        # Record the MEASURED period on the solver whether or not adaptivity is
        # enabled: `cmd_tau` is a lag in seconds and needs it to stay a lag in
        # seconds at any sensor rate.  Nothing reads it while `cmd_tau` is 0.0.
        self.mpc.period = p
        if not (self.enabled and self.est.ready):
            return self.values
        if not (self.rate_lo <= p <= self.rate_hi):
            self.clamped += 1
            p = min(max(p, self.rate_lo), self.rate_hi)
        if self.slew > 0.0:
            if self._t_prev is None:
                # ACQUISITION is not a drift: the first estimate the window is
                # ready to give is taken whole.  Rate-limiting it too would
                # leave the car racing the 0.104 s tune on a 0.058 s host for
                # `|0.104 - 0.058| / slew` seconds, which is the whole race at
                # any useful slew.
                self._t_prev = float(t_stamp)
            else:
                # Bound the CHANGE, not the value: a clamped or stepped
                # estimate is still followed, only smoothly.  The budget is
                # measured from the last ACCEPTED change, not from the last
                # sample -- otherwise `min_change` and the slew deadlock each
                # other (a per-sample budget of `slew * period` below
                # `min_change` is never big enough to be applied, and the
                # adaptation freezes at its acquisition value for the rest of
                # the run: measured, `rate_slew` 0.005 on a 0.058 -> 0.125 s
                # ramp gave 74 collisions in 60 laps that way).
                step = self.slew * max(float(t_stamp) - self._t_prev, 0.0)
                p = min(max(p, self.applied - step), self.applied + step)
        if abs(p - self.applied) < self.min_change:
            return self.values
        self._t_prev = float(t_stamp)
        self.applied = p
        v = self.adapter.values(p)
        self.values = v
        self.mpc.set_rate(delay_s=v['delay_s'],
                          apply_ahead_s=v['apply_ahead_s'],
                          thr_lead_s=v['thr_lead_s'])
        self.act.dead_time = v['cmd_dead_time']
        if self.filt is not None and 0.0 < self.adapter.blend0 < 1.0:
            self.filt.blend = v['pose_blend']
        if self.speed_est is not None and 0.0 < self.adapter.enc0 < 1.0:
            self.speed_est.alpha = v['enc_alpha']
        if self.speed_est is not None:
            # Counts samples, means a hold TIME: 6 samples is 0.624 s at
            # 0.104 s and 0.528 s at 0.088 s of steering by a stale speed.
            self.speed_est.reject_max = vehicle.reject_max_for_dt(p)
        if self.thr is not None:
            self.thr.hold = p
        return v


# --------------------------------------------------------------------------
# degraded mode for a telemetry GAP  (OPT-IN: `gap_predict_s`)
# --------------------------------------------------------------------------

class GapRider:
    """What to do while the telemetry has stopped, and on the first pose back.

    The shipping behaviour is a cliff: `stale_pose_s` (0.35 s at the nominal
    period, and ONE-SIDED under `rate_adaptive`, so 0.195 s at a 0.058 s
    period) and then the watchdog holds the last steering angle and publishes
    zero throttle.  Measured in the harness, one dropped frame is harmless and
    three (0.40 s at 0.104 s) put the car on that coast for long enough to miss
    a corner entry.

    Nothing here is a new control law.  `ride` says "the pose is old but not
    yet useless": the tracker keeps solving from the SAME estimate dead
    reckoned further forward by `Actuator.predict` -- the plan and the actuator
    model already describe the next second -- at the plan's own timing, and
    `v_cap` ramps the commanded speed down while it does so, because an
    open-loop plan deserves less speed the longer it runs blind.  Past
    `predict_s` the old coast takes over unchanged.

    `blend`/`gate` are the re-acquisition guard: the first pose after a long
    gap is the output of a filter that has just dead reckoned through the gap
    itself (`localization.py` clamps its own step to 0.5 s and drops to
    `_patience` 1), so taking it whole is a snap.  Blending it against our own
    propagation for a few frames costs nothing when the pose is good.

    Every default is off, and `enabled` is False then, so a node or harness
    that does not set `gap_predict_s` behaves exactly as before.
    """

    def __init__(self, predict_s=0.0, decel=3.0, v_min=1.0, reacq_blend=0.0,
                 reacq_gate_m=0.0, reacq_n=2, reacq_gap_s=0.0):
        self.predict_s = float(predict_s)
        self.decel = float(decel)
        self.v_min = float(v_min)
        self.reacq_blend = float(reacq_blend)
        self.reacq_gate_m = float(reacq_gate_m)
        self.reacq_n = int(reacq_n)
        # Gap length that arms the guard; 0.0 means "one stale window", which
        # the caller passes in because only it knows the rate-adapted value.
        self.reacq_gap_s = float(reacq_gap_s)
        self._last_stamp = None
        self._armed = 0
        self.rides = 0            # solves made on a dead-reckoned pose
        self.gaps = 0             # gaps long enough to arm the guard
        self.last_gap = 0.0

    @property
    def enabled(self):
        return self.predict_s > 0.0

    # -- while the telemetry is gone ---------------------------------------
    def ride(self, age, stale_s):
        """True while a gap of `age` seconds is still worth driving through."""
        if not self.enabled:
            return False
        return stale_s < age <= stale_s + self.predict_s

    def window(self, stale_s):
        """Seconds of dead reckoning from the pose stamp that `ride` needs."""
        return stale_s + self.predict_s if self.enabled else 0.0

    def v_cap(self, age, stale_s, v_cmd):
        """Ramp the commanded speed down while driving on an old pose."""
        if not self.enabled:
            return v_cmd
        over = age - stale_s
        if over <= 0.0:
            return v_cmd
        return min(v_cmd, max(self.v_min, v_cmd - self.decel * over))

    # -- the first pose back ------------------------------------------------
    def see_pose(self, stamp, stale_s):
        """Record an accepted pose; arm the guard if it ended a long gap."""
        prev, self._last_stamp = self._last_stamp, float(stamp)
        if prev is None:
            return 0.0
        gap = float(stamp) - prev
        self.last_gap = gap
        if not (self.enabled and self.reacq_blend > 0.0):
            return gap
        trig = self.reacq_gap_s if self.reacq_gap_s > 0.0 else stale_s
        if gap > trig:
            self.gaps += 1
            self._armed = max(1, self.reacq_n)
        return gap

    def arc_ahead(self, age, stale_s, v):
        """Metres travelled since the pose stamp, under the ramp.

        Plan speed until the pose went stale, then the ramp `v_cap` applies;
        the average of the two ends is exact for a linear ramp.
        """
        if age <= 0.0:
            return 0.0
        head = min(age, stale_s)
        tail = max(age - stale_s, 0.0)
        v_end = self.v_cap(age, stale_s, v)
        return v * head + 0.5 * (v + v_end) * tail

    def filter_gains(self, blend, gate):
        """(blend, gate) for THIS pose; consumes one armed frame."""
        if self._armed <= 0:
            return blend, gate
        self._armed -= 1
        g = self.reacq_gate_m if self.reacq_gate_m > 0.0 else gate
        return min(blend, self.reacq_blend), g


def gap_feedforward(path, i0, ds, yaw_gain=1.0, cap=None):
    """The LINE's own steering angle `ds` metres past index `i0`.

    The open-loop half of the gap handler (`gap_mode` 'ff'): no cross-track
    term at all, because with no pose there is no cross-track measurement --
    only a dead reckoning of one, and the solver answering a dead-reckoned
    error is what makes a long ride worse than the plan it is riding.  Returns
    `(index, steering angle)`.
    """
    j = int(i0 + round(max(ds, 0.0) / max(path.spacing, 1e-3))) % path.n
    k = float(path.kappa[j]) / max(float(yaw_gain), 1e-3)
    d = math.atan(vehicle.WHEELBASE * k)
    lim = vehicle.MAX_STEER if cap is None else min(vehicle.MAX_STEER, cap)
    return j, max(-lim, min(lim, d))


# --------------------------------------------------------------------------
# iterative learning control over arc length  (OPT-IN: `ilc_enable`)
# --------------------------------------------------------------------------

def corridor_clearance(path: RefPath, i0, e_lat, body_m=0.25):
    """Room left between the car body and the corridor walls, at ``i0``.

    A proxy for ``tools/footprint_analysis``'s footprint clearance that can be
    evaluated LIVE, from the two things the tracker already has: the corridor
    half-widths the barrier is built from and its own lateral error.  The
    minimum is taken over the arc the body spans so that a body-length before a
    pinch point already counts as tight.  Returns ``inf`` without a corridor.
    """
    if not path.has_corridor:
        return float('inf')
    n = max(1, int(round(body_m / max(path.spacing, 1e-3))))
    idx = (np.arange(i0 - n, i0 + n + 1)) % path.n
    hw = 0.5 * vehicle.CAR_WIDTH
    left = path.w_left[idx] - e_lat - hw
    right = path.w_right[idx] + e_lat - hw
    return float(min(left.min(), right.min()))


def _max_circular_gap(seen) -> int:
    """Longest run of False in a circular boolean array (all False -> len).

    Rate-free by construction: it measures how much of the LAP was never
    driven, in bins, where a sparse-but-complete lap leaves only the sampling
    gaps (a few bins at any sensor period) and a fragment leaves one huge one.
    """
    a = np.asarray(seen, dtype=bool)
    n = a.size
    if not a.any():
        return int(n)
    if a.all():
        return 0
    idx = np.flatnonzero(a)
    # Circular gaps BETWEEN consecutive seen bins, wrapping the last to the
    # first, minus the seen bin itself.
    d = np.diff(np.concatenate([idx, [idx[0] + n]])) - 1
    return int(d.max())


class ILC:
    """P-type iterative learning control over arc length.

    The car drives the same 28.6 m every lap, so the error at a given ``s`` is
    not noise -- it is a repeatable consequence of the plant's lag against that
    piece of geometry, and a feedback tracker re-discovers it from scratch
    every lap.  Two tables over ``s``, both identity until a lap has been
    driven:

    ``kappa_ff``  a curvature offset ADDED to the reference curvature the
                  optimiser tracks.  Positive reference curvature is a left
                  turn, so a lap spent left of the line (``e_lat > 0``) is
                  corrected by a negative offset: ``kff -= gain * mean(e)``.
                  It is clamped twice -- to ``k_max`` outright, and to the
                  curvature that would shift the car ``shift_max_m`` over
                  ``shift_len_m`` of preview, which is the tighter of the two
                  at the shipping numbers.
    ``v_scale``   a multiplier on the speed CEILING.  Driven by the worst
                  corridor clearance seen in the bin, not by the error: the
                  thing that ends a race is contact, and clearance is the
                  quantity that measures it.  Down fast where the lap came
                  closer than ``clear_target``, up slowly and only where it was
                  comfortably clear.

    Both are passed through a Q-filter (a circular box average over
    ``q_span_m``) before they are used, which is what keeps a bin-to-bin
    ripple -- the classic ILC failure -- from being learned and amplified.
    The learning gains are well under 1, so the iteration is a contraction.
    """

    def __init__(self, total_s, bin_m=0.25, k_gain=0.30, q_span_m=0.50,
                 k_max=0.15, shift_max_m=0.08, shift_len_m=1.50,
                 clear_target=0.30, clear_band=0.10, clear_loss=0.05,
                 v_down=1.0, v_up=0.005, v_enable=True,
                 v_lo=0.85, v_hi=1.05, leak=0.02, min_count=1, min_cover=0.60,
                 max_gap=0.25, store=''):
        self.total_s = float(total_s)
        self.bin_m = float(bin_m)
        self.nb = max(4, int(round(self.total_s / self.bin_m)))
        self.k_gain = float(k_gain)
        self.k_clamp = min(float(k_max),
                           2.0 * float(shift_max_m) / max(float(shift_len_m), 1e-3) ** 2)
        self.clear_target = float(clear_target)
        self.clear_band = float(clear_band)
        # The speed table must punish clearance the TRACKER gives away, not
        # clearance the LINE never had.  Measured: with a flat 0.30 m target on
        # `porto_mesh_mc0.63_ay6.75` -- whose corridor is under 0.30 m at the
        # hairpins by construction -- every bin scored "tight" every lap and
        # the multiplicative cut ratcheted the whole lap to the 0.85 floor:
        # error still converged (p50 0.077 -> 0.013) but the lap went
        # 7.11 -> 8.28 s.  The target is therefore capped at the line's own
        # clearance less `clear_loss`, which is zero-cost wherever the line is
        # already tight and still defends the absolute floor where it is not.
        self.clear_loss = float(clear_loss)
        self.v_enable = bool(v_enable)
        self.nominal_clear = None
        self.v_down, self.v_up = float(v_down), float(v_up)
        self.v_lo, self.v_hi = float(v_lo), float(v_hi)
        # Leakage.  A pure integrator over a table with a hard clamp winds up:
        # measured without it, the steering table converged to |e| p50 0.032 m
        # by lap 11 and then drifted back to 0.075 and hunted, with most bins
        # pinned at the clamp.  `kff <- (1 - leak) * kff - gain * e` is the
        # standard fix and makes the iteration a strict contraction.
        self.leak = float(leak)
        self.min_count = int(min_count)
        # A lap at 10 Hz is only ~70 poses over 28.6 m, i.e. one sample every
        # 0.41 m.  0.1 m bins are therefore NOT observable in one lap however
        # attractive they look: measured, 286 bins saw 68 samples and the
        # update never fired at all.  The table is kept fine-grained and the
        # gaps are interpolated across instead -- the Q-filter smooths over
        # 0.5 m anyway, so nothing finer than that survives regardless.
        # ...and a FRAGMENT of a lap is not a lap.  `_fill` interpolates the
        # unseen bins circularly, which is right for the 10 Hz sampling gaps it
        # was written for and a lie for a lap cut short by a collision
        # teleport: measured, a 20%-covered fragment carrying a constant
        # +0.30 m error drove all 114 bins to the -0.0711 1/m clamp, i.e. a
        # full-clamp curvature bias round the entire lap, from one collision.
        # `invalidate()` is the primary defence but it depends on the encoder
        # noticing the teleport; this is the one that does not.
        self.min_cover = float(min_cover)
        # Largest unseen arc, as a fraction of the lap, that is still a lap.
        # 0.25 against a real lap's 3-4 bins (0.03) and the measured fragment's
        # 0.80: the two populations are two orders of magnitude apart, so this
        # is not a tuned number.
        self.max_gap = float(max_gap)
        self.store = str(store)
        span = max(1, int(round(float(q_span_m) / self.bin_m)))
        self.q = np.ones(2 * span + 1) / (2 * span + 1)
        self.kappa_ff = np.zeros(self.nb)
        self.v_scale = np.ones(self.nb)
        self.laps = 0
        self.valid = True
        self._reset_lap()
        if self.store:
            self.load()

    # .................................................................. state
    def _reset_lap(self):
        self.sum_e = np.zeros(self.nb)
        self.cnt = np.zeros(self.nb)
        self.min_clear = np.full(self.nb, np.inf)

    def invalidate(self):
        """This lap is not evidence (a teleport, a stuck escape, a restart)."""
        self.valid = False

    def set_nominal_clearance(self, path, body_m=0.25):
        """The clearance the LINE itself has, per bin, with the car ON it."""
        if not path.has_corridor:
            self.nominal_clear = None
            return None
        c = np.array([corridor_clearance(path, i, 0.0, body_m)
                      for i in range(path.n)])
        b = (path.s / self.bin_m).astype(int) % self.nb
        out = np.full(self.nb, np.inf)
        np.minimum.at(out, b, c)
        self.nominal_clear = self._fill(np.where(np.isfinite(out), out, 0.0),
                                        np.isfinite(out))
        return self.nominal_clear

    # .................................................................. use
    def _bins(self, s):
        return (np.asarray(s) / self.bin_m).astype(int) % self.nb

    def kappa_offset(self, s):
        return self.kappa_ff[self._bins(s)]

    def speed_scale(self, s):
        return self.v_scale[self._bins(s)]

    # .................................................................. learn
    def observe(self, s, e_lat, clearance=None):
        b = int(s / self.bin_m) % self.nb
        self.sum_e[b] += float(e_lat)
        self.cnt[b] += 1.0
        if clearance is not None and math.isfinite(clearance):
            self.min_clear[b] = min(self.min_clear[b], float(clearance))

    def _fill(self, a, seen):
        """Circularly interpolate a per-bin quantity across the unseen bins.

        Zero-filling would be a lie in the other direction: an unseen bin is
        not a bin with no error, it is a bin the 10 Hz sensor did not land in.
        """
        if seen.all():
            return np.asarray(a, float)
        i = np.flatnonzero(seen)
        if not len(i):
            return np.zeros(self.nb)
        x = np.arange(self.nb, dtype=float)
        # wrap one sample round each end so the interpolation is periodic
        xp = np.concatenate([i - self.nb, i, i + self.nb]).astype(float)
        fp = np.concatenate([a[i], a[i], a[i]])
        return np.interp(x, xp, fp)

    def _q_filter(self, a):
        k = len(self.q) // 2
        pad = np.concatenate([a[-k:], a, a[:k]])
        return np.convolve(pad, self.q, mode='valid')

    def finish_lap(self):
        """Fold one lap's measurements into the tables.  Call on an s wrap."""
        seen = self.cnt >= self.min_count
        # Two questions, and `min_cover * nb` used to answer them with one
        # number that only works at one sensor period.
        #
        # *How densely was the lap sampled?*  Coverage is bounded by the SAMPLE
        # count, not by the bin count, and the sample count is rate dependent:
        # ~77 poses a lap at 0.104 s against ~91 at 0.088 s, over 114 bins on
        # Porto, so the achievable coverage is 0.67 on the dev host and 0.80 in
        # the submission image -- against a 0.60 gate. A normal lap on a slow
        # host therefore sits a few dropped poses away from silently learning
        # nothing, and on a fast one the gate can never bind. Judge density
        # against what the sampling could reach.
        #
        # *Was it a whole lap?*  That is what the gate was really for, and
        # density cannot answer it: a fragment is dense in its own bins. The
        # rate-free test is the largest circular RUN of unseen bins -- a real
        # lap at any period leaves gaps of a few bins (0.6-0.7 m of travel
        # between poses against 0.25 m bins), while a lap cut short by a
        # teleport leaves one gap the size of the part it never drove.
        reach = min(float(self.nb), float(self.cnt.sum()))
        if not self.valid or seen.sum() < self.min_cover * reach \
                or _max_circular_gap(seen) > self.max_gap * self.nb:
            self._reset_lap()
            self.valid = True
            return None
        mean_e = self._fill(np.where(seen, self.sum_e / np.maximum(self.cnt, 1.0),
                                     0.0), seen)
        kff = (1.0 - self.leak) * self.kappa_ff - self.k_gain * mean_e
        kff = np.clip(self._q_filter(kff), -self.k_clamp, self.k_clamp)

        vs = self.v_scale.copy()
        tgt = np.full(self.nb, self.clear_target)
        if self.nominal_clear is not None:
            tgt = np.minimum(tgt, self.nominal_clear - self.clear_loss)
        clr = self._fill(np.where(np.isfinite(self.min_clear), self.min_clear,
                                  0.0), np.isfinite(self.min_clear))
        tight = clr < tgt
        roomy = clr > tgt + self.clear_band
        # THIS LAP'S multiplier, then the table.  Eroding the cumulative table
        # instead (the first version) walks the minimum two bins further round
        # the lap every lap: after 29 laps one tight hairpin had pulled all 114
        # bins to the 0.85 floor and the lap time went 7.11 -> 8.28 s while the
        # error converged.  Eroding the INCREMENT spreads a cut to the metre
        # around the tight spot and no further, and a bin that is not tight
        # this lap gets a multiplier of exactly 1.
        m = np.ones(self.nb)
        m[tight] = 1.0 - self.v_down * (tgt[tight] - clr[tight])
        m[roomy] = 1.0 + self.v_up
        m = np.minimum.reduce([np.roll(m, k) for k in (-2, -1, 0, 1, 2)])
        m = self._q_filter(m)
        if self.v_enable:
            vs = np.clip(vs * m, self.v_lo, self.v_hi)

        stats = dict(lap=self.laps + 1,
                     mean_abs_e=float(np.abs(mean_e[seen]).mean()),
                     max_abs_e=float(np.abs(mean_e[seen]).max()),
                     kff_max=float(np.abs(kff).max()),
                     d_kff=float(np.abs(kff - self.kappa_ff).max()),
                     v_scale_min=float(vs.min()), v_scale_max=float(vs.max()),
                     v_target_min=float(tgt.min()),
                     cover=float(seen.mean()),
                     min_clear=float(clr.min()))
        self.kappa_ff, self.v_scale = kff, vs
        self.laps += 1
        self._reset_lap()
        if self.store:
            self.save()
        return stats

    # .................................................................. disk
    def store_path(self):
        return self.store if self.store.endswith('.npz') else self.store + '.npz'

    def save(self):
        """Write the tables ATOMICALLY.

        `np.savez` straight onto the live path is a torn write waiting to
        happen: this is called from inside the control callback at every lap
        boundary, and a run killed there (which is what `stop_run.sh` does)
        leaves a half-written zip that the NEXT run then tries to load --
        measured, `BadZipFile` on a truncated file and `EOFError` on an empty
        one, neither of which `load` used to catch, both of which unwound
        `MPC3Tracker.__init__` and left the race with no tracker at all.
        Writing to a pid-unique temporary and `os.replace`-ing it also makes
        two concurrent runs of the same line each other's problem no longer.
        """
        final = self.store_path()
        tmp = f'{final}.{os.getpid()}.tmp.npz'
        try:
            np.savez(tmp, kappa_ff=self.kappa_ff, v_scale=self.v_scale,
                     laps=self.laps, total_s=self.total_s, bin_m=self.bin_m)
            os.replace(tmp, final)
        except OSError:
            try:
                os.unlink(tmp)
            except OSError:
                pass

    def load(self):
        """Restore a table only if it describes THIS line at THIS resolution.

        A stored table is a CONVENIENCE; the car must start regardless.  So the
        failure mode here is `return False` for anything at all -- a truncated
        zip, an empty file, a table from another numpy, a pickled surprise --
        and never an exception, because this runs in the node's constructor.
        """
        try:
            z = np.load(self.store_path())
            if (len(z['kappa_ff']) != self.nb
                    or abs(float(z['bin_m']) - self.bin_m) > 1e-9
                    or abs(float(z['total_s']) - self.total_s) > 0.5):
                return False
            kff = np.clip(np.asarray(z['kappa_ff'], float),
                          -self.k_clamp, self.k_clamp)
            vs = np.clip(np.asarray(z['v_scale'], float), self.v_lo, self.v_hi)
            laps = int(z['laps'])
        except Exception:                                       # noqa: BLE001
            return False
        # Commit only once everything parsed: a half-restored table is worse
        # than none, and `load` is allowed to be a no-op.
        if not (np.isfinite(kff).all() and np.isfinite(vs).all()):
            return False
        self.kappa_ff, self.v_scale, self.laps = kff, vs, laps
        return True


class MPC3:
    """Gauss-Newton over a steering-rate sequence, with a coupled speed plan.

    Decision variable ``u`` is the steering RATE at each *free* step, boxed at
    the actuator's 3.2 rad/s.  State is ``(e_lat, e_yaw, delta)`` in the
    path-relative frame plus the speed ``v``, which is planned rather than
    optimised:

        e_lat'  = sin(e_yaw)                                    (per metre)
        e_yaw'  = g*sat(tan(delta)/L) - k_ref cos(e_yaw)/(1 - k_ref e_lat)
        delta'  = u dt                       |u| <= 3.2, |delta| <= cap

    The speed enters through the arc length of each step and through the grip
    saturation, and is held fixed inside one Gauss-Newton solve so the Jacobian
    stays the cheap 3-state one.  ``outer`` passes alternate
    speed-plan -> steering-plan, which is a fixed point that converges in two
    (measured: the third pass moves the first command by < 1e-3 rad).
    """

    def __init__(self, path: RefPath, **cfg):
        self.path = path
        p = dict(DEFAULTS)
        p.update({k: v for k, v in cfg.items() if v is not None})
        self.cfg = p
        self.N = int(p['horizon'])
        self.dt = float(p['step_dt'])
        # `period` is written by `RateControl.update` every pose; the setter
        # below re-derives `nd` from it when `nd_mode` is 'period'.
        self._period = float(RATE_NOMINAL)
        self.nd = self._nd_for(float(p.get('nd_delay_s', 0.0) or 0.0)
                               or p['delay_s'])
        self.u = np.zeros(self.N)
        self.v_plan = None            # last speed plan, warm start for ds
        self.v_ceiling = None         # the ceiling the throttle is handed
        self.v_ceiling_idx = None     # the reference indices it is built on
        self.e_pred = np.zeros(self.N + 1)
        self.k_cmd = np.zeros(self.N)
        self.last_cost = float('nan')
        self.last_iters = 0
        # Opt-in extras; both are None in the shipping configuration and every
        # code path that reads them is a no-op while they are.
        self.ilc = None               # see `ILC`; set by the node / harness
        # The last angle actually PUBLISHED (after the opt-in output filters),
        # and the measured sensor period `RateControl` last saw.  Both are
        # inert unless one of `w_dcmd` / `cmd_tau` / `cmd_notch` is non-zero.
        self.last_cmd = None
        # The published angle BEFORE `cmd_tau`/`cmd_notch`.  Only read when the
        # rollout models the rack as a lag, where it is the plan command the
        # free steps continue from.
        self.last_cmd_raw = None
        self._apply = (0, 0.0)        # publish knot + fraction, set in solve()
        # (command plan, its Jacobian) from the last rollout; None unless
        # `steer_tau` / `model_cmd_tau` put a lag in the chain.
        self._pub = None


    # ..................................................................
    def _nd_for(self, delay):
        """Committed steps for a delay in seconds, boxed to the horizon."""
        return max(0, min(self.N - 2, int(round(float(delay) / self.dt))))

    def _refresh_nd(self):
        """Re-derive `nd` from whichever law `nd_mode` selects."""
        pinned = float(self.cfg.get('nd_delay_s', 0.0) or 0.0)
        if pinned > 0.0:
            self.nd = self._nd_for(pinned)
        elif str(self.cfg.get('nd_mode', 'fixed')) == 'period':
            self.nd = self._nd_for(self._period + self.cfg['dead_tick_c'])
        else:
            self.nd = self._nd_for(self.cfg['delay_s'])

    @property
    def period(self):
        """The measured sensor period, as last seen by `RateControl`."""
        return self._period

    @period.setter
    def period(self, p):
        self._period = float(p)
        if (str(self.cfg.get('nd_mode', 'fixed')) == 'period'
                or float(self.cfg.get('nd_delay_s', 0.0) or 0.0) > 0.0):
            self._refresh_nd()

    # ..................................................................
    def set_rate(self, delay_s=None, apply_ahead_s=None, thr_lead_s=None):
        """Re-point the period-dependent constants at a measured period.

        Only ``delay_s`` changes the solver's STRUCTURE (``nd``, the number of
        horizon steps whose steering is already committed).  ``self.u`` keeps
        its length ``N``, so a change in ``nd`` mid-run only re-slices the warm
        start, which is what should happen.
        """
        if delay_s is not None:
            self.cfg['delay_s'] = float(delay_s)
            self._refresh_nd()
        if apply_ahead_s is not None:
            self.cfg['apply_ahead_s'] = float(apply_ahead_s)
        if thr_lead_s is not None:
            self.cfg['thr_lead_s'] = float(thr_lead_s)
        return self.nd

    # ..................................................................
    def _indices(self, i0, v):
        """Reference indices and arc lengths, from the warm speed plan."""
        P = self.path
        N = self.N
        if self.v_plan is None or len(self.v_plan) != N + 1:
            self.v_plan = np.full(N + 1, max(v, 0.5))
        ds = np.maximum(self.v_plan[:N] * self.dt, self.cfg['min_step_m'])
        s_acc = np.concatenate([[0.0], np.cumsum(ds)])
        idx = (i0 + (s_acc / max(P.spacing, 1e-3)).astype(int)) % P.n
        return idx, ds

    # ..................................................................
    def _speed_plan(self, idx, ds, v0):
        """A feasible speed trajectory over the horizon.

        Ceilings, in order: the offline profile; the profile cut by the
        predicted lateral error; and the grip budget at the curvature the
        current plan is ASKING for (not the reference's curvature -- that is the
        whole point: a car running wide has to turn harder than the line does,
        and must pay for it in speed).  Then a backward pass at ``a_brake`` so
        every ceiling ahead is reachable, and a forward pass at ``a_accel`` and
        ``a_brake`` so the trajectory starts from the speed the car HAS.
        """
        c = self.cfg
        P = self.path
        N = self.N
        v_lim = P.v_prof[idx].astype(float).copy()
        if self.ilc is not None:
            v_lim *= self.ilc.speed_scale(P.s[idx])

        e = np.abs(self.e_pred[:N + 1])
        mod = np.clip(1.0 - c['k_e'] * np.maximum(0.0, e - c['e_free']),
                      c['v_mod_min'], 1.0)
        sc = float(c.get('e_cut_accel_scale', 1.0))
        if sc != 1.0:
            # One-sided: weaken the cut only where the profile is accelerating.
            ok = P.a_prof[idx] > 0.0
            kg = float(c.get('e_cut_accel_kappa', 0.0))
            if kg > 0.0:
                ok &= np.abs(P.kappa[idx]) < kg
            mod = np.where(ok, 1.0 - sc * (1.0 - mod), mod)
        v_lim *= mod

        # The grip ceiling.  It is built from the REFERENCE curvature plus a
        # BOUNDED allowance for the correction the plan is adding on top -- not
        # from the plan's raw commanded curvature.
        #
        # Measured (Sept 12, live mc0.60 log replayed through this solver): the
        # raw version loses the whole fast straight.  The steering-rate plan may
        # swing the angle by up to 0.19 rad in one step and swing it back, and
        # such a 60 ms transient is invisible in the trajectory but sets
        # `sqrt(ay_budget/0.52) = 3.8 m/s` at that step -- which the backward
        # brake pass then propagates all the way back to the car.  At s 3.5 the
        # profile asked 6.60 m/s and the plan commanded 3.85, on a straight with
        # 0.034 m of lateral error.  The deficit is self-sustaining: a slower
        # car has shorter steps, so the horizon stays inside the corner-entry
        # region and the blip never leaves it.
        #
        # Only the EXCESS over the reference is a real statement about grip the
        # profile has not already paid for, and it is clipped so that no single
        # solver transient can cost more than `k_excess_max`.  The committed
        # steps are excluded outright: their steering is history (it may be
        # unwinding a previous correction) and is not a demand at all.
        k_ref = np.abs(P.kappa[idx])
        k_ex = np.zeros(N + 1)
        nd = self.nd
        if N > nd:
            k_ex[nd:N] = np.clip(np.abs(self.k_cmd[nd:N]) - k_ref[nd:N],
                                 0.0, c['k_excess_max'])
        kd = k_ref + k_ex
        with np.errstate(divide='ignore'):
            v_grip = np.where(kd > 1e-3, np.sqrt(c['ay_budget'] / np.maximum(kd, 1e-9)),
                              np.inf)
        v_lim = np.minimum(v_lim, v_grip)

        # Beyond the horizon: the profile still has to be met, and the horizon
        # is only ~1.2 s.  Seed the backward pass with the fastest speed that
        # can still make every planned speed over the next few metres.
        tail = idx[N]
        n_tail = max(2, int(4.0 / max(P.spacing, 1e-3)))
        j = (tail + np.arange(n_tail)) % P.n
        d_ahead = np.arange(n_tail) * P.spacing
        v_lim[N] = min(v_lim[N], float(np.sqrt(
            P.v_prof[j] ** 2 + 2.0 * c['a_brake'] * d_ahead).min()))

        for k in range(N - 1, -1, -1):
            v_lim[k] = min(v_lim[k], math.sqrt(v_lim[k + 1] ** 2
                                               + 2.0 * c['a_brake'] * ds[k]))
        # Two trajectories come out of here and they are NOT the same thing.
        #
        # `v_lim` is the CEILING: the fastest the car may go at each step and
        # still make every planned speed ahead.  That is what the throttle
        # should be handed.  The simulator's throttle is a speed servo with
        # +5.0 / -4.5 m/s2 of authority that saturates as soon as the setpoint
        # is ~0.3 m/s off, so it closes the gap by itself and eases off on
        # arrival; handing it an acceleration-feasible setpoint instead just
        # rate-limits the command for no gain.  Measured on the live mc0.60
        # log: commanding the feasible value left the setpoint 1.25 m/s under
        # the profile on the straight even after the grip-cut fix.
        #
        # `v` is the FEASIBLE trajectory, integrated from the speed the car
        # actually has.  The horizon's arc lengths and the delivered-curvature
        # saturation must use this one, because that is where the car will be.
        v = np.empty(N + 1)
        v[0] = max(float(v0), 0.3)
        for k in range(N):
            hi = math.sqrt(v[k] ** 2 + 2.0 * c['a_accel'] * ds[k])
            lo = math.sqrt(max(v[k] ** 2 - 2.0 * c['a_brake'] * ds[k], 0.04))
            v[k + 1] = max(lo, min(v_lim[k + 1], hi))
        self.v_ceiling = v_lim
        # The reference indices `v_lim` is indexed BY.  `solve` recomputes
        # `idx` after this call (the speed plan changes the arc steps), so the
        # two index sets are not the same array, and comparing the ceiling
        # against `P.v_prof[solve's idx]` would compare two different points.
        # `thr_source` reads the profile through THIS one.
        self.v_ceiling_idx = idx
        return v

    # ..................................................................
    def _bounds(self, idx):
        """One-sided corridor hinges, left and right, at each horizon step."""
        P = self.path
        hw = 0.5 * vehicle.CAR_WIDTH + self.cfg['barrier_margin']
        if not P.has_corridor:
            return (np.full(self.N, -np.inf), np.full(self.N, np.inf))
        hi = P.w_left[idx[1:]] - hw
        lo = -(P.w_right[idx[1:]] - hw)
        return lo, hi


    # ..................................................................
    def _rollout(self, u, z0, ds, kap, v, cap, committed, jac=False):
        """Path-relative rollout over the horizon.

        ``u`` holds the FREE steering rates only.  Over the first ``nd`` steps
        the steering angle is ``committed`` -- whatever the commands already in
        flight will produce -- because the dead time makes it data, not a
        decision.  Forcing it here, rather than shifting the pose forward and
        pretending the horizon starts later, keeps the reference the free steps
        are judged against correct in TIME and keeps the delay in the structure
        instead of in the cost.
        """
        N, nd = self.N, self.nd
        nf = N - nd
        c = self.cfg
        g, knee, ayd = c['yaw_gain'], c['sat_knee'], c['ay_deliver']
        e = np.empty(N + 1)
        psi = np.empty(N + 1)
        d = np.empty(N + 1)
        e[0], psi[0], d[0] = z0
        S = np.zeros((3, nf)) if jac else None
        Sout = np.empty((N + 1, 3, nf)) if jac else None
        if jac:
            Sout[0] = 0.0
        kc = np.empty(N)
        # OPT-IN rack lag chain (`steer_tau` / `model_cmd_tau`, see DEFAULTS).
        # `dc` is the plan's COMMAND (what would be published), `fl` the same
        # after the `cmd_tau` output filter, and `d` the rack angle that
        # actually steers the car.  With both taus 0 the three collapse into
        # one and the original branch below runs untouched.
        tau_a = float(c['steer_tau'])
        tau_c = float(c['cmd_tau']) if float(c['model_cmd_tau']) else 0.0
        lag_on = tau_a > 0.0 or tau_c > 0.0
        dc = fl = Sdc_out = None
        Ea = math.exp(-self.dt / tau_a) if tau_a > 0.0 else 0.0
        Ec = math.exp(-self.dt / tau_c) if tau_c > 0.0 else 0.0
        if lag_on:
            dc = np.empty(N + 1)
            fl = np.empty(N + 1)
            Sdc = np.zeros(nf) if jac else None
            Sfl = np.zeros(nf) if jac else None
            Sdc_out = np.empty((N + 1, nf)) if jac else None
        for k in range(N):
            if lag_on and k == nd:
                # The command in flight when the free steps begin: the last one
                # published (raw before the output filter, filtered after it).
                # `d[nd]` -- the angle the rack is predicted to hold -- is the
                # fallback for the first solve of a run.
                h_raw = (self.last_cmd_raw if self.last_cmd_raw is not None
                         else (self.last_cmd if self.last_cmd is not None
                               else d[nd]))
                h_f = self.last_cmd if self.last_cmd is not None else h_raw
                dc[:nd + 1] = h_raw
                fl[:nd + 1] = h_f
                if jac:
                    Sdc_out[:nd + 1] = 0.0
            ek, pk, dk = e[k], psi[k], d[k]
            kr = kap[k]
            dsk = ds[k]
            den = 1.0 - kr * ek
            if den < 0.2:
                den = 0.2
                dden_de = 0.0
            else:
                dden_de = -kr
            sp, cp = math.sin(pk), math.cos(pk)
            k_max = ayd / max(v[k] * v[k], 0.25)
            kach, dkach = _sat_curvature(dk, k_max, knee, g)
            kc[k] = math.tan(dk) / vehicle.WHEELBASE
            e[k + 1] = ek + sp * dsk
            psi[k + 1] = wrap(pk + (kach - kr * cp / den) * dsk)
            raw = None
            dc_clip = False
            if k < nd:
                d[k + 1] = committed[k] if committed is not None else dk
            elif lag_on:
                raw = dc[k] + u[k - nd] * self.dt
                dcn = min(cap, max(-cap, raw))
                dc_clip = (raw != dcn)
                dc[k + 1] = dcn
                sl = (dcn - dc[k]) / self.dt
                if tau_c > 0.0:
                    fn = dcn - sl * tau_c + (fl[k] - dc[k] + sl * tau_c) * Ec
                else:
                    fn = dcn
                fl[k + 1] = fn
                if tau_a > 0.0:
                    slf = (fn - fl[k]) / self.dt
                    an = fn - slf * tau_a + (dk - fl[k] + slf * tau_a) * Ea
                else:
                    an = fn
                raw = an
                d[k + 1] = min(cap, max(-cap, an))
            else:
                raw = dk + u[k - nd] * self.dt
                d[k + 1] = min(cap, max(-cap, raw))
            if jac:
                a10 = -(-kr * cp) * dden_de / (den * den) * dsk
                a11 = 1.0 + (kr * sp / den) * dsk
                a12 = dkach * dsk
                Sn = np.empty((3, nf))
                Sn[0] = S[0] + cp * dsk * S[1]
                Sn[1] = a10 * S[0] + a11 * S[1] + a12 * S[2]
                if k < nd:
                    Sn[2] = 0.0            # committed: independent of u
                elif lag_on:
                    if dc_clip:
                        Sdc_n = np.zeros(nf)
                    else:
                        Sdc_n = Sdc.copy()
                        Sdc_n[k - nd] += self.dt
                    Ss = (Sdc_n - Sdc) / self.dt
                    if tau_c > 0.0:
                        Sf_n = Sdc_n - tau_c * Ss + Ec * (Sfl - Sdc
                                                          + tau_c * Ss)
                    else:
                        Sf_n = Sdc_n
                    if tau_a > 0.0:
                        Ssf = (Sf_n - Sfl) / self.dt
                        Sa_n = Sf_n - tau_a * Ssf + Ea * (S[2] - Sfl
                                                          + tau_a * Ssf)
                    else:
                        Sa_n = Sf_n
                    Sn[2] = 0.0 if raw != d[k + 1] else Sa_n
                    Sdc, Sfl = Sdc_n, Sf_n
                    Sdc_out[k + 1] = Sdc_n
                elif raw != d[k + 1]:
                    Sn[2] = 0.0            # against the steering cap
                else:
                    Sn[2] = S[2]
                    Sn[2, k - nd] += self.dt
                S = Sn
                Sout[k + 1] = S
        self.k_cmd = kc
        # What `solve` publishes and what `w_dcmd` charges: the plan's COMMAND
        # when the rack is modelled as a lag, the plan's ANGLE otherwise (they
        # are the same number in that case).  Set on every rollout, including
        # the line-search ones, because `_residuals` reads it immediately.
        self._pub = (dc, Sdc_out) if lag_on else None
        return e, psi, d, Sout

    # ..................................................................
    def _residuals(self, u, e, psi, d, lo, hi, jac=None):
        """Weighted residual stack (and its Jacobian rows).

        e_lat at every step, again at the terminal, heading error, a small
        penalty on steering magnitude, one-sided hinges on each corridor wall,
        and rate/jerk on the decision variable.  No cut term, no inside/outside
        asymmetry: the geometry belongs to the line.
        """
        c = self.cfg
        N, nd = self.N, self.nd
        nf = N - nd
        r = []
        J = [] if jac is not None else None
        sw_yaw = math.sqrt(c['w_yaw'])
        sw_steer = math.sqrt(c['w_steer'])
        sw_rate = math.sqrt(c['w_rate'])
        sw_jerk = math.sqrt(c['w_jerk'])
        sw_bar = math.sqrt(c['w_barrier'])
        sw_lat = math.sqrt(c['w_lat'])
        sw_latT = math.sqrt(c['w_lat'] + c['w_lat_T'])
        sw_yawT = math.sqrt(c['w_yaw'] + c['w_yaw_T'])
        zero = np.zeros(nf)
        e_dead = float(c['e_dead'])
        for k in range(N):
            term = (k == N - 1)
            sw = sw_latT if term else sw_lat
            swy = sw_yawT if term else sw_yaw
            ek = e[k + 1]
            lat_live = True
            if e_dead > 0.0:
                if ek > e_dead:
                    ek -= e_dead
                elif ek < -e_dead:
                    ek += e_dead
                else:
                    ek, lat_live = 0.0, False
            r.append(sw * ek)
            r.append(swy * psi[k + 1])
            r.append(sw_steer * d[k + 1])
            hu = e[k + 1] - hi[k]
            hd = lo[k] - e[k + 1]
            r.append(sw_bar * (hu if hu > 0 else 0.0))
            r.append(sw_bar * (hd if hd > 0 else 0.0))
            if jac is not None:
                Se, Sp, Sd = jac[k + 1][0], jac[k + 1][1], jac[k + 1][2]
                J.append(sw * Se if lat_live else zero)
                J.append(swy * Sp)
                J.append(sw_steer * Sd)
                J.append(sw_bar * Se if hu > 0 else zero)
                J.append(-sw_bar * Se if hd > 0 else zero)
        for i in range(nf):
            r.append(sw_rate * u[i] * self.dt)
            du = u[i] - (u[i - 1] if i else u[0])
            r.append(sw_jerk * du * self.dt)
            if jac is not None:
                ek = np.zeros(nf)
                ek[i] = 1.0
                dk = np.zeros(nf)
                dk[i] = 1.0
                if i:
                    dk[i - 1] = -1.0
                J.append(sw_rate * self.dt * ek)
                J.append(sw_jerk * self.dt * dk)
        wdc = float(c['w_dcmd'])
        if wdc > 0.0 and self.last_cmd is not None:
            # Move suppression ACROSS frames.  `d` is the plan's steering
            # angle, so the angle that would be published this frame is the
            # same interpolation `solve` publishes, and its sensitivity to `u`
            # is the same interpolation of the rollout's own Jacobian rows.
            ka, fr = self._apply
            # With a modelled rack lag the published number is the plan's
            # COMMAND, so that is what this residual has to charge.
            pub, Spub = self._pub if self._pub is not None else (d, None)
            if Spub is None and jac is not None:
                Spub = jac[:, 2]
            if ka >= N:
                da = pub[N]
                Sa = Spub[N] if jac is not None else None
            else:
                da = pub[ka] + fr * (pub[ka + 1] - pub[ka])
                Sa = (Spub[ka] + fr * (Spub[ka + 1] - Spub[ka])
                      if jac is not None else None)
            sw_dc = math.sqrt(wdc)
            r.append(sw_dc * (da - self.last_cmd))
            if jac is not None:
                J.append(sw_dc * Sa)
        r = np.asarray(r)
        return (r, np.asarray(J)) if jac is not None else (r, None)

    # ..................................................................
    def solve(self, e_lat, e_yaw, delta0, i0, v0, cap=None, committed=None):
        """One control cycle.  Call once per new pose.

        ``delta0`` is the actuator angle at the pose stamp, ``committed`` the
        steering angle the in-flight commands produce at each of the first
        ``nd`` horizon steps (both from ``Actuator``).  Returns
        ``(command_rad, info)``; ``info['v_cmd']`` is the speed the throttle
        should be asked for, which is the *plan's*, not the profile's.
        """
        c = self.cfg
        N, nd = self.N, self.nd
        nf = N - nd
        # A non-finite state is not a control problem, and it must not be
        # silently absorbed: `_rollout` clips with `min(cap, max(-cap, raw))`,
        # and Python's max returns `-cap` for a NaN, so a NaN steering angle
        # would be published as FULL RIGHT LOCK. The caller holds its last good
        # command instead.
        if not all(math.isfinite(float(q))
                   for q in (e_lat, e_yaw, delta0, v0)):
            raise ValueError(
                f'non-finite state: e_lat={e_lat} e_yaw={e_yaw} '
                f'delta0={delta0} v0={v0}')
        if committed is not None and not np.isfinite(
                np.asarray(committed, dtype=float)).all():
            raise ValueError(f'non-finite committed steering: {committed}')
        cap = vehicle.MAX_STEER if cap is None else min(cap, vehicle.MAX_STEER)
        rate = float(c['steer_rate'])
        z0 = (float(e_lat), float(wrap(e_yaw)),
              float(np.clip(delta0, -vehicle.MAX_STEER, vehicle.MAX_STEER)))
        if committed is not None:
            committed = np.clip(np.asarray(committed, dtype=float)[:nd],
                                -vehicle.MAX_STEER, vehicle.MAX_STEER)

        u = np.clip(np.roll(self.u, -1)[:nf], -rate, rate)
        if nf > 1:
            u[-1] = u[-2]

        # Where the published angle is read out of the plan.  Computed here,
        # before the solve, because `w_dcmd` costs exactly that angle.
        if str(c['lead_mode']).strip().lower() == 'spatial':
            # A lead in METRES, walked along the warm speed plan from the first
            # free step.  Rate-invariant by construction; `apply_ahead_s` (and
            # therefore `rate_adaptive`'s `apply` term) is not read at all.
            lead = max(float(c['lead_min_m']),
                       float(c['lead_m0']) + float(c['lead_k']) * float(v0))
            vw = (self.v_plan if (self.v_plan is not None
                                  and len(self.v_plan) == N + 1)
                  else np.full(N + 1, max(float(v0), 0.5)))
            acc = 0.0
            _f = float(N)
            for _j in range(nd, N):
                dsj = max(float(vw[_j]) * self.dt, float(c['min_step_m']))
                if acc + dsj >= lead:
                    _f = _j + (lead - acc) / dsj
                    break
                acc += dsj
            _f = max(_f, nd + float(c['apply_floor_steps']))
        elif float(c['apply_lead_steps']) >= 0.0:
            # OPT-IN: the readout lead, decoupled from `nd` (see DEFAULTS).
            _f = nd + float(c['apply_lead_steps'])
        else:
            _f = nd + max(float(c['apply_floor_steps']),
                          float(c['apply_ahead_s']) / self.dt)
        _k = min(int(_f), N)
        self._apply = (_k, _f - _k)

        cost = float('nan')
        it = 0
        e = psi = d = None
        idx = ds = kap = lo = hi = None
        for _ in range(max(1, int(c['outer']))):
            # speed plan -> arc lengths -> reference, then the steering solve
            idx, ds = self._indices(i0, v0)
            self.v_plan = self._speed_plan(idx, ds, v0)
            idx, ds = self._indices(i0, v0)
            kap = self.path.kappa[idx[:N]]
            if self.ilc is not None:
                # The learned offset is added to the REFERENCE the rollout is
                # judged against, not to the command: the solver then plans a
                # feasible, rate-limited way of delivering it, and every cap
                # (`cap`, the grip saturation, the corridor barrier) still
                # applies on top.
                kap = kap + self.ilc.kappa_offset(self.path.s[idx[:N]])
            lo, hi = self._bounds(idx)

            e, psi, d, S = self._rollout(u, z0, ds, kap, self.v_plan, cap,
                                         committed, jac=True)
            r, J = self._residuals(u, e, psi, d, lo, hi, jac=S)
            cost = float(r @ r)
            lam = float(c['lam0'])
            step = np.zeros(nf)
            for it in range(int(c['iters'])):
                H = J.T @ J
                gr = J.T @ r
                # Damp the DIAGONAL of H, not a trace-scaled identity: the
                # barrier makes the trace explode exactly when a violation
                # needs correcting (mpc2 learned this the hard way).
                dg = np.diag(H).copy()
                dg[dg < 1e-9] = 1e-9
                for _ in range(6):
                    try:
                        step = np.linalg.solve(H + lam * np.diag(dg), -gr)
                    except np.linalg.LinAlgError:
                        lam *= 10.0
                        continue
                    u_try = np.clip(u + step, -rate, rate)
                    e2, psi2, d2, _ = self._rollout(u_try, z0, ds, kap,
                                                    self.v_plan, cap, committed)
                    r2, _ = self._residuals(u_try, e2, psi2, d2, lo, hi)
                    c2 = float(r2 @ r2)
                    if c2 < cost:
                        u, cost = u_try, c2
                        lam = max(lam * 0.4, 1e-6)
                        break
                    lam *= 6.0
                else:
                    break
                e, psi, d, S = self._rollout(u, z0, ds, kap, self.v_plan, cap,
                                             committed, jac=True)
                r, J = self._residuals(u, e, psi, d, lo, hi, jac=S)
                if float(np.max(np.abs(step))) < 1e-3:
                    break
            e, psi, d, _ = self._rollout(u, z0, ds, kap, self.v_plan, cap,
                                         committed)
            self.e_pred = e

        self.u = np.concatenate([u, np.full(nd, u[-1] if nf else 0.0)])
        self.last_cost = cost
        self.last_iters = it + 1

        # What to publish: the plan's steering angle `apply_ahead_s` after the
        # new command can first bite.  The plan is feasible by construction
        # (|u| <= the slew rate), so that is an angle the actuator can reach and
        # hold for the frame.  Handing it d[nd] -- the angle it is already
        # predicted to hold -- would make the command never change.
        k, frac = self._apply
        pub = self._pub[0] if self._pub is not None else d
        cmd = pub[N] if k >= N else pub[k] + frac * (pub[k + 1] - pub[k])
        cmd = float(np.clip(cmd, -cap, cap))
        # OPT-IN no-anticipation gate: on a straight, never publish more than
        # the line-matching angle plus a bounded correction (see DEFAULTS).
        gc = float(c.get('pub_gate_corr', 0.0) or 0.0)
        if gc > 0.0:
            P = self.path
            look = max(1, int(round(float(c['pub_gate_d'])
                                    / max(P.spacing, 1e-3))))
            j = (int(i0) + np.arange(look + 1)) % P.n
            k_ahead = float(np.abs(P.kappa[j]).max())
            if (k_ahead < float(c['pub_gate_kappa'])
                    and abs(float(e_lat)) <= float(c['pub_gate_e'])):
                lim = abs(math.atan(vehicle.WHEELBASE * k_ahead)) + gc
                cmd = float(np.clip(cmd, -lim, lim))
        cmd_raw = cmd
        self.last_cmd_raw = cmd_raw
        # Opt-in output smoothing; both are the identity at their 0.0 default.
        if self.last_cmd is not None:
            b = float(c['cmd_notch'])
            if b > 0.0:
                cmd = (1.0 - b) * cmd + b * self.last_cmd
            tau = float(c['cmd_tau'])
            if tau > 0.0:
                a = 1.0 - math.exp(-max(float(self.period), 1e-3) / tau)
                cmd = self.last_cmd + a * (cmd - self.last_cmd)
            cmd = float(np.clip(cmd, -cap, cap))
        self.last_cmd = cmd

        vp = self.v_ceiling if self.v_ceiling is not None else self.v_plan
        # `thr_source`: which curve the throttle SETPOINT is read from.  The
        # default 'ceiling' is the historical behaviour; see DEFAULTS.
        # `info['v_ceiling']` keeps reporting the real ceiling either way.
        v_src = vp
        src = str(c.get('thr_source', 'ceiling')).strip().lower()
        # Both the ceiling and the profile are read through `v_ceiling_idx`,
        # the index set the ceiling was built on -- not `solve`'s own `idx`,
        # which the speed plan has moved since.
        src_idx = (self.v_ceiling_idx if self.v_ceiling_idx is not None
                   else idx)
        if src != 'ceiling':
            v_pr = self.path.v_prof[src_idx].astype(float)
            v_src = v_pr if src == 'profile' else np.minimum(vp, v_pr)
        lead_s = float(c['thr_lead_s'])
        # Brake-phase lead: only extend where the offline profile is already
        # braking inside the base window, and then take the window MINIMUM so
        # the extension is one-sided (see DEFAULTS).
        brake_ext = False
        bl = float(c.get('thr_brake_lead_s', 0.0) or 0.0)
        if bl > 0.0:
            fk0 = min(lead_s / self.dt, N - 1e-6)
            ki0 = int(fk0)
            if float(np.min(self.path.a_prof[src_idx[:ki0 + 2]])) < 0.0:
                lead_s += bl
                brake_ext = True
        fk = min(lead_s / self.dt, N - 1e-6)
        ki = int(fk)
        v_cmd = float(v_src[ki] + (fk - ki) * (v_src[ki + 1] - v_src[ki]))
        if brake_ext or str(c['thr_lead_mode']) == 'min':
            v_cmd = float(min(v_cmd, v_src[:ki + 2].min()))
        info = dict(cost=cost, iters=self.last_iters, delta=d, e=e, psi=psi,
                    cmd_raw=cmd_raw,
                    v=self.v_plan, v_ceiling=vp, v_cmd=v_cmd, lo=lo, hi=hi,
                    kap=kap, idx=idx, ds=ds, u=u.copy())
        return cmd, info
