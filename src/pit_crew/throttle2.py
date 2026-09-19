"""A throttle law that treats the simulator's throttle as what it is: a servo.

`vehicle.ThrottleController` assumes the throttle buys acceleration, so it hands
over the steady-state throttle for the speed it wants (`0.040 * v_target`) and
waits.  The simulator does something different: the throttle sets a SPEED
SETPOINT, `SERVO_GAIN * u`, and an inner loop drives the car towards it with
roughly `a = K * (setpoint - v)`, saturating at the measured +5.0 / -4.5 m/s2.
Two consequences
decide the whole design of this module:

1. Asking for the speed you want gets you a small error and therefore a small
   acceleration.  Whatever the servo's real authority is, a feedforward law
   never spends it.  To accelerate hard you must command a setpoint well ABOVE
   the speed you want, and take it away again as you arrive.
2. A setpoint BELOW the current speed is a genuine brake, worth 4.5 m/s2, and
   it stays a forward command the whole time.  Negative throttle is
   not a brake pedal at all -- it is a reverse setpoint, and the shipping
   `brake_gain` exists only because nothing else was available.  This class
   never emits a negative command, and clamps the setpoint above a walking
   pace while the car is moving, so a mis-timed brake can slow the car but can
   never put it in reverse.

The measured plant (`tools/sim_servo.py fit`, ground-truth speed from two Porto
race logs) is the default here; pass the numbers explicitly to try another.

Same `command(v_target, v_meas, a_target)` signature as
`vehicle.ThrottleController`, so a node can select between them at runtime
without any other change.
"""

import math
import time

try:
    from pit_crew.vehicle import RATE_NOMINAL
except ImportError:                                  # running from src/pit_crew
    from vehicle import RATE_NOMINAL

# Fitted on two PORTO race logs (`tools/sim_servo.py fit`) and **re-confirmed
# on the finals track** (Sep 19 audit): on the two clean 32-lap logs
# (`race_20260918_191411` display, `_185202` Xvfb), steady-state speed per unit
# throttle (|a| < 0.5 m/s2, u > 0.02) is 23.80 and 24.03 (p25-p75 22.4-24.9 and
# 23.1-24.7). The shipping feedforward `0.040*v + 0.005` is `v/25 + 0.005`,
# within 2 % of `v/24.0`. No change warranted.
SERVO_GAIN = 24.6       # m/s of setpoint per unit of throttle (measured)
# The simulator consumes one command per telemetry tick, so this IS the sensor
# period -- a property of the MACHINE, not of the throttle law: 0.104 s on the
# dev host, 0.088 s in the submission image, 0.097 s at the organisers. It was
# written as a hard 0.09 s, which is a third machine nobody runs on. Default it
# to the one nominal constant and let whoever measures the real period re-point
# `hold` (`mpc3_core.RateControl` does, when `rate_adaptive` is on).
CMD_PERIOD = RATE_NOMINAL


class ServoThrottle:
    """Inverts the speed servo instead of feeding it a steady-state throttle.

    `k_p` is not a classical speed-loop gain, and with its default of 0 this
    is not a closed speed loop at all -- the law is then pure feedforward and
    does not read the encoder, so no sensor fault can move the command.  That
    matters here: the closed loop that burned us before, `throttle_kp`, fed the error straight to
    the actuator and went bang-bang at 10 Hz (75 collisions, car reversing).
    If `k_p` is raised here the error only ever adds a bounded `head_max` of
    extra SPEED SETPOINT on top of the target, one-sided (never below it), and
    the reachable actuator range is a forward speed between a walking pace and
    `u_max`.  The worst a wrong error can then do is hold the car `head_max`
    too fast for as long as the error persists.
    """

    def __init__(self, K=18.0, drag=0.0, tau=0.135, a_auth=4.5,
                 a_accel=5.0, k_p=0.0, k_ff=1.0, head_max=0.5,
                 lead_max=0.25, ay_limit=6.0, v_floor=1.0, u_max=0.60,
                 hold=CMD_PERIOD, servo_gain=SERVO_GAIN, predict=True,
                 brake_mode='none', brake_a_min=1.5, brake_dv=0.3,
                 brake_a_full=4.6, head_kappa=0.0):
        self.K = float(K)                # servo proportional gain, 1/s
        self.drag = float(drag)          # rigidbody linear drag, 1/s
        self.tau = float(tau)            # actuator dead time, s
        self.a_auth = float(a_auth)      # braking authority, m/s2 (measured 4.5)
        self.a_accel = float(a_accel)    # accelerating authority (measured 5.0)
        self.hold = float(hold)          # how long one command stays in force
        self.k_p = float(k_p)            # over-command per m/s of speed error
        self.head_max = float(head_max)  # m/s, cap on that over-command
        self.head_kappa = float(head_kappa)  # 1/m, 0 = off: fade head_max out by |kappa|
        self.lead_max = float(lead_max)  # m/s, cap on the dead-time lead
        self.ay_limit = float(ay_limit)  # m/s2, what the lead may spend
        self.k_ff = float(k_ff)          # fraction of the plan's ax fed forward
        self.v_floor = float(v_floor)    # never command a setpoint below this
        self.u_max = float(u_max)
        self.g = float(servo_gain)
        self.predict = bool(predict)
        # --- OPT-IN real brake ('none' reproduces every earlier number) -----
        # The simulator's VehicleController.Drive(): DriveTorque =
        # throttle * MotorTorque, and when DriveTorque is exactly 0 it applies
        # brakeTorque = MotorTorque on all four wheels -- a genuine brake worth
        # ~4.6 m/s2 locked and up to the longitudinal mu (~7.1) modulated.
        # Negative throttle is REVERSE DRIVE under our prefab's brake type, so
        # the only brake we own is the exact float 0.0, and the sim samples one
        # command per telemetry tick: it must be published as 0.0 ON the tick.
        # The law above never emits 0.0 (measured: 0.4% of samples in every
        # log, never negative) because `v_floor` holds a walking-pace setpoint,
        # so we have never braked at all -- the car coasts at -2.4..-3.3 m/s2
        # against a plan that asks -4.0.
        #   'none'  the shipping law, untouched.
        #   'zero'  publish exactly 0.0 whenever the setpoint asks for more
        #           deceleration than `brake_a_min`, or sits `brake_dv` below
        #           the (predicted) measured speed.
        #   'pulse' duty-cycle 0.0 against the feedforward command, one tick
        #           per period, to approximate a requested deceleration
        #           anywhere between 0 and the full brake `brake_a_full`.
        # `brake_a_full` 4.6 + `drag` 0.273 re-measured on the finals track
        # (Sep 19 audit) from the samples where the published throttle is
        # exactly 0.0 and the car is above 1.5 m/s, IPS truth differentiated:
        # on the Xvfb log (`race_20260918_185202`, 1556 samples) the least
        # squares fit is **-a = 4.31 + 0.284 v**, against this model's
        # 4.60 + 0.273 v; the display log agrees on the median (5.39 m/s2
        # against the Xvfb 5.60) but its own fit is unusable, because
        # differentiating a 0.058 s IPS step puts outliers to 37 m/s in it.
        # Both constants stand as they are.
        # NOTE: any string that is not 'none' or 'pulse' falls through to
        # 'zero' below, so a typo brakes and is never reported. `mpc3.py`
        # rejects an unknown spelling at construction.
        self.brake_mode = str(brake_mode)
        self.brake_a_min = float(brake_a_min)
        self.brake_dv = float(brake_dv)
        self.brake_a_full = float(brake_a_full)
        self._duty = 0.0                 # pulse-width accumulator
        self.brakes = 0                  # ticks published as exactly 0.0
        self._hist = []                  # (t, u) as issued, for dead-time
        self.last_target = 0.0           # last setpoint, m/s (diagnostics)

    # -- dead time ---------------------------------------------------------
    def _predict(self, v_meas, now):
        """Where the car will be when this command lands, tau from now.

        The commands already in flight are the ones issued in the last `tau`
        seconds; roll the plant forward under them.  Without this the law
        reacts to a speed that is one dead time stale, which on corner entry
        is most of a metre of braking distance.
        """
        v = float(v_meas)
        if not self.predict or self.tau <= 0.0:
            return v
        t0 = now - self.tau
        pts = [(max(t, t0), u) for t, u in self._hist if t >= t0]
        if not pts:
            return v
        pts.append((now, pts[-1][1]))
        for (ta, ua), (tb, _) in zip(pts, pts[1:]):
            dt = tb - ta
            if dt <= 0.0:
                continue
            n = max(1, int(dt / 0.01))
            h = dt / n
            for _ in range(n):
                if self.brake_mode != 'none' and ua == 0.0:
                    # A command of exactly 0.0 is the four-wheel brake, not a
                    # zero setpoint fed to the servo.  Modelling it as the
                    # latter would under-predict the deceleration already in
                    # flight by ~2 m/s2 and re-brake on the next tick.
                    a = -self.brake_a_full - self.drag * v
                else:
                    a = self.K * (self.g * ua - v)
                    a = max(-self.a_auth, min(self.a_accel, a)) - self.drag * v
                v = max(0.0, v + a * h)
        return v

    # -- the law -----------------------------------------------------------
    def command(self, v_target, v_meas=None, a_target=0.0, now=None,
                kappa=None, head_scale=1.0):
        """Throttle in [0, u_max].  Never negative -- see the module docstring.

        Three terms, in the order they matter:

        * **the setpoint IS the target.**  Because the servo is fast (K 18-40
          1/s against a 0.09 s period), commanding the speed we want already
          buys the full +5.0 / -4.5 m/s2 -- the servo saturates on its own
          while the error is larger than ~0.25 m/s and eases off as it
          arrives.  There is nothing to gain from a bang-bang law here, and a
          setpoint BELOW the target only over-brakes.  This is also why the
          shipping feedforward is not as wrong as it looks: `0.040*v + 0.005`
          is `v/25 + 0.005`, within 1% of `v/24.6`.
        * **lead, not gain.**  What the feedforward law really loses is time:
          its setpoint is the plan speed AT THE CAR, but the command does not
          take effect for `tau` and then stays in force for one period, so it
          holds a speed the plan wanted ~0.2 s ago.  Where the plan's ax is
          below the servo's authority that lag is a pure deficit, and the fix
          is to command the speed the plan will want when the command is
          actually in force: `v_target + a_target*(tau + hold)`.
        * **a bounded over-command.**  `k_p * (lead - v_predicted)`, capped at
          `head_max`, covers the encoder's own lag (one tick, alpha 0.35) and
          the half-period quantisation of the dead time.  It is added ONLY
          when the car is below the target, and it vanishes as the error does,
          so it cannot leave a standing overspeed.

        `v_target` is what the caller wants held -- `mpc` passes the MINIMUM
        profile speed over its braking horizon, i.e. the anti-overshoot target
        is chosen by the caller.  `a_target` is the plan's ax at the car, and
        mpc zeroes it whenever that horizon minimum has already dropped below
        the local plan speed.  This class enforces the same rule again: when
        `a_target <= 0` no lead is taken, so a caller that forgets cannot make
        us accelerate into a braking zone.
        """
        now = time.monotonic() if now is None else float(now)
        v_t = max(0.0, float(v_target))
        v_pred = self._predict(v_meas, now) if v_meas is not None else v_t

        # The lead: the plan speed at the position the car will have reached
        # when this command is in force, which to first order is
        # `v + a_plan * (tau + hold)`.  Uncapped this is up to 1.5 m/s at
        # ax 6.5, and offline that shows up as the car sitting ABOVE the plan
        # at its own position -- because the servo reaches a setpoint in about
        # a third of a period, not a whole one, so a full-period lead
        # over-leads.  It is harmless on a straight and not harmless at a
        # hairpin EXIT, which is where every collision above ay 5 has been.
        # So the lead is capped twice: by `lead_max`, and -- when the caller
        # passes the local curvature -- by the speed the grip budget still
        # allows at that curvature.
        a_t = float(a_target)
        lead_m = self.lead_max
        if kappa is not None:
            k = abs(float(kappa))
            v_grip = ((self.ay_limit / k) ** 0.5 if k > 1e-4 else float('inf'))
            lead_m = max(0.0, min(lead_m, v_grip - v_t))
        lead = v_t + min(lead_m,
                         self.k_ff * max(a_t, 0.0)
                         * (max(self.tau, 0.0) + self.hold))
        err = lead - v_pred
        # Over-command only upwards.  Downwards the setpoint is the target
        # itself, which already commands the servo's full braking authority;
        # going below it would brake past the corner-entry speed and cost the
        # exit, and (with the `v_floor` below) is the one route to a reverse
        # setpoint, which is what `brake_gain` used to do by accident.
        head = self.head_max * max(0.0, min(1.0, float(head_scale)))
        # OPT-IN (`head_kappa` 0 = off, byte-identical).  The over-command
        # exists to close the straight-line exit deficit; applied while the car
        # is still TURNING it chases the profile's exit acceleration mid-corner
        # and runs the car wide (finals, Sep 19: h3c + servo, 6 collisions in
        # 100 Xvfb laps, all 0.3 m wide on corner exits at kappa 0.3-0.6).  Fade
        # the headroom linearly to zero as |kappa| under the car rises to
        # `head_kappa`.
        if self.head_kappa > 0.0 and kappa is not None:
            head *= max(0.0, 1.0 - abs(float(kappa)) / self.head_kappa)
        target = lead + max(0.0, min(head, self.k_p * err))

        # Never a reverse setpoint, and never a crawl setpoint while moving:
        # below ~1 m/s a low setpoint reads as "stop".  Do not raise the floor
        # above what the plan wants, or the car will not slow for a hairpin
        # that genuinely asks for 2 m/s.
        target = max(min(target, self.g * self.u_max), min(self.v_floor, v_t))
        target = max(target, 0.0)
        self.last_target = target

        u = max(0.0, min(self.u_max, target / self.g))

        # --- the brake (opt-in) ---------------------------------------------
        if self.brake_mode != 'none' and v_meas is not None:
            # How much deceleration the setpoint is asking for, over the time
            # this command is actually in force: the dead time plus one hold.
            # At the default 0.135 + 0.09 s that makes `brake_dv` 0.3 m/s and
            # `brake_a_min` 1.5 m/s2 the same trigger to within 10%, which is
            # what makes the OR of them a single sensible threshold.
            t_force = max(self.tau, 0.0) + self.hold
            dv = v_pred - v_t
            a_req = dv / t_force if t_force > 1e-6 else 0.0
            want = (a_req > self.brake_a_min or dv > self.brake_dv)
            # Never brake at a crawl: below `v_floor` the car is parking, and
            # a full four-wheel brake there just stops it on the line.
            if want and v_pred > self.v_floor:
                if self.brake_mode == 'pulse':
                    duty = max(0.0, min(1.0, a_req / max(self.brake_a_full,
                                                         1e-6)))
                    self._duty += duty
                    if self._duty >= 1.0:
                        self._duty -= 1.0
                        u = 0.0
                else:                                   # 'zero'
                    u = 0.0
            else:
                self._duty = 0.0
        if u == 0.0:
            self.brakes += 1
        self._hist.append((now, u))
        cutoff = now - max(self.tau, 0.0) - 0.5
        while len(self._hist) > 2 and self._hist[0][0] < cutoff:
            self._hist.pop(0)
        return float(u)


def brake_limited_target(v_plan_ahead, ds_ahead, a_brake=4.0, v_floor=0.5):
    """Fastest speed we may hold NOW and still make every planned speed ahead.

    `mpc` currently takes the minimum profile speed over `brake_horizon_s *
    v` of track -- a fixed *time* window, which at 7 m/s is 2.5 m.  That is
    both too early far from the corner (the target has already dropped 1.6 m/s
    below the local plan; measured on the Sept 11 ay-5.0 run) and, if the
    window were shortened, too late to stop in.  The honest question is a
    braking-distance one: with `a_brake` available, the speed we may hold now
    is `sqrt(v_plan(s')^2 + 2*a_brake*(s'-s))` minimised over the track ahead.

    `v_plan_ahead` and `ds_ahead` are same-length sequences of planned speeds
    and their distances ahead of the car.  Nothing here needs the car's own
    speed, so it is a pure function of the plan -- safe to precompute.
    """
    best = float('inf')
    for v_p, ds in zip(v_plan_ahead, ds_ahead):
        best = min(best, (max(v_p, 0.0) ** 2 + 2.0 * a_brake * max(ds, 0.0)) ** 0.5)
    return max(best, v_floor)
