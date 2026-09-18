"""Shared vehicle constants and helpers for the RoboRacer digital twin."""

import math

# From the official technical guide (verify against calibration data, task #3)
WHEELBASE = 0.324           # m
TRACK_WIDTH = 0.236         # m
CAR_WIDTH = 0.270           # m (overall)
CAR_LENGTH = 0.500          # m
WHEEL_RADIUS = 0.059        # m
MAX_STEER = 0.5236          # rad (±30°); steering_command is normalized [-1, 1]
STEER_RATE = 3.2            # rad/s actuator slew
TOP_SPEED = 22.88           # m/s (claimed; calibrate)
# Calibrated plausibility bounds for the wheel-encoder derivative. The fastest
# the car has ever been planned to go is 7.29 m/s and the fastest measured is
# 6.58, so 9.0 leaves headroom without admitting nonsense; it reverses only
# under the escape manoeuvre, at around 1 m/s.
SPEED_MAX = 9.0             # m/s, largest believable forward wheel speed
# The bound has to stay tight: the glitches run from -2 to -10.8 m/s with a
# MEDIAN of -5.4, so loosening it to 9.0 lets most of them straight through
# (measured: RMS back to 1.42 m/s from 0.46). Tightness alone is a trap in the
# other direction -- the estimator then cannot SEE a real fast reverse, keeps
# reporting a stale forward speed, and the brake goes on firing, which is what
# drives the reverse. So the discriminator is not the magnitude but whether it
# PERSISTS: reject an isolated excursion, believe a sustained one (clamped).
REVERSE_MAX = 3.0           # m/s, largest believable reverse wheel speed
MAX_ENC_GAP = 0.5           # s, older than this and we re-seed instead
MAX_CONSEC_REJECT = 6       # rejections before we re-seed rather than hold
# ...and that count is in SAMPLES, which is a different amount of TIME on every
# machine: 6 samples is 0.624 s at the 0.104 s period it was tuned at and only
# 0.528 s at the 0.088 s the submission image measures. The window that decides
# "isolated glitch" from "sustained motion" -- and therefore how long the
# controller may steer and brake by a stale speed -- must not shrink because the
# host got faster. `SpeedEstimator.reject_max` is re-pointed from this, and is
# the identity at RATE_NOMINAL by construction.
REJECT_HOLD_S = 0.624       # s, == MAX_CONSEC_REJECT samples at RATE_NOMINAL
BRAKE_MIN_SPEED = 1.0       # m/s below which a negative command is just reverse
BRAKE_MAX = 0.45            # largest braking command, magnitude
# Total vehicle mass. The technical guide gives 3.906 kg = 3.470 sprung +
# 0.436 unsprung, and compare_sim_builds.py reads 3.47 off the Unity rigidbody
# because a Unity rigidbody carries only the SPRUNG mass -- the wheels are
# WheelColliders with their own mass. A review flagged the planner's 3.906 as
# "unsourced" and it was briefly changed to 3.47; that was wrong, 3.906 is the
# figure a velocity profile wants. With drag_coeff=0 mass cancels out of a
# friction-limited profile either way, so no line ever moved -- but the constant
# should say what it means.
MASS = 3.906                # kg, total (sprung + unsprung)
SPRUNG_MASS = 3.470         # kg, what the Unity rigidbody reports

# LiDAR geometry
# Sensor mounting, measured directly out of the simulator's own scene with
# tools/compare_sim_builds.py machinery (2026-08-23), not taken from a datasheet:
#   rear axle centre  z = 0.640
#   IPS / IMU         z = 0.720   -> +0.0800 m ahead of the rear axle
#   LiDAR             z = 0.9133  -> +0.2733 m ahead of the rear axle
#
# The pose we localize and race against is the IPS point, so what matters to ray
# casting is the LiDAR's offset from *that*: 0.2733 - 0.0800.
#
# Neither the mapper nor the particle filter applied this. Rays were cast 0.19 m
# behind where they physically originate — larger than the entire 0.171 m
# lateral localization error we are trying to reduce. It partly self-cancels
# because both stages made the same mistake, but only while heading the way the
# map was recorded.
IPS_FROM_REAR_AXLE = 0.0800
LIDAR_FROM_REAR_AXLE = 0.2733
LIDAR_FROM_POSE = LIDAR_FROM_REAR_AXLE - IPS_FROM_REAR_AXLE   # 0.1933 m

# Measured front-to-rear axle distance in the same scene is 0.3312 m, against
# the 0.324 the datasheet gives and this file uses. Left alone for now: the
# 2.2% difference feeds the steering model and the whole shipping config is
# validated against 0.324. Worth revisiting deliberately, not incidentally.

LIDAR_N = 1080
LIDAR_FOV = math.radians(270.0)
LIDAR_ANGLE_MIN = -math.radians(135.0)
LIDAR_ANGLE_INC = LIDAR_FOV / (LIDAR_N - 1)
LIDAR_RANGE_MIN = 0.06
LIDAR_RANGE_MAX = 10.0


def steer_to_cmd(angle_rad: float) -> float:
    """Convert a steering angle in radians to a normalized [-1, 1] command.

    A non-finite angle publishes STRAIGHT, not full lock.  Python's `min`/`max`
    return the FIRST argument when the comparison with a NaN is False, so the
    old `max(-1.0, min(1.0, nan))` evaluated `min(1.0, nan)` -> 1.0 and then
    `max(-1.0, 1.0)` -> 1.0, i.e. a NaN steering angle was published as **full
    LEFT lock** and latched there (this simulator holds the last command). The
    same trap is documented in `mpc3_core.MPC3._solve`, which raises on a
    non-finite state for exactly this reason; this is the backstop for every
    other route to the actuator. Finite angles take the identical path.
    """
    if not math.isfinite(angle_rad):
        return 0.0
    return max(-1.0, min(1.0, angle_rad / MAX_STEER))


# Measured understeer, from tools/grip_curve.py on the ay-4.0 race trace: the
# fraction of the curvature a steering angle asks for that the front tyres
# actually deliver. They track the kinematic bicycle model below ~0.10 rad, are
# visibly saturating by 0.15, and return 42% at full lock.
#
# This matters to any model that predicts where the car will be. Rolling forward
# with tan(delta)/L past ~0.15 rad expects curvature the car will not produce, so
# the car runs wide, and a controller that believes its own model answers by
# winding on more lock — which lands further down this curve and makes it worse.
# That understeer-then-recover cycle is what put the ay-4.0 run into the wall.
_UNDERSTEER = ((0.000, 1.00), (0.075, 1.00), (0.125, 0.88), (0.175, 0.80),
               (0.225, 0.79), (0.275, 0.68), (0.350, 0.52), (0.460, 0.42),
               (MAX_STEER, 0.40))


def understeer_ratio(delta: float) -> float:
    """Fraction of the asked-for curvature the front tyres really deliver."""
    a = abs(delta)
    if a <= _UNDERSTEER[0][0]:
        return _UNDERSTEER[0][1]
    for (x0, y0), (x1, y1) in zip(_UNDERSTEER, _UNDERSTEER[1:]):
        if a <= x1:
            return y0 + (y1 - y0) * (a - x0) / (x1 - x0)
    return _UNDERSTEER[-1][1]


def achieved_curvature(delta: float) -> float:
    """Curvature this steering angle actually produces, understeer included."""
    return understeer_ratio(delta) * math.tan(delta) / WHEELBASE


# The sensor period is a property of the MACHINE, not of this stack: measured
# 0.104 s on the dev host, 0.088 s inside the submission image, 0.097 s by the
# organisers. Anything tuned PER SAMPLE therefore means something different on
# each of them, and `SpeedEstimator.alpha` is exactly that -- 0.35 per sample is
# a 0.193 s lag at 0.104 s and a 0.163 s lag at 0.088 s. The speed that lag
# distorts is what `localization.predict` dead-reckons with, so the filter's
# steady-state along-track error becomes rate dependent for no physical reason.
# Fix the lag in SECONDS and derive alpha from the period instead.
RATE_NOMINAL = 0.104        # s, the sensor period every default was tuned at
ENC_TAU = 0.193             # s, == alpha 0.35 at RATE_NOMINAL


def alpha_for_lag(dt: float, tau: float = ENC_TAU) -> float:
    """Per-sample low-pass factor giving a fixed first-order lag `tau`.

    Identity by construction at the period the constant came from:
    `alpha_for_lag(0.104, 0.193) == 0.35` to three decimals.
    """
    if not (dt > 0.0 and tau > 0.0):
        return 1.0
    return float(dt / (tau + dt))


def noise_scale_for_dt(dt: float, nominal: float = RATE_NOMINAL) -> float:
    """Scale a PER-UPDATE random walk so its diffusion per unit TIME is fixed.

    0.18 m injected every update is 0.558 m/sqrt(s) at 0.104 s and 0.607 at
    0.088 s: the same filter is 9% more diffusive on a faster host, which moves
    the posterior's weight between prior and scan. Identity at `nominal`.
    """
    if not (dt > 0.0 and nominal > 0.0):
        return 1.0
    return float(math.sqrt(dt / nominal))


def reject_max_for_dt(dt: float, hold_s: float = REJECT_HOLD_S) -> int:
    """Consecutive-rejection count giving a fixed hold TIME `hold_s`.

    `reject_max_for_dt(0.104) == 6 == MAX_CONSEC_REJECT`, i.e. the identity at
    the period the constant was tuned at. Floored at 2 so an isolated glitch is
    still rejected however slow the host is.
    """
    if not (dt > 0.0 and hold_s > 0.0):
        return MAX_CONSEC_REJECT
    return max(2, int(round(hold_s / dt)))


class SpeedEstimator:
    """Vehicle speed from wheel encoder angles.

    `position` is the cumulative wheel angle in radians, so speed is simply
    wheel_radius * d(position)/dt. Verified against IPS ground truth on two
    different tracks: the signal is monotonic while driving forward and the
    scale factor comes out at 0.993 and 0.999, i.e. exactly right.

    An earlier revision declared this signal unusable. That was measured on a
    stretch where the car was repeatedly stuck, reversing and being teleported
    to checkpoints, which makes the derivative look like noise. Judge a sensor
    on clean data.
    """

    def __init__(self, alpha: float = 0.35):
        self.alpha = alpha          # low-pass factor for the derivative
        # Consecutive rejections tolerated before a sample is believed. In
        # SAMPLES, because that is what `update` counts -- but it means a fixed
        # TIME, so whoever knows the sensor period re-points it with
        # `reject_max_for_dt`. Identity at RATE_NOMINAL.
        self.reject_max = MAX_CONSEC_REJECT
        # Largest sample interval still differentiated. Default MAX_ENC_GAP.
        # It matters on a TELEMETRY STALL: above this the encoder contributes
        # nothing across the gap, so the filtered speed stays at 100% of its
        # pre-gap value and whoever dead-reckons with it marches the car
        # forward through a stall it may have spent standing still. Raising it
        # makes the sample measure the MEAN speed over the blind window, which
        # is the correct thing for both stall shapes. Opt-in: localization
        # leaves it at the constant unless `enc_gap_max` is set.
        self.max_gap = MAX_ENC_GAP
        self.speed = 0.0            # m/s, filtered
        self._last = {}             # wheel -> (stamp_sec, position_rad)
        self._per = {}              # wheel -> filtered speed for that wheel
        self._rej = {}              # wheel -> consecutive rejected samples
        self.jumped = False         # set when a sample implies a teleport

    def update(self, wheel: str, stamp_sec: float, position_rad: float) -> float:
        if not (math.isfinite(stamp_sec) and math.isfinite(position_rad)):
            return self.speed
        last = self._last.get(wheel)
        if last is None:
            self._last[wheel] = (stamp_sec, position_rad)
            return self.speed
        dt = stamp_sec - last[0]
        if dt <= 1e-4:
            return self.speed                    # duplicate / out-of-order
        if dt >= self.max_gap:
            self._last[wheel] = (stamp_sec, position_rad)
            return self.speed                    # too stale to differentiate
        v = WHEEL_RADIUS * (position_rad - last[1]) / dt
        # Plausibility. `TOP_SPEED` is a spec CLAIM (22.88 m/s, and its own
        # comment says "calibrate"), so the old bound of 1.1x it -- 25 m/s --
        # rejected essentially nothing. Measured on a clean 120 s Porto run:
        # the cumulative wheel angle steps BACKWARDS on 6.7% of samples, in
        # pairs one sample apart, by up to 18.5 rad. Every one of those passed
        # the old guard and was fed in as a speed down to -10.8 m/s.
        if not -REVERSE_MAX <= v <= SPEED_MAX:
            # Holding the reference is right for an isolated glitch and wrong
            # for sustained motion outside the bounds: the estimate then
            # freezes at its last good value and the controller steers by a
            # stale number. Re-seed after a few consecutive rejections so the
            # worst case is a short gap, not an indefinite lie.
            self._rej[wheel] = self._rej.get(wheel, 0) + 1
            if self._rej[wheel] >= self.reject_max:
                # Persisted: this is motion, not a glitch. Believe it, clamped,
                # so a car that really is reversing hard is reported as
                # reversing instead of holding a stale forward speed forever.
                self._last[wheel] = (stamp_sec, position_rad)
                self._rej[wheel] = 0
                cl = max(-REVERSE_MAX, min(SPEED_MAX, v))
                self._per[wheel] = cl
                self.speed = float(sum(self._per.values()) / len(self._per))
                return self.speed
            # Do NOT advance `_last` here. It used to be advanced before the
            # check, so a rejected sample poisoned the NEXT one too: the
            # recovery step then measured the whole glitch as real motion in
            # the opposite direction. Holding the last good reference makes the
            # next sample differentiate cleanly across the gap instead.
            if abs(v) > SPEED_MAX * 2.0:
                self.jumped = True               # a real teleport / reset
            return self.speed
        self._rej[wheel] = 0
        self._last[wheel] = (stamp_sec, position_rad)
        # One filter per wheel, then average. Both wheels used to feed a single
        # shared filter alternately, so in a corner -- where the inner and outer
        # wheels genuinely differ -- the estimate chattered between them.
        self._per[wheel] = self._per.get(wheel, v) + self.alpha * (
            v - self._per.get(wheel, v))
        self.speed = float(sum(self._per.values()) / len(self._per))
        return self.speed

    def reset(self) -> None:
        """Drop everything this estimator holds about where the car *was*.

        Same effect as a consumed teleport, but callable by a detector that
        saw the teleport some other way (LiDAR / IMU). Opt-in: nothing in the
        default path calls it, so behaviour is unchanged unless a caller asks.
        The wheels keep spinning through a checkpoint reset -- measured on 329
        real collisions, the filtered encoder speed goes 3.15 -> 3.02 m/s while
        the body's true speed goes 2.85 -> 0.85 -- so the stored positions are
        about a body that has stopped and must not be differentiated across.
        """
        self.speed = 0.0
        self._last.clear()
        self._per.clear()
        self._rej.clear()
        self.jumped = False

    def consume_jump(self) -> bool:
        """True once per detected teleport, clearing the state it invalidated.

        The simulator returns the car to its last checkpoint after a collision,
        which both moves it and zeroes the encoders. Everything this estimator
        holds is about where the car *was*: the filtered speed describes motion
        that has stopped, and the stored positions would make the next
        derivative garbage too. Drop both and re-seed from the next sample.
        """
        if not self.jumped:
            return False
        self.jumped = False
        self.speed = 0.0
        self._last.clear()
        self._per.clear()
        self._rej.clear()
        return True


class ThrottleController:
    """Open-loop throttle from a target speed.

    Open loop because of the *loop rate*, not the sensor. SpeedEstimator above
    is verified good (scale 0.993/0.999 against ground truth on two tracks); an
    earlier revision of this docstring said the platform had no trustworthy
    speed feedback, which was retracted and is kept out of here deliberately --
    that belief cost real time once already.

    What actually stops the loop closing is that every sensor shares one
    WebSocket at ~10 Hz. Feedback at 10 Hz on an actuator with no torque model
    made the car surge back and forth, so speed is commanded feedforward from
    the measured throttle->steady-state-speed curve instead: `ff_gain` defaults
    to the **measured** 0.040 throttle per m/s, not a placeholder. `kp`
    defaults to 0, i.e. the loop is open unless someone deliberately closes it.
    """

    def __init__(self, kp: float = 0.0, ff_gain: float = 0.040,
                 brake_gain: float = 0.0, idle: float = 0.005,
                 brake_deadband: float = 0.5, a_gain: float = 0.0):
        self.kp = kp
        # Throttle per m/s2 of PLANNED acceleration -- open loop, so unlike kp
        # it cannot oscillate against the 10 Hz feedback. Measured by least
        # squares on the Aug 29 brake-7.0 race trace (7492 samples):
        #   throttle = 0.0144*v + 0.0414*a + 0.055
        # The binned form is blunter still: at 3-6 m/s a throttle of 0.18-0.30
        # produced only +1.44 m/s2 where the profile was asking for 6.5, and
        # the car sat 0.96 m/s under its own plan wherever ax > 3. Default 0.0
        # so no caller changes behaviour without asking.
        self.a_gain = a_gain
        self.ff = ff_gain
        # Default 0.0: braking stays OFF unless a caller asks for it, so this
        # change cannot alter any node that has not opted in.
        self.brake_gain = brake_gain
        self.idle = idle
        self.brake_deadband = brake_deadband

    def command(self, v_target: float, v_meas: float = None,
                a_target: float = 0.0) -> float:
        """Throttle in [-1, 1]. Negative is braking.

        The braking branch used to be gated behind `kp > 0`, and nothing sets
        `kp`, so it was unreachable: every command was `ff*v_target + idle`,
        which is strictly positive. The car had **no braking authority at all**
        and shed speed only by coasting -- while the offline velocity profile
        was built assuming 4.0 m/s2 of deceleration it could not produce.
        Braking now depends on `brake_gain` alone, so it can be enabled without
        also closing the proportional loop (which surges at 10 Hz).
        """
        u = self.ff * v_target + self.idle + self.a_gain * max(a_target, 0.0)
        if v_meas is not None:
            err = v_target - v_meas
            if (err < -self.brake_deadband and self.brake_gain > 0.0
                    and v_meas > BRAKE_MIN_SPEED):
                # Negative throttle in this simulator is REVERSE DRIVE, not a
                # brake pedal, so an unclamped `brake_gain * err` will happily
                # drive the car backwards into a wall. That stayed hidden while
                # the encoder speed read ~20% low and was mostly noise: braking
                # barely fired. With the corrected estimator it fires properly,
                # and unclamped it put the car in reverse for 52% of a run --
                # 176 collisions. Two guards: never brake below a walking pace,
                # where any negative command is just reverse; and cap the
                # magnitude, because at 10 Hz the car can cross zero between
                # samples however good the deadband is.
                u = max(-BRAKE_MAX, self.brake_gain * err)
            elif self.kp > 0.0:
                u = u + self.kp * err
        return max(-1.0, min(1.0, u))
