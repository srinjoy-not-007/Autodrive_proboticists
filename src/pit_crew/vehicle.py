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
    """Convert a steering angle in radians to a normalized [-1, 1] command."""
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
        self.speed = 0.0            # m/s, filtered
        self._last = {}             # wheel -> (stamp_sec, position_rad)
        self.jumped = False         # set when a sample implies a teleport

    def update(self, wheel: str, stamp_sec: float, position_rad: float) -> float:
        last = self._last.get(wheel)
        self._last[wheel] = (stamp_sec, position_rad)
        if last is not None:
            dt = stamp_sec - last[0]
            if 1e-4 < dt < 0.5:
                v = WHEEL_RADIUS * (position_rad - last[1]) / dt
                # Reject teleports/encoder resets (checkpoint resets zero the
                # encoders → derivative explodes); track can't exceed top speed
                if abs(v) <= TOP_SPEED * 1.1:
                    self.speed += self.alpha * (v - self.speed)
                else:
                    # Physically impossible derivative: a dropped or duplicated
                    # sample rather than real motion.
                    self.jumped = True
        return self.speed

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
            if err < -self.brake_deadband and self.brake_gain > 0.0:
                u = self.brake_gain * err          # negative -> braking
            elif self.kp > 0.0:
                u = u + self.kp * err
        return max(-1.0, min(1.0, u))
