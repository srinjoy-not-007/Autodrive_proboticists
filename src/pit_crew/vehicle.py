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

# LiDAR geometry
LIDAR_N = 1080
LIDAR_FOV = math.radians(270.0)
LIDAR_ANGLE_MIN = -math.radians(135.0)
LIDAR_ANGLE_INC = LIDAR_FOV / (LIDAR_N - 1)
LIDAR_RANGE_MIN = 0.06
LIDAR_RANGE_MAX = 10.0


def steer_to_cmd(angle_rad: float) -> float:
    """Convert a steering angle in radians to a normalized [-1, 1] command."""
    return max(-1.0, min(1.0, angle_rad / MAX_STEER))


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

    Open loop on purpose: there is no trustworthy speed feedback on this
    platform (see SpeedEstimator), and closing the loop on the encoder signal
    made the car surge back and forth. `ff_gain` is a placeholder until the
    throttle->steady-state-speed curve is measured; pass `v_meas` only once a
    real speed estimate (localization-derived) exists.
    """

    def __init__(self, kp: float = 0.0, ff_gain: float = 0.040,
                 brake_gain: float = 0.6, idle: float = 0.005):
        self.kp = kp
        self.ff = ff_gain
        self.brake_gain = brake_gain
        self.idle = idle

    def command(self, v_target: float, v_meas: float = None) -> float:
        u = self.ff * v_target + self.idle
        if v_meas is not None and self.kp > 0.0:
            err = v_target - v_meas
            u = self.brake_gain * err if err < -0.5 else u + self.kp * err
        return max(-1.0, min(1.0, u))
