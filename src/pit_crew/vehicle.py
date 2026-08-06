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
    """Vehicle speed from cumulative wheel encoder angles (rad)."""

    def __init__(self, alpha: float = 0.35):
        self.alpha = alpha          # low-pass factor for the derivative
        self.speed = 0.0            # m/s, filtered
        self._last = {}             # wheel -> (stamp_sec, position_rad)

    def update(self, wheel: str, stamp_sec: float, position_rad: float) -> float:
        last = self._last.get(wheel)
        self._last[wheel] = (stamp_sec, position_rad)
        if last is not None:
            dt = stamp_sec - last[0]
            if 1e-4 < dt < 0.5:
                v = WHEEL_RADIUS * (position_rad - last[1]) / dt
                self.speed += self.alpha * (v - self.speed)
        return self.speed


class ThrottleController:
    """Feedforward + proportional throttle controller tracking a target speed.

    The feedforward gain assumes roughly linear throttle->steady-state speed;
    replace with the measured curve once calibration (task #3) is done.
    """

    def __init__(self, kp: float = 0.25, ff_gain: float = 1.0 / TOP_SPEED,
                 brake_gain: float = 0.6):
        self.kp = kp
        self.ff = ff_gain
        self.brake_gain = brake_gain

    def command(self, v_target: float, v_meas: float) -> float:
        err = v_target - v_meas
        if err < -0.5:  # need to shed speed: actively brake, scaled by overshoot
            u = self.brake_gain * err  # negative
        else:
            u = self.ff * v_target + self.kp * err
        return max(-1.0, min(1.0, u))
