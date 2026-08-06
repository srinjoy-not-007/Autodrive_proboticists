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

    DO NOT USE on this platform. Measured against IPS ground truth over a
    200 s bag, this signal correlates with true speed at |r| < 0.2 for every
    interpretation tried (raw, unwrapped at 2*pi, unwrapped at 360): the
    simulator's encoder angle aliases at the ~10 Hz publish rate, so the
    derivative is noise. Kept only so the finding stays documented.

    Use ThrottleController's open-loop map, or differentiate the localization
    pose, instead.
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
                    inn = max(-2.0, min(2.0, v - self.speed))
                    self.speed += self.alpha * inn
                else:
                    # Checkpoint resets zero the encoders, so an impossible
                    # derivative means the car was teleported, not accelerated.
                    self.jumped = True
        return self.speed


class ThrottleController:
    """Open-loop throttle from a target speed.

    Open loop on purpose: there is no trustworthy speed feedback on this
    platform (see SpeedEstimator), and closing the loop on the encoder signal
    made the car surge back and forth. `ff_gain` is a placeholder until the
    throttle->steady-state-speed curve is measured; pass `v_meas` only once a
    real speed estimate (localization-derived) exists.
    """

    def __init__(self, kp: float = 0.0, ff_gain: float = 0.075,
                 brake_gain: float = 0.6, idle: float = 0.02):
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
