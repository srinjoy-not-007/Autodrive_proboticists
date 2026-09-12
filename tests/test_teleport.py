"""The opt-in teleport handler in `localization.py`.

Background, all measured (see docs/STATUS.md and the report in the session):
a collision returns the car to its last NUMBERED checkpoint, at rest. Over 329
real collisions in `data/logs/race_2026091*.csv` the reset moves the car
0.65 m (p50, p90 1.06 m) while one scan period of normal driving already moves
it 0.28 m, and the wheel encoders do NOT notice -- filtered encoder speed goes
3.15 -> 3.02 m/s across the reset while the body's true speed goes 2.85 ->
0.85. So neither `SpeedEstimator.jumped` nor the particle filter's lost-score
branch ever fires, and the cloud is dead-reckoned forward through a standstill.

Everything here is opt-in: with the shipped defaults the detector is never
consulted and the motion model is untouched.
"""

import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, 'tests')
SRC = os.environ.get('PIT_CREW_SRC') or os.path.join(ROOT, 'src')
DATA = os.environ.get('PIT_CREW_DATA') or os.path.join(ROOT, 'data')

sys.path.insert(0, TESTS)
import rclpy_stub                                              # noqa: E402

rclpy_stub.install()
sys.path.insert(0, SRC)

from pit_crew import vehicle                                   # noqa: E402

MAP_PREFIX = os.path.join(DATA, 'maps', 'porto_mesh')
CENTERLINE = os.path.join(DATA, 'racelines', 'porto_mesh_centerline.csv')
CHECKPOINTS = os.path.join(DATA, 'maps', 'porto_mesh_checkpoints.csv')


def scan_msg(t, ranges):
    from sensor_msgs.msg import LaserScan
    m = LaserScan()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    m.angle_min = vehicle.LIDAR_ANGLE_MIN
    m.angle_increment = vehicle.LIDAR_ANGLE_INC
    m.angle_max = vehicle.LIDAR_ANGLE_MIN + \
        vehicle.LIDAR_ANGLE_INC * (vehicle.LIDAR_N - 1)
    m.range_min = 0.05
    m.range_max = 10.0
    m.ranges = [float(v) for v in ranges]
    return m


def imu_msg(t, yaw, accel=0.0):
    from sensor_msgs.msg import Imu
    m = Imu()
    m.header.stamp.sec = int(t)
    m.orientation.z = math.sin(yaw / 2.0)
    m.orientation.w = math.cos(yaw / 2.0)
    m.linear_acceleration.x = float(accel)
    m.linear_acceleration.y = 0.0
    return m


def node(**over):
    from pit_crew.localization import ParticleFilter
    over.setdefault('map_prefix', MAP_PREFIX)
    over.setdefault('centerline_csv', CENTERLINE)
    over.setdefault('n_particles', 200)
    rclpy_stub.reset_clock()
    rclpy_stub.set_overrides(over)
    n = ParticleFilter()
    rclpy_stub.set_overrides({})
    return n


class TestDefaultsAreInert:

    def test_no_checkpoints_and_no_detector_by_default(self):
        n = node()
        assert n.checkpoints is None
        assert n.tp_scan_jump == 0.0 and n.tp_imu_g == 0.0
        assert n._prev_ranges is None and n._tp_hold == 0

    def test_the_detector_is_not_consulted_with_the_default_thresholds(self):
        n = node()
        n.imu_accel = 1e9                  # would trip any enabled branch
        n._prev_ranges = np.zeros(vehicle.LIDAR_N)
        assert n.teleport_detected(np.full(vehicle.LIDAR_N, 9.0),
                                   scan_msg(1.0, [9.0] * vehicle.LIDAR_N)) is None

    def test_predict_is_untouched_when_no_hold_is_armed(self):
        n = node(motion_noise_xy=0.0, motion_noise_theta=0.0)
        n.init_particles_near(0.0, 0.0, 0.0)
        n.pth[:] = 0.0
        n.imu_yaw = 0.0
        n.speed_est.speed = 3.0
        n.predict(0.1)
        assert float(np.mean(n.px)) == pytest.approx(0.3, abs=1e-9)


class TestCheckpoints:

    def test_only_numbered_checkpoints_are_loaded(self):
        n = node(checkpoints_csv=CHECKPOINTS)
        assert n.checkpoints is not None
        # Porto's scene has 21 numbered checkpoints plus two named finish
        # lines and two spawns, which are not reset targets.
        assert n.checkpoints.shape == (21, 3)
        assert np.isfinite(n.checkpoints).all()

    def test_a_missing_file_disables_the_reseed_without_raising(self):
        n = node(checkpoints_csv=os.path.join(DATA, 'maps', 'nope.csv'))
        assert n.checkpoints is None
        n.init_particles_near(0.0, 0.0, 0.1)
        n.imu_yaw = 0.0
        assert n.seed_at_checkpoints() is False

    def test_the_reseed_stays_near_the_current_estimate(self):
        n = node(checkpoints_csv=CHECKPOINTS, teleport_cp_radius=2.5)
        # (0.60, -2.04) is checkpoint 4; 5 and 3 are its neighbours.
        n.init_particles_near(0.60, -2.04, 0.05)
        n.imu_yaw = -1.5708
        assert n.seed_at_checkpoints() is True
        d = np.hypot(n.px - 0.60, n.py + 2.04)
        # every particle sits on a checkpoint inside the radius (+ its spread)
        assert d.max() < 2.5 + 1.0
        # and carries that checkpoint's heading, not a widened prior
        heads = n.imu_yaw + n.pth
        assert np.abs(np.cos(heads)).max() < 0.8

    def test_radius_zero_means_every_checkpoint(self):
        n = node(checkpoints_csv=CHECKPOINTS, teleport_cp_radius=0.0)
        n.init_particles_near(0.60, -2.04, 0.05)
        n.imu_yaw = 0.0
        assert n.seed_at_checkpoints() is True
        assert np.hypot(n.px - 0.60, n.py + 2.04).max() > 5.0


class TestDetection:

    def test_scan_branch_ignores_a_scan_that_barely_moved(self):
        n = node(teleport_scan_jump=0.25)
        r0 = np.full(vehicle.LIDAR_N, 3.0)
        n._prev_ranges = r0
        r1 = r0 + 0.05
        assert n.teleport_detected(r1, scan_msg(1.0, r1)) is None

    def test_scan_branch_fires_on_a_wholesale_change(self):
        n = node(teleport_scan_jump=0.25)
        n._prev_ranges = np.full(vehicle.LIDAR_N, 3.0)
        r1 = np.full(vehicle.LIDAR_N, 5.0)
        assert n.teleport_detected(r1, scan_msg(1.0, r1)) == 'scan'

    def test_imu_branch_fires_only_above_the_threshold(self):
        n = node(teleport_imu_g=15.0)
        r = np.full(vehicle.LIDAR_N, 3.0)
        n.imu_accel = 8.76            # the clean-bag maximum over 2980 scans
        assert n.teleport_detected(r, scan_msg(1.0, r)) is None
        n.imu_accel = 40.0
        assert n.teleport_detected(r, scan_msg(1.0, r)) == 'imu'

    def test_the_imu_accel_is_only_read_when_that_branch_is_on(self):
        off, on = node(), node(teleport_imu_g=15.0)
        for n in (off, on):
            n.on_imu(imu_msg(1.0, 0.0, accel=30.0))
        assert off.imu_accel == 0.0
        assert on.imu_accel == pytest.approx(30.0)


class TestResponse:

    def test_a_hold_freezes_dead_reckoning_and_spreads_instead(self):
        n = node(motion_noise_xy=0.0, motion_noise_theta=0.0)
        n.init_particles_near(0.0, 0.0, 0.0)
        n.pth[:] = 0.0
        n.imu_yaw = 0.0
        n.speed_est.speed = 3.0
        n._tp_hold = 2
        for _ in range(2):
            n.predict(0.1)
        assert float(np.mean(np.abs(n.px))) < 1e-9     # no phantom travel
        assert n._tp_hold == 0
        n.predict(0.1)                                  # hold expired
        assert float(np.mean(n.px)) == pytest.approx(0.3, abs=1e-9)

    def test_the_hold_length_is_what_was_asked_for(self):
        n = node(teleport_scan_jump=0.25, teleport_hold_n=6,
                 checkpoints_csv=CHECKPOINTS)
        n.init_particles_near(0.60, -2.04, 0.1)
        n.imu_yaw = -1.5708
        n.speed_est.update('L', 1.0, 0.0)
        n.speed_est.update('L', 1.1, 5.0)
        n._prev_ranges = np.full(vehicle.LIDAR_N, 3.0)
        r1 = np.full(vehicle.LIDAR_N, 6.0)
        assert n.teleport_detected(r1, scan_msg(1.0, r1)) == 'scan'
        n.speed_est.reset()
        assert n.speed_est.speed == 0.0 and not n.speed_est._last


class TestSpeedEstimatorReset:

    def test_reset_drops_the_speed_and_the_stored_wheel_angles(self):
        se = vehicle.SpeedEstimator()
        se.update('L', 0.0, 0.0)
        se.update('L', 0.1, 5.0)
        assert se.speed > 0.0
        se.reset()
        assert se.speed == 0.0
        assert se._last == {} and se._per == {} and se._rej == {}
        # and the next pair differentiates cleanly across the gap
        se.update('L', 5.0, 1000.0)
        assert se.speed == 0.0
        se.update('L', 5.1, 1000.0 + 5.0 / vehicle.WHEEL_RADIUS * 0.1)
        # the per-wheel filter is seeded at its first accepted sample, so the
        # estimate is the measurement itself rather than a lagged fraction of
        # a pre-teleport value it no longer holds
        assert se.speed == pytest.approx(5.0, rel=1e-6)
