"""Localization's behaviour across a TELEMETRY GAP (Sept 12), all opt-in.

Measured on the real collision bag `data/bags/exp_display_watch_120426`
(2344 scans at 0.0585 s), replayed through this filter with gaps injected
every 5 s; see docs/STATUS.md for the table. Two mechanisms decide what the
filter does with a gap and both are invisible until one appears:

  * `on_scan` clamps the scan interval at `scan_dt_max` 0.5 s and `predict`
    dead-reckons the whole of it at `max_step_mps` 9.0 -- one 4.5 m step.
  * `SpeedEstimator` refuses to differentiate across an interval of
    `MAX_ENC_GAP` 0.5 s or more, so for exactly the gaps that matter the
    speed it hands `predict` is 100% of the STALE pre-gap value.

The defaults are unchanged; every test here that asserts an improvement
turns a parameter on to get it.
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


def straight(n, v=5.0):
    n.init_particles_near(0.0, 0.0, 0.0)
    n.px[:] = 0.0
    n.py[:] = 0.0
    n.pth[:] = 0.0
    n.imu_yaw = 0.0
    n.speed_est.speed = v
    return n


class TestDefaultsAreInert:

    def test_the_gap_parameters_are_all_off(self):
        n = node()
        assert n.gap_dt == 0.0
        assert n.gap_step_cap_m == 0.0
        assert n.gap_noise_mult == 1.0
        assert n.lost_patience_min == 1
        assert n.scan_dt_max == 0.5
        assert n.enc_gap_max == 0.0
        assert n.speed_est.max_gap == vehicle.MAX_ENC_GAP

    def test_a_long_step_is_dead_reckoned_in_full_by_default(self):
        n = straight(node(motion_noise_xy=0.0, motion_noise_theta=0.0))
        n.predict(0.5)
        assert float(np.mean(n.px)) == pytest.approx(2.5, abs=1e-9)

    def test_the_default_step_is_capped_only_by_the_speed_cap(self):
        # 0.5 s x max_step_mps 9.0 is the 4.5 m worst case quoted above.
        n = straight(node(motion_noise_xy=0.0, motion_noise_theta=0.0,
                          max_step_mps=9.0), v=20.0)
        n.predict(0.5)
        assert float(np.mean(n.px)) == pytest.approx(4.5, abs=1e-9)


class TestGapStepCap:

    def test_the_cap_applies_only_above_the_gap_threshold(self):
        n = straight(node(motion_noise_xy=0.0, motion_noise_theta=0.0,
                          gap_dt=0.15, gap_step_cap_m=0.25))
        n.predict(0.104)                      # a normal period: untouched
        assert float(np.mean(n.px)) == pytest.approx(0.52, abs=1e-9)
        n.px[:] = 0.0
        n.predict(0.5)                        # a gap: capped
        assert float(np.mean(n.px)) == pytest.approx(0.25, abs=1e-9)

    def test_a_zero_cap_leaves_the_step_alone(self):
        n = straight(node(motion_noise_xy=0.0, motion_noise_theta=0.0,
                          gap_dt=0.15, gap_step_cap_m=0.0))
        n.predict(0.5)
        assert float(np.mean(n.px)) == pytest.approx(2.5, abs=1e-9)


class TestGapNoise:

    def test_a_gap_widens_the_cloud_instead_of_relocalizing(self):
        wide = straight(node(gap_dt=0.15, gap_noise_mult=4.0, n_particles=4000))
        tight = straight(node(gap_dt=0.15, n_particles=4000))
        np.random.seed(0)
        wide.predict(0.5)
        np.random.seed(0)
        tight.predict(0.5)
        assert np.std(wide.py) > 3.5 * np.std(tight.py)
        assert np.std(wide.pth) > 3.5 * np.std(tight.pth)

    def test_a_normal_period_is_not_widened(self):
        a = straight(node(gap_dt=0.15, gap_noise_mult=4.0, n_particles=4000))
        b = straight(node(n_particles=4000))
        np.random.seed(1)
        a.predict(0.058)
        np.random.seed(1)
        b.predict(0.058)
        assert float(np.std(a.py)) == pytest.approx(float(np.std(b.py)), rel=1e-9)


class TestPatienceFloor:

    def test_one_bad_scan_declares_the_filter_lost_after_a_gap(self):
        # dt 0.5 s -> round(0.52 / 0.5) == 1, which is the shipped behaviour
        # and the reason a post-gap pose can be dropped entirely when
        # publish_on_reloc is false.
        n = node()
        n.repoint(0.5)
        assert n._patience == 1

    def test_the_floor_keeps_a_blind_window_of_more_than_one_scan(self):
        n = node(lost_patience_min=3)
        n.repoint(0.5)
        assert n._patience == 3

    def test_the_floor_never_shortens_the_derived_count(self):
        a, b = node(), node(lost_patience_min=3)
        a.repoint(0.058)
        b.repoint(0.058)
        assert b._patience == a._patience == 9


class TestScanIntervalClamp:

    def test_the_clamp_is_what_bounds_the_motion_step(self):
        assert node().scan_dt_max == 0.5
        assert node(scan_dt_max=1.2).scan_dt_max == 1.2


class TestEncoderGapCeiling:

    def test_by_default_a_half_second_gap_is_not_differentiated(self):
        se = vehicle.SpeedEstimator(alpha=1.0)
        se.update('L', 0.0, 0.0)
        se.update('L', 0.1, 5.0 / vehicle.WHEEL_RADIUS * 0.1)
        assert se.speed == pytest.approx(5.0, rel=1e-6)
        # The car stands still for 0.6 s (a stalled simulator): the encoder
        # says nothing, so the caller still believes 5 m/s.
        se.update('L', 0.7, 5.0 / vehicle.WHEEL_RADIUS * 0.1)
        assert se.speed == pytest.approx(5.0, rel=1e-6)

    def test_raising_the_ceiling_lets_the_stall_be_measured(self):
        se = vehicle.SpeedEstimator(alpha=1.0)
        se.max_gap = 1.5
        se.update('L', 0.0, 0.0)
        se.update('L', 0.1, 5.0 / vehicle.WHEEL_RADIUS * 0.1)
        se.update('L', 0.7, 5.0 / vehicle.WHEEL_RADIUS * 0.1)
        assert se.speed == pytest.approx(0.0, abs=1e-9)

    def test_localization_only_touches_the_ceiling_when_asked(self):
        assert node().speed_est.max_gap == vehicle.MAX_ENC_GAP
        assert node(enc_gap_max=1.5).speed_est.max_gap == 1.5
