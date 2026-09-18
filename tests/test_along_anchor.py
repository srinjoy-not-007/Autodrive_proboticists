"""Along-track anchoring on a featureless straight (Sep 19), all opt-in.

The finals track's s 2-16 is an 18 m straight between parallel walls. The
likelihood field pins the LATERAL offset and the heading there and says
almost nothing about how far along the corridor the car is, so the
along-track coordinate of the cloud is driven by the process noise and by
resampling alone.

Measured on four finals logs (`data/logs/race_20260918_{194320,210358,
211133,211852}.csv`, truth interpolated at `pstamp` and projected on the
line tangent, collisions and the first 25 s masked):

  * the along-track error over the straight is a per-lap random walk, sd
    0.17 m (0.058 s display) to 0.32 m (0.097 s Xvfb), range -1.15 to
    +0.61 m, against a lateral error of 0.015-0.021 m (p98) on the same
    stretch;
  * it is released as a systematic +0.28 to +0.35 m STEP at s ~ 10.5, the
    first place on the straight where the geometry says anything
    along-track: p90 0.53 m and max 1.27 m in ONE 0.097 s scan, where the
    car itself travels 0.78 m.

Nothing here changes the shipped filter: every default is off and the
first class asserts that.
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


def on_the_straight(n, v=7.9, yaw=0.0):
    """A converged cloud driving east at `v`, one scan already behind it."""
    n.init_particles_near(0.0, 0.0, 0.0)
    n.px[:] = 0.0
    n.py[:] = 0.0
    n.pth[:] = 0.0
    n.imu_yaw = yaw
    n.speed_est.speed = v
    n._prev_mean = (0.0, 0.0)
    n._last_v = v
    n._last_jumped = False
    n._just_relocalized = False
    return n


def shift(n, along, yaw=0.0):
    """Move the whole cloud `along` metres down its own heading."""
    n.px += along * math.cos(yaw)
    n.py += along * math.sin(yaw)


class TestDefaultsAreInert:

    def test_every_new_parameter_is_off(self):
        n = node()
        assert n.along_clamp_mps == 0.0
        assert n.along_clamp_min_v == 1.0
        assert n.noise_along == 0.0

    def test_the_clamp_does_nothing_at_the_default(self):
        n = on_the_straight(node())
        n._last_step = 0.78
        shift(n, 5.0)                       # an absurd along-track jump
        assert n.anchor_along(0.097) == 0.0
        assert float(np.mean(n.px)) == pytest.approx(5.0)

    def test_the_default_process_noise_is_bit_identical(self):
        """The isotropic branch must draw the same numbers in the same order,
        or every tuned default moves under us."""
        out = []
        for along in (0.0, 0.0):
            n = on_the_straight(node(motion_noise_along=along))
            np.random.seed(1234)
            n.predict(0.097)
            out.append((n.px.copy(), n.py.copy(), n.pth.copy()))
        for a, b in zip(out[0], out[1]):
            assert np.array_equal(a, b)

    def test_splitting_the_noise_changes_the_cloud(self):
        n = on_the_straight(node(motion_noise_along=0.02))
        np.random.seed(1234)
        n.predict(0.097)
        m = on_the_straight(node())
        np.random.seed(1234)
        m.predict(0.097)
        assert not np.array_equal(n.px, m.px)


class TestTheClampBoundsTheJump:

    def test_a_forward_jump_is_cut_back_to_the_allowance(self):
        n = on_the_straight(node(along_clamp_mps=4.0))
        n._last_step = 0.78                 # what the encoders paid for
        shift(n, 1.27)                      # the measured worst single scan
        removed = n.anchor_along(0.097)
        # allowance 0.78 + 4.0 * 0.097 = 1.168 m
        assert removed == pytest.approx(1.27 - 1.168, abs=1e-6)
        assert float(np.mean(n.px)) == pytest.approx(1.168, abs=1e-6)

    def test_a_jump_inside_the_allowance_is_untouched(self):
        n = on_the_straight(node(along_clamp_mps=4.0))
        n._last_step = 0.78
        shift(n, 1.0)
        assert n.anchor_along(0.097) == 0.0
        assert float(np.mean(n.px)) == pytest.approx(1.0)

    def test_it_bounds_a_BACKWARD_jump_too(self):
        """The straight's error runs both ways: the per-lap minimum measured
        -1.15 m, i.e. the estimate a metre BEHIND the car."""
        n = on_the_straight(node(along_clamp_mps=4.0))
        n._last_step = 0.78
        shift(n, -0.5)
        removed = n.anchor_along(0.097)
        assert removed == pytest.approx(-0.5 - (0.78 - 0.388), abs=1e-6)
        assert float(np.mean(n.px)) == pytest.approx(0.78 - 0.388, abs=1e-6)

    def test_the_allowance_is_a_speed_so_it_survives_the_period_lottery(self):
        """0.058 s on a real display, 0.088-0.106 s on Xvfb. A per-SCAN metre
        allowance would mean two different filters."""
        for dt in (0.058, 0.097, 0.104):
            n = on_the_straight(node(along_clamp_mps=4.0))
            n._last_step = 7.9 * dt
            shift(n, 7.9 * dt + 4.0 * dt + 0.25)
            assert n.anchor_along(dt) == pytest.approx(0.25, abs=1e-6)

    def test_the_whole_cloud_moves_not_just_the_published_mean(self):
        n = on_the_straight(node(along_clamp_mps=4.0))
        n._last_step = 0.78
        n.px += np.linspace(-0.2, 0.2, n.n_p)     # a cloud with real spread
        spread = n.px.std()
        shift(n, 2.0)
        n.anchor_along(0.097)
        assert n.px.std() == pytest.approx(spread, abs=1e-9)

    def test_it_clamps_along_the_HEADING_not_along_x(self):
        yaw = 0.7
        n = on_the_straight(node(along_clamp_mps=4.0), yaw=yaw)
        n._last_step = 0.78
        shift(n, 2.0, yaw)
        n.anchor_along(0.097)
        adv = (float(np.mean(n.px)) * math.cos(yaw)
               + float(np.mean(n.py)) * math.sin(yaw))
        assert adv == pytest.approx(1.168, abs=1e-6)
        lat = (-float(np.mean(n.px)) * math.sin(yaw)
               + float(np.mean(n.py)) * math.cos(yaw))
        assert lat == pytest.approx(0.0, abs=1e-9)


class TestItStandsDownWhereAJumpIsCorrect:

    def test_the_first_scan_is_never_clamped(self):
        n = on_the_straight(node(along_clamp_mps=4.0))
        n._prev_mean = None
        n._last_step = 0.78
        shift(n, 5.0)
        assert n.anchor_along(0.097) == 0.0

    def test_a_relocalization_is_never_clamped(self):
        """`init_particles_global` moves the cloud the length of the track on
        purpose; holding it to the encoder would undo the recovery."""
        n = on_the_straight(node(along_clamp_mps=4.0))
        n._last_step = 0.78
        n._just_relocalized = True
        shift(n, 5.0)
        assert n.anchor_along(0.097) == 0.0

    def test_a_teleport_is_never_clamped(self):
        n = on_the_straight(node(along_clamp_mps=4.0))
        n._last_step = 0.0
        n._last_jumped = True
        shift(n, -3.0)
        assert n.anchor_along(0.097) == 0.0

    def test_a_teleport_hold_is_never_clamped(self):
        n = on_the_straight(node(along_clamp_mps=4.0))
        n._last_step = 0.0
        n._tp_hold = 3
        shift(n, -3.0)
        assert n.anchor_along(0.097) == 0.0

    def test_it_is_inert_below_the_speed_floor(self):
        """Standing on the grid, the encoder vouches for nothing."""
        n = on_the_straight(node(along_clamp_mps=4.0, along_clamp_min_v=1.0),
                            v=0.2)
        n._last_v = 0.2
        n._last_step = 0.02
        shift(n, 2.0)
        assert n.anchor_along(0.097) == 0.0

    def test_a_zero_interval_is_never_clamped(self):
        n = on_the_straight(node(along_clamp_mps=4.0))
        n._last_step = 0.0
        shift(n, 2.0)
        assert n.anchor_along(0.0) == 0.0


class TestItIsWiredIntoTheScanPath:

    def test_on_scan_calls_it(self):
        src = open(os.path.join(SRC, 'pit_crew', 'localization.py')).read()
        body = src.split('def on_scan(', 1)[1].split('\n    def ', 1)[0]
        assert 'self.anchor_along(dt)' in body
        # ...after the scan update, or it would clamp the prediction instead
        # of the posterior the controller is actually given.
        assert body.index('self.update(scan)') < body.index('self.anchor_along')

    def test_predict_records_what_the_encoder_paid_for(self):
        n = on_the_straight(node(motion_noise_xy=0.0, motion_noise_theta=0.0,
                                 max_step_mps=9.0))   # the shipping cap
        n.predict(0.1)
        assert n._last_step == pytest.approx(0.79)
        assert n._last_v == pytest.approx(7.9)
        assert n._last_jumped is False
