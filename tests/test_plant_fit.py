"""The log-fitted plant is OPT-IN and its pieces do what they say.

The load-bearing property is that nothing changes without `--plant-fit` (or an
explicit `--plant-lat-law fit`): every number already in docs/STATUS.md and
docs/FINALS_TUNING.md was produced with the historical plant and has to stay
reproducible.
"""

import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'src'))

import sim_tracker as ST                                       # noqa: E402
from pit_crew import vehicle                                   # noqa: E402

L = vehicle.WHEELBASE


def _plant(delta=0.0, v=3.0, **kw):
    p = ST.Plant(0.0, 0.0, 0.0, v=v, **kw)
    # Hold the rack where the test puts it: `step` slews `delta` toward
    # `steer_target` before it computes the curvature.
    p.delta = p.steer_target = float(delta)
    return p


def test_default_lateral_law_is_the_historical_knee():
    p = _plant()
    assert p.lat_law == 'knee'
    assert p.lat_tau == 0.0
    assert p.grip_c == 0.0


def test_knee_law_unchanged_by_the_refactor():
    """The 'knee' branch must still be the exact old expression."""
    p = _plant(delta=0.30, v=5.0, ay_max=ST.AY_MAX)
    p.step(0.0)
    k_ask = math.tan(0.30) / L
    ay = 25.0 * abs(k_ask) * ST.GRIP_RATIO
    knee = ST.AY_KNEE * ST.AY_MAX
    span = ST.AY_MAX - knee
    ay = knee + span * (1.0 - math.exp(-(ay - knee) / span))
    assert p.kappa == pytest.approx(math.copysign(ay, k_ask) / 25.0, rel=1e-9)


def test_fitted_law_has_no_knee_and_delivers_more_at_high_ay():
    """At an asked ay the live logs say is deliverable, 'fit' beats 'knee'."""
    a = _plant(delta=0.35, v=2.9, lat_law='knee', ay_max=ST.AY_MAX)
    b = _plant(delta=0.35, v=2.9, lat_law='fit')
    a.step(0.0)
    b.step(0.0)
    k_ask = abs(math.tan(0.35) / L)
    ay_ask = 2.9 ** 2 * k_ask
    assert ay_ask > 8.0                      # in the hairpin band
    assert abs(b.kappa) > abs(a.kappa)
    assert abs(b.kappa) / k_ask == pytest.approx(
        ST.GRIP_A - ST.GRIP_B * ay_ask, rel=1e-6)


def test_fitted_ratio_is_clipped():
    p = _plant(delta=0.50, v=8.0, lat_law='fit')   # ay_ask ~ 100
    p.step(0.0)
    assert abs(p.kappa) / abs(math.tan(0.50) / L) == pytest.approx(
        ST.GRIP_LO, rel=1e-6)


def test_lateral_pole_is_first_order():
    # servo_k 0 and no drag: the speed is frozen, so the only thing moving is
    # the lateral pole.
    p = _plant(delta=0.20, v=4.0, lat_law='fit', lat_tau=ST.LAT_TAU,
               servo_k=0.0, drag=0.0)
    for _ in range(3):
        p.step(0.0)
    k3 = p.kappa
    for _ in range(600):                      # ~0.6 s, many time constants
        p.step(0.0)
    k_ss = p.kappa
    assert abs(k3) < 0.5 * abs(k_ss)          # still rising after 3 ms
    k_ask = abs(math.tan(0.20) / L)
    ay = 16.0 * k_ask
    assert abs(k_ss) / k_ask == pytest.approx(
        ST.GRIP_A - ST.GRIP_B * ay, rel=2e-3)


def test_friction_ellipse_is_off_below_its_threshold():
    p = _plant(delta=0.20, v=2.0, lat_law='fit', grip_c=0.06)
    p.accel = 3.0
    k_ask = abs(math.tan(0.20) / L)
    ay_low = 4.0 * k_ask
    assert ay_low < ST.GRIP_AY0               # no coupling here
    p.step(0.0)
    assert abs(p.kappa) / k_ask == pytest.approx(
        ST.GRIP_A - ST.GRIP_B * ay_low, rel=1e-6)


def test_friction_ellipse_bites_above_its_threshold():
    k_ask = abs(math.tan(0.35) / L)
    ay = 2.9 ** 2 * k_ask
    assert ay > ST.GRIP_AY0
    a = _plant(delta=0.35, v=2.9, lat_law='fit')
    b = _plant(delta=0.35, v=2.9, lat_law='fit', grip_c=0.06)
    b.accel = 2.0
    a.step(0.0)
    b.step(0.0)
    assert abs(b.kappa) < abs(a.kappa)
    want = max(ST.GRIP_LO,
               ST.GRIP_A - ST.GRIP_B * ay - 0.06 * 2.0 * (ay - ST.GRIP_AY0))
    assert abs(b.kappa) / k_ask == pytest.approx(want, rel=1e-6)


def test_brake_at_zero_is_the_measured_constant():
    assert ST.FIT_BRAKE_AT_ZERO == pytest.approx(4.15)
    p = _plant(v=4.0, lat_law='fit', brake_at_zero=ST.FIT_BRAKE_AT_ZERO,
               drag=0.273, servo_k=18.0, a_accel=5.5, a_brake=4.0)
    p.throttle = 0.0
    v0 = p.v
    p.step(0.0)
    a = (p.v - v0) / ST.DT
    # the display log measures -5.07 m/s2 at v 3-4 with the throttle at 0.0
    assert a == pytest.approx(-(4.15 + 0.273 * 4.0), abs=0.02)
    assert -5.6 < a < -4.8


def test_encoder_model_is_the_measured_one():
    """One frame of delay and ~1 % slip, not a 0.193 s pole and 3.9 %."""
    assert ST.FIT_ENC_ALPHA == 1.0
    assert 1.0 <= ST.FIT_ENC_SLIP <= 1.03


def test_pose_noise_is_heteroscedastic_in_curvature():
    assert ST.FIT_POSE_NOISE == pytest.approx(0.0095)
    assert ST.FIT_POSE_NOISE_K > 3 * ST.FIT_POSE_NOISE
    assert ST.FIT_POSE_BIAS_INSIDE < 0.0      # the bias is OUTWARD
    assert ST.FIT_POSE_BIAS_KAPPA > 1.0


def test_mpc3sim_accepts_servo_head_kappa():
    """The harness has to be able to run the shipping config at all."""
    import inspect
    src = inspect.getsource(ST.MPC3Sim.__init__)
    assert 'servo_head_kappa' in src
    assert 'head_kappa=self.P' in src


def test_surrogate_race_defaults_to_the_shipping_flags():
    import surrogate_race as SR
    for flag in ('pub_gate_corr=0.05', 'servo_head_kappa=0.4',
                 'brake_mode=zero', 'rate_adaptive=1'):
        assert flag in SR.SHIPPING
