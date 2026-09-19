"""The residual policy is bounded, gated, opt-in and numpy-only.

The properties that matter for shipping, in order: with no `residual_policy`
parameter nothing changes at all; a policy that IS loaded can never move the
steering more than 0.03 rad or the speed setpoint outside 0.95-1.05; and it can
never ADD speed in a corner or a declared slow zone, which is where every live
failure on this track lives.
"""

import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from pit_crew import residual_policy as RP                      # noqa: E402


def _npz(tmp_path, **over):
    """A deliberately SATURATED net: every output is +1 after tanh, so the
    bounds are the only thing between it and the actuator."""
    rng = np.random.default_rng(0)
    n = RP.N_OBS
    d = dict(W0=np.zeros((n, 4)), b0=np.full(4, 5.0),
             W1=np.full((4, 2), 5.0), b1=np.full(2, 5.0),
             obs_mean=np.zeros(n), obs_std=np.ones(n))
    d.update(over)
    p = str(tmp_path / 'pol.npz')
    np.savez(p, **d)
    return p


def test_load_of_an_empty_parameter_is_none():
    assert RP.load('') is None
    assert RP.load(None) is None
    assert RP.load('   ') is None


def test_load_of_a_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        RP.load('/nonexistent/policy.npz')


def test_bounds_are_hard(tmp_path):
    pol = RP.ResidualPolicy(_npz(tmp_path))
    d_steer, v_scale = pol.act(np.zeros(RP.N_OBS), s=8.0, kappa=0.0)
    assert d_steer == pytest.approx(RP.STEER_MAX, rel=1e-9)
    assert v_scale == pytest.approx(RP.V_HI, rel=1e-9)
    assert abs(d_steer) <= RP.STEER_MAX + 1e-12
    assert RP.V_LO - 1e-12 <= v_scale <= RP.V_HI + 1e-12


def test_an_npz_may_narrow_the_bounds_but_never_widen_them(tmp_path):
    wide = RP.ResidualPolicy(_npz(tmp_path, steer_max=np.float64(0.5),
                                  v_lo=np.float64(0.5),
                                  v_hi=np.float64(2.0)))
    assert wide.steer_max == RP.STEER_MAX
    assert wide.v_lo == RP.V_LO
    assert wide.v_hi == RP.V_HI
    narrow = RP.ResidualPolicy(_npz(tmp_path, steer_max=np.float64(0.01),
                                    v_hi=np.float64(1.0)))
    assert narrow.steer_max == 0.01
    assert narrow.v_hi == 1.0


def test_no_speed_up_in_a_corner(tmp_path):
    pol = RP.ResidualPolicy(_npz(tmp_path))
    _d, v = pol.act(np.zeros(RP.N_OBS), s=8.0, kappa=0.35)
    assert v == 1.0
    _d, v = pol.act(np.zeros(RP.N_OBS), s=8.0, kappa=0.25)
    assert v > 1.0


def test_no_speed_up_in_a_slow_zone_including_one_that_wraps(tmp_path):
    pol = RP.ResidualPolicy(_npz(
        tmp_path, zones=np.array([[17.5, 21.5], [42.0, 2.0]]),
        track_len=np.float64(44.82)))
    for s in (17.5, 19.0, 21.5, 42.5, 44.8, 0.0, 1.9):
        assert pol.act(np.zeros(RP.N_OBS), s=s, kappa=0.0)[1] == 1.0, s
    for s in (8.0, 25.0, 41.0, 3.0):
        assert pol.act(np.zeros(RP.N_OBS), s=s, kappa=0.0)[1] > 1.0, s


def test_slowing_down_is_allowed_everywhere(tmp_path):
    pol = RP.ResidualPolicy(_npz(
        tmp_path, W1=np.full((4, 2), -5.0), b1=np.full(2, -5.0),
        zones=np.array([[17.5, 21.5]]), track_len=np.float64(44.82)))
    _d, v = pol.act(np.zeros(RP.N_OBS), s=19.0, kappa=1.5)
    assert v == pytest.approx(RP.V_LO, rel=1e-9)


def test_a_non_finite_net_falls_back_to_the_identity(tmp_path):
    pol = RP.ResidualPolicy(_npz(tmp_path, b1=np.full(2, np.nan)))
    assert pol.act(np.zeros(RP.N_OBS), s=8.0, kappa=0.0) == (0.0, 1.0)


def test_a_wrong_shaped_archive_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        RP.ResidualPolicy(_npz(tmp_path, obs_mean=np.zeros(3),
                               obs_std=np.ones(3)))
    with pytest.raises(ValueError):
        RP.ResidualPolicy(_npz(tmp_path, W1=np.full((4, 3), 1.0),
                               b1=np.zeros(3)))


def test_inference_imports_nothing_but_numpy():
    src = open(os.path.join(ROOT, 'src/pit_crew/residual_policy.py')).read()
    for banned in ('import torch', 'import scipy', 'from scipy',
                   'sklearn', 'import matplotlib'):
        assert banned not in src


def test_observation_shape_and_wall_terms():
    from pit_crew.mpc2_core import RefPath
    line = os.path.join(ROOT,
                        'data/racelines/finals_mesh_exitm_m10t6_h3_race.csv')
    path = RefPath(line)
    obs = RP.observation(path, 10, 0.05, -0.02, 4.0, 0.097)
    assert obs.shape == (RP.N_OBS,)
    assert obs[0] == 0.05 and obs[2] == 4.0 and obs[3] == 0.097
    # no corridor loaded -> the 2.0 m probe ceiling, not a crash
    assert np.all(obs[-2 * len(RP.PREVIEW_WALL_M):] == 2.0)


def test_mpc3_default_leaves_the_policy_off():
    src = open(os.path.join(ROOT, 'src/pit_crew/mpc3.py')).read()
    assert "('residual_policy', '')" in src
    assert 'self.residual = _rp.load' in src
    assert 'if self.residual is not None:' in src
