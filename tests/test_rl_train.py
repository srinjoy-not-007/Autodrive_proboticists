"""The numpy PPO's gradients are right, and its export is the net it trained.

Hand-written backprop is exactly the kind of thing that is silently 5 % wrong
and still looks like it is learning, so it is checked against finite
differences; and the whole point of training in numpy is that the arrays that
came out of training ARE the arrays that ship, so the export is checked to
reproduce the trainer's forward pass bit for bit.
"""

import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from pit_crew import residual_policy as RP                      # noqa: E402
import rl_train as T                                            # noqa: E402


@pytest.mark.parametrize('tanh_out', [True, False])
def test_mlp_gradients_match_finite_differences(tanh_out):
    rng = np.random.default_rng(0)
    Ws, bs = T.init_mlp((5, 7, 2), rng, out_scale=0.5)
    x = rng.normal(size=(4, 5))
    targ = rng.normal(size=(4, 2))

    def loss():
        y, _ = T.mlp_forward(Ws, bs, x, tanh_out)
        return float(np.sum((y - targ) ** 2))

    y, cache = T.mlp_forward(Ws, bs, x, tanh_out)
    gW, gb = T.mlp_backward(Ws, cache, 2.0 * (y - targ), tanh_out)
    eps = 1e-6
    worst = 0.0
    for li in range(len(Ws)):
        for arr, g in ((Ws[li], gW[li]), (bs[li], gb[li])):
            for idx in np.ndindex(arr.shape):
                old = arr[idx]
                arr[idx] = old + eps
                lp = loss()
                arr[idx] = old - eps
                lm = loss()
                arr[idx] = old
                num = (lp - lm) / (2 * eps)
                worst = max(worst, abs(num - g[idx]) / max(1.0, abs(num)))
    assert worst < 1e-5


def test_export_reproduces_the_trainers_forward_pass(tmp_path):
    rng = np.random.default_rng(1)
    ag = T.Agent(RP.N_OBS, rng)
    # Give it non-trivial weights: a freshly initialised output layer is tiny
    # and would hide a scaling bug.
    for W in ag.pW:
        W += rng.normal(0.0, 0.3, size=W.shape)
    ag.obs_mean = rng.normal(size=RP.N_OBS)
    ag.obs_std = 0.5 + np.abs(rng.normal(size=RP.N_OBS))
    p = str(tmp_path / 'pol.npz')
    T.export(ag, p, track_len=44.82)
    pol = RP.ResidualPolicy(p)
    for _ in range(20):
        obs = rng.normal(size=RP.N_OBS) * 2.0
        want = ag.mean(np.atleast_2d(obs))[0]
        got = pol._forward(obs)
        assert np.allclose(want, got, atol=1e-12)


def test_export_carries_the_bounds_and_the_slow_zones(tmp_path):
    ag = T.Agent(RP.N_OBS, np.random.default_rng(0))
    p = str(tmp_path / 'pol.npz')
    T.export(ag, p, track_len=44.82)
    d = np.load(p)
    assert float(d['steer_max']) == RP.STEER_MAX
    assert float(d['v_lo']) == RP.V_LO and float(d['v_hi']) == RP.V_HI
    assert d['zones'].shape == (3, 2)
    assert float(d['track_len']) == pytest.approx(44.82)
    pol = RP.ResidualPolicy(p)
    for s in (18.0, 38.0, 43.0, 1.0):
        assert pol.no_speedup(s, 0.0), s
    for s in (8.0, 25.0, 31.0):
        assert not pol.no_speedup(s, 0.0), s


def test_a_fresh_agent_is_almost_the_identity_residual():
    """Training must START from the shipping controller, not from noise."""
    ag = T.Agent(RP.N_OBS, np.random.default_rng(3))
    obs = np.zeros((32, RP.N_OBS))
    mu = ag.mean(obs)
    assert np.abs(mu).max() < 0.05          # |d_steer| < 0.0015 rad


def test_gae_matches_a_direct_sum():
    rew = np.array([1.0, 2.0, 3.0])
    val = np.array([0.5, 0.5, 0.5])
    g, lam = 0.9, 0.8
    adv, ret = T.gae(rew, val, g, lam)
    d = [rew[0] + g * val[1] - val[0],
         rew[1] + g * val[2] - val[1],
         rew[2] + 0.0 - val[2]]
    want2 = d[2]
    want1 = d[1] + g * lam * want2
    want0 = d[0] + g * lam * want1
    assert adv == pytest.approx([want0, want1, want2])
    assert ret == pytest.approx(adv + val)
