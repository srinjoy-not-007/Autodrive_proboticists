"""The opt-in rack model (`steer_tau`, `model_cmd_tau`) and the spatial
publish lead (`lead_mode`, `lead_m0`, `lead_k`, `lead_min_m`).

Research R3: the sim's steering rack is a wall-clock 3.2 rad/s slew limiter
with ~0.07-0.10 s of dead time AND a small first-order lag (joint fits: dead
0.100 / tau 0.04 at a 0.058 s period, dead 0.14-0.16 / tau 0.06-0.08 at
0.100 s).  ``Actuator`` modelled dead time and slew but not the lag, and the
rollout the plan is optimised against inherited that.  ``lead_mode=spatial``
replaces the lead in SECONDS (which `rate_adaptive` has to scale) with a lead
in METRES (which no sensor period can change).

Both are 0.0 / 'temporal' by default and the contract defended here is that at
those defaults nothing moves at all.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from pit_crew import vehicle                                     # noqa: E402
from pit_crew.mpc3_core import DEFAULTS, MPC3, Actuator, RefPath  # noqa: E402

from test_mpc3_core import straight_path as straight_csv          # noqa: E402

I0 = 300


def straight_path(tmp_path):
    return RefPath(straight_csv(tmp_path))


def drive(path, frames=25, offset=0.05, **cfg):
    m = MPC3(path, **cfg)
    return np.array([m.solve(offset, 0.0, 0.0, I0, 4.0)[0]
                     for _ in range(frames)])


# --------------------------------------------------------------------- defaults
def test_defaults_are_inert():
    assert DEFAULTS['steer_tau'] == 0.0
    assert DEFAULTS['model_cmd_tau'] == 0.0
    assert DEFAULTS['lead_mode'] == 'temporal'


def test_knobs_off_are_the_identity(tmp_path):
    path = straight_path(tmp_path)
    a = drive(path)
    b = drive(path, steer_tau=0.0, model_cmd_tau=0.0, lead_mode='temporal')
    assert np.array_equal(a, b)


def test_actuator_default_is_a_pure_slew_limiter():
    a = Actuator(dead_time=0.0)
    b = Actuator(dead_time=0.0, steer_tau=0.0)
    for act in (a, b):
        act.record(0.0, 0.3)
    assert a.delta_at(0.5) == b.delta_at(0.5)
    # 0.3 rad at 3.2 rad/s is 0.094 s of slew, so it is there by 0.2 s.
    assert abs(a.delta_at(0.2) - 0.3) < 1e-6


def test_actuator_lag_delays_the_rack_but_keeps_the_same_endpoint():
    slew = Actuator(dead_time=0.0)
    lag = Actuator(dead_time=0.0, steer_tau=0.06)
    for act in (slew, lag):
        act.record(0.0, 0.3)
    assert lag.delta_at(0.10) < slew.delta_at(0.10) - 0.02
    assert abs(lag.delta_at(1.0) - 0.3) < 1e-3
    # ... and it never moves faster than the rack: the lag only subtracts.
    prev = 0.0
    for k in range(1, 40):
        cur = lag.delta_at(0.01 * k)
        assert cur - prev <= 3.2 * 0.01 + 1e-6
        prev = cur


# ----------------------------------------------------------------- the rollout
def _rollout_case(path, **cfg):
    m = MPC3(path, **cfg)
    m.last_cmd, m.last_cmd_raw = -0.05, -0.04
    N, nd = m.N, m.nd
    rng = np.random.default_rng(0)
    u = rng.uniform(-2.0, 2.0, N - nd)
    args = (u, (0.08, 0.05, -0.03), np.full(N, 0.25), path.kappa[:N].copy(),
            np.full(N + 1, 4.0), 0.30, np.full(nd, -0.04))
    m._apply = (nd + 3, 0.4)
    return m, args


def test_rollout_lag_makes_the_angle_trail_the_command(tmp_path):
    path = straight_path(tmp_path)
    m, args = _rollout_case(path, steer_tau=0.06)
    _e, _p, d, _S = m._rollout(*args)
    dc = m._pub[0]
    nd = m.nd
    # The command plan is the unlagged integration of u; the angle follows it.
    assert np.abs(d[nd + 1:] - dc[nd + 1:]).max() > 0.01
    assert np.abs(d[nd + 1:]).max() <= np.abs(dc[nd + 1:]).max() + 1e-9


def test_rollout_without_lag_publishes_the_plan_angle_itself(tmp_path):
    path = straight_path(tmp_path)
    m, args = _rollout_case(path)
    m._rollout(*args)
    assert m._pub is None


def test_rollout_jacobian_matches_finite_differences(tmp_path):
    """The Gauss-Newton step is only as good as the Jacobian of the chain."""
    path = straight_path(tmp_path)
    for cfg in (dict(w_dcmd=50.0, steer_tau=0.06),
                dict(w_dcmd=50.0, cmd_tau=0.10, model_cmd_tau=1.0),
                dict(w_dcmd=50.0, steer_tau=0.06, cmd_tau=0.10,
                     model_cmd_tau=1.0)):
        m, args = _rollout_case(path, **cfg)
        N = m.N
        lo, hi = np.full(N, -1.0), np.full(N, 1.0)
        e, psi, d, S = m._rollout(*args, jac=True)
        r, J = m._residuals(args[0], e, psi, d, lo, hi, jac=S)
        h = 1e-6
        for j in range(len(args[0])):
            col = np.zeros_like(r)
            for sgn in (1, -1):
                u2 = args[0].copy()
                u2[j] += sgn * h
                e2, p2, d2, _ = m._rollout(u2, *args[1:])
                r2, _ = m._residuals(u2, e2, p2, d2, lo, hi)
                col += sgn * r2 / (2 * h)
            # The lateral, heading, steering and cross-frame rows; the jerk row
            # at i = 0 has a known constant-Jacobian quirk that predates this.
            keep = np.ones(len(r), bool)
            keep[5 * N + 1:5 * N + 3] = False
            assert np.abs(J[keep, j] - col[keep]).max() < 1e-5, (cfg, j)


def test_model_cmd_tau_is_inert_without_the_filter(tmp_path):
    """It models `cmd_tau`; with no filter to model it must change nothing."""
    path = straight_path(tmp_path)
    a = drive(path, w_dcmd=50.0)
    b = drive(path, w_dcmd=50.0, model_cmd_tau=1.0)
    assert np.array_equal(a, b)


def test_modelling_the_filter_changes_the_plan_and_stays_bounded(tmp_path):
    """With the filter in the rollout the solver plans against a command it
    knows will be smoothed, so the command stream is not the same one -- and
    it must still be a converging, in-cap sequence."""
    path = straight_path(tmp_path)
    off = 0.12
    a = drive(path, offset=off, w_dcmd=50.0, cmd_tau=0.10)
    b = drive(path, offset=off, w_dcmd=50.0, cmd_tau=0.10, model_cmd_tau=1.0)
    assert not np.array_equal(a, b)
    assert np.isfinite(b).all()
    assert np.abs(b).max() <= vehicle.MAX_STEER + 1e-9
    assert abs(b[-1] - b[-2]) < 0.01


# ------------------------------------------------------------- spatial lead
def _warm_index(path, lead_m, v=4.0, **cfg):
    """Publish index and the distance it sits ahead, on a KNOWN warm plan.

    The index is fixed before the solve (``w_dcmd`` charges that angle), so it
    is walked along the speed plan the previous frame left behind -- which is
    what is pinned here so the arithmetic is checkable.
    """
    cfg.setdefault('lead_min_m', 0.0)
    m = MPC3(path, lead_mode='spatial', lead_m0=lead_m, lead_k=0.0, **cfg)
    m.v_plan = np.full(m.N + 1, v)
    m.solve(0.0, 0.0, 0.0, I0, v)
    k, frac = m._apply
    ds = max(v * m.dt, m.cfg['min_step_m'])
    return k + frac, (k - m.nd + frac) * ds


def test_spatial_lead_index_is_the_asked_distance_ahead(tmp_path):
    path = straight_path(tmp_path)
    for lead in (0.4, 0.8, 1.2):
        _k, walked = _warm_index(path, lead)
        assert abs(walked - lead) < 1e-6


def test_spatial_lead_index_grows_with_the_asked_distance(tmp_path):
    path = straight_path(tmp_path)
    assert _warm_index(path, 1.2)[0] > _warm_index(path, 0.4)[0] + 1.0


def test_spatial_lead_proportional_to_speed_is_a_time_lead(tmp_path):
    """`lead_k` v metres is `lead_k` seconds while the plan holds one speed --
    the point of the spatial form is that it does NOT drift where the plan's
    speed changes, i.e. at a corner entry."""
    path = straight_path(tmp_path)
    idx = []
    for v in (2.0, 6.0):
        m = MPC3(path, lead_mode='spatial', lead_m0=0.0, lead_k=0.09,
                 lead_min_m=0.0)
        m.v_plan = np.full(m.N + 1, v)
        m.solve(0.0, 0.0, 0.0, I0, v)
        idx.append(m._apply)
    assert idx[0] == idx[1]


def test_spatial_lead_is_rate_invariant_where_the_temporal_one_is_not(tmp_path):
    """`apply_ahead_s` is a time, so `rate_adaptive` scales it and the publish
    index moves with the sensor period.  A distance does not."""
    path = straight_path(tmp_path)
    idx = []
    for ahead in (0.058 * 0.87, 0.104 * 0.87):
        m = MPC3(path, apply_ahead_s=ahead)
        m.solve(0.05, 0.0, 0.0, I0, 4.0)
        idx.append(m._apply)
    assert idx[0] != idx[1]
    sp = []
    for ahead in (0.058 * 0.87, 0.104 * 0.87):
        m = MPC3(path, apply_ahead_s=ahead, lead_mode='spatial')
        m.solve(0.05, 0.0, 0.0, I0, 4.0)
        sp.append(m._apply)
    assert sp[0] == sp[1]


def test_spatial_lead_floor_keeps_the_index_off_the_committed_knot(tmp_path):
    path = straight_path(tmp_path)
    m = MPC3(path, lead_mode='spatial', lead_m0=0.0, lead_k=0.0,
             lead_min_m=0.0)
    m.solve(0.05, 0.0, 0.0, I0, 4.0)
    k, frac = m._apply
    assert k + frac >= m.nd + m.cfg['apply_floor_steps'] - 1e-9


def test_spatial_lead_floor_binds_at_walking_pace(tmp_path):
    """At 0.5 m/s `lead_k * v` is 4.5 cm; the floor is what keeps the publish
    point off the committed knot."""
    path = straight_path(tmp_path)
    _k, walked = _warm_index(path, 0.0, v=0.5, lead_min_m=0.75,
                             apply_floor_steps=0.0)
    assert abs(walked - 0.75) < 1e-6


def test_solve_stays_finite_with_everything_on(tmp_path):
    path = straight_path(tmp_path)
    m = MPC3(path, w_dcmd=50.0, cmd_tau=0.10, model_cmd_tau=1.0,
             steer_tau=0.06, lead_mode='spatial')
    for _ in range(30):
        cmd, info = m.solve(0.15, 0.1, -0.05, I0, 5.0)
        assert math.isfinite(cmd) and abs(cmd) <= 0.5236 + 1e-9
        assert np.isfinite(info['delta']).all()


# ------------------------------------------------------- lead_mode 'time'
def test_time_lead_is_off_by_default(tmp_path):
    """Defaults are byte-identical: `lead_t0`/`lead_tk` are 0 and the mode is
    'temporal', so nothing reads them."""
    assert DEFAULTS['lead_t0'] == 0.0 and DEFAULTS['lead_tk'] == 0.0
    path = straight_path(tmp_path)
    a = MPC3(path)
    b = MPC3(path, lead_t0=0.0, lead_tk=0.0)
    a.solve(0.05, 0.0, 0.0, I0, 4.0)
    b.solve(0.05, 0.0, 0.0, I0, 4.0)
    assert a._apply == b._apply


def test_time_lead_is_a_fixed_number_of_sensor_periods(tmp_path):
    """The whole point: the readout lead in SECONDS scales with the measured
    period, so the lead in periods is the same at 0.058 and 0.094 s -- where
    the shipping 'temporal' law gives 4.0 periods at the fast rate and 2.8 at
    the slow one, which is the sign flip this mode exists to remove.  `tk` is
    above 4.2 here so the `nd + apply_floor_steps` floor does not bind at the
    fast rate (with the default `nd` = 3 that floor is 0.24 s)."""
    path = straight_path(tmp_path)
    leads = []
    for period in (0.058, 0.094):
        m = MPC3(path, lead_mode='time', lead_t0=0.0, lead_tk=5.0)
        m.period = period
        m.solve(0.05, 0.0, 0.0, I0, 4.0)
        k, frac = m._apply
        leads.append((k + frac) * m.dt / period)
    assert abs(leads[0] - leads[1]) < 1e-6
    assert abs(leads[0] - 5.0) < 1e-6


def test_time_lead_keeps_the_index_off_the_committed_knot(tmp_path):
    """Read at `nd` itself and the published angle is the one already in
    flight: measured, a 0.058 s harness cell went 0 -> 236 collisions."""
    path = straight_path(tmp_path)
    m = MPC3(path, lead_mode='time', lead_t0=0.0, lead_tk=0.1)
    m.period = 0.058
    m.solve(0.05, 0.0, 0.0, I0, 4.0)
    k, frac = m._apply
    assert k + frac >= m.nd + m.cfg['apply_floor_steps'] - 1e-9
