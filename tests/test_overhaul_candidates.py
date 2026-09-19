"""The three overhaul candidates: they construct, they drive, mpc3 is untouched.

`docs/research/overhaul_mpcc_sep20.md` has the numbers.  What this file pins is
the part that cannot be checked by racing them:

* the hooks added to `mpc3.py` and `tools/sim_tracker.py` are the IDENTITY for
  mpc3, so nothing in `docs/FINALS_TUNING.md` moves;
* each new node constructs against the rclpy stub and publishes finite,
  in-range commands for a full 10 Hz stream (rclpy is not installed on the
  host, so without this the first time a node's `__init__` runs is on the
  organisers' box);
* `MPCC`'s envelope is the MEASURED one -- a yaw-rate limit with an ay
  ceiling, not a constant-ay circle -- and its hinges are soft;
* `PlanReplay` at gain 0 returns exactly what the solver published.
"""

import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'src'))

import test_node_smoke as NS                                    # noqa: E402
from pit_crew import vehicle                                    # noqa: E402
from pit_crew.mpcc_core import (AY_BREAK, MPCC, MPCC_DEFAULTS,   # noqa: E402
                                R_BREAK, envelope_speed)
from pit_crew.planreplay import PlanReplay                       # noqa: E402

LINE = NS.MPC3_LINE
MAP = NS.MAP_PREFIX


# --------------------------------------------------------- mpc3 is untouched

def test_mpc3_node_hooks_are_the_identity():
    from pit_crew.mpc3 import MPC3Tracker
    n = NS.make_node(MPC3Tracker, raceline_csv=LINE, map_prefix=MAP)
    assert n.extra_solver_args({'anything': 1}) == {}
    sentinel = object()
    assert n.post_throttle(0.37, {'brake': True}, 4.0) == 0.37
    assert n.post_steer(0.11, {}, 0.0, 0, 0.0, 0.0, 4.0, 0.5) == 0.11
    assert n.MPC_CLASS.__name__ == 'MPC3'
    assert n.EXTRA_PARAMS == ()
    _ = sentinel


def test_harness_mpc3_hooks_are_the_identity():
    import sim_tracker as ST
    from pit_crew.mpc2_core import RefPath
    path = RefPath(LINE)
    tr = ST.MPC3Sim(path)
    assert tr.post_throttle(0.42, {'brake': True}, 4.0) == 0.42
    assert tr.post_steer(0.07, {}, 0.0, 0, 0.0, 0.0, 4.0, 0.5) == 0.07
    assert tr.MPC_CLASS.__name__ == 'MPC3'
    assert tr.EXTRA_SOLVER_KEYS == ()


# ------------------------------------------------------------ the A envelope

def test_envelope_is_a_yaw_rate_limit_with_an_ay_ceiling():
    # Refitted from the sysid staircase: breakaway is a yaw-rate limit.
    assert R_BREAK == pytest.approx(2.71)
    assert AY_BREAK == pytest.approx(10.5)
    # A hairpin (1.6 1/m) is capped by the YAW RATE, a fast sweep (0.3 1/m) by
    # the lateral acceleration.  If that ever swaps, the envelope is wrong.
    assert envelope_speed(1.6) == pytest.approx(R_BREAK / 1.6, rel=1e-6)
    assert envelope_speed(0.3) == pytest.approx(math.sqrt(AY_BREAK / 0.3),
                                                rel=1e-6)
    assert envelope_speed(0.0) == pytest.approx(9.0)      # v_max, not infinite


def test_envelope_hinges_are_soft_and_one_sided():
    """Inside the envelope they cost nothing; outside they cost, not veto."""
    from pit_crew.mpc2_core import RefPath
    path = RefPath(LINE)
    m = MPCC(path)
    N = m.N
    ones = np.ones((N + 1, 1))
    E = 0.0 * ones
    PSI = 0.0 * ones
    V = 3.0 * ones
    lo = np.full((N, 1), -1.0)
    hi = np.full((N, 1), 1.0)
    tgt = 3.0 * np.ones(N + 1)
    U = np.zeros((N - m.nd, 1))
    A = np.zeros((N - m.nd, 1))

    def cost(delta):
        D = delta * ones
        r = m._resid(U, A, E, PSI, D, V, lo, hi, tgt, 9.0, D)[:, 0]
        return float(r @ r)

    # r = v tan(d)/L: at 3 m/s, 2.71 rad/s needs tan(d) = 0.293 -> d = 0.285.
    inside = cost(0.20)
    edge = cost(0.28)
    outside = cost(0.40)
    assert edge >= inside                     # monotone, no cliff
    assert outside > edge * 1.05              # and it really does cost
    assert math.isfinite(outside)             # soft: finite, not a veto


def test_profile_cap_defaults_to_the_offline_profile():
    """MPCC may brake later; it may not invent speed the profile refused."""
    assert MPCC_DEFAULTS['v_prof_cap'] == pytest.approx(1.0)
    # ... and it plans on MORE brake than the profile's 4.6, which is the
    # measured throttle-0.0 deceleration and the whole point of the candidate.
    assert MPCC_DEFAULTS['lon_brake_plan'] > 4.6


# --------------------------------------------------------------- the C replay

def test_plan_replay_gain_zero_is_mpc3():
    r = PlanReplay(gain=0.0)
    r.set_plan(np.linspace(0.0, 0.5, 21), t0=10.0, dt=0.06, k0=4.5,
               cap=0.5236, cmd=0.1)
    assert not r.enabled
    assert r.angle(10.05) is None


def test_plan_replay_walks_the_plan_and_respects_the_rate_cap():
    plan = np.linspace(0.0, 2.0, 21)          # 0.1 rad per step
    r = PlanReplay(gain=1.0, cmd_tau=0.0, rate_cap=0.0)
    r.set_plan(plan, t0=10.0, dt=0.06, k0=4.0, cap=10.0, cmd=plan[4])
    assert r.angle(10.0) == pytest.approx(plan[4])
    assert r.angle(10.06) == pytest.approx(plan[5])       # one step on
    # With the rate cap on, it can never move further than the rack could.
    r2 = PlanReplay(gain=1.0, cmd_tau=0.0, rate_cap=0.5)
    r2.set_plan(plan, t0=10.0, dt=0.06, k0=4.0, cap=10.0, cmd=plan[4])
    assert r2.angle(10.06) == pytest.approx(plan[4] + 0.5 * 0.06)


# ------------------------------------------------------------- node liveness

class TestCandidateNodes:

    @pytest.mark.parametrize('name,cls_name', [
        ('mpcc', 'MPCCTracker'),
        ('ffb', 'FFBTracker'),
        ('fastpub', 'FastPubTracker'),
    ])
    def test_it_constructs_and_drives(self, name, cls_name):
        mod = __import__(f'pit_crew.{name}', fromlist=[cls_name])
        cls = getattr(mod, cls_name)
        n = NS.make_node(cls, raceline_csv=LINE, map_prefix=MAP)
        assert n.path.has_corridor
        NS.drive_tracker(n, NS.Line(LINE))
        NS.assert_commands(n, n_min=NS.CYCLES - 2)
        NS.assert_no_errors(n)

    def test_mpcc_owns_the_brake(self):
        from pit_crew.mpcc import MPCCTracker
        n = NS.make_node(MPCCTracker, raceline_csv=LINE, map_prefix=MAP,
                         brake_mode='zero')
        # The throttle law's own trigger is off; the solver's decision stands.
        assert n.thr.brake_mode == 'none'
        assert n._thr_brake_mode == 'zero'
        assert n.post_throttle(0.31, {'brake': True}, 4.0) == 0.0
        assert n.post_throttle(0.31, {'brake': False}, 4.0) == 0.31

    def test_ffb_bypass_publishes_the_solver_command(self):
        from pit_crew.ffb import FFBTracker
        n = NS.make_node(FFBTracker, raceline_csv=LINE, map_prefix=MAP,
                         ffb_bypass=True)
        assert n.post_steer(0.123, {}, 0.0, 0, 0.0, 0.0, 4.0,
                            vehicle.MAX_STEER) == 0.123
