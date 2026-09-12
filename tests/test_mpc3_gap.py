"""Telemetry-gap degraded mode (OPT-IN: `gap_predict_s`).

Measured in `tools/sim_tracker.py` with `--drop-every 5`: one dropped frame is
harmless at any period, but the stale window is 3.37 sensor periods wide at
every period the rate clamp admits (`stale_pose_s` is scaled ONE-SIDED, so it
is 0.195 s at 0.058 s and 0.350 s at 0.104 s), and three consecutive drops
exceed it.  The watchdog then holds the steering and publishes zero throttle,
which mid-corner is a collision: 12-lap runs with a 3-frame stall every 5 s
take collisions with the shipping config and none with `gap_predict_s` 0.6.

Everything here is off by default; the harness reports are byte-identical with
the feature unset (verified separately against the pre-change tree).
"""

import ast
import math
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from pit_crew.mpc3_core import (GapRider, RateAdapter,       # noqa: E402
                                RATE_TERMS, gap_feedforward)
from pit_crew import vehicle                                  # noqa: E402

MPC3_PY = os.path.join(ROOT, 'src', 'pit_crew', 'mpc3.py')
SIM_PY = os.path.join(ROOT, 'tools', 'sim_tracker.py')


def node_defaults():
    tree = ast.parse(open(MPC3_PY).read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'MPC3Tracker')
    out = {}
    for n in ast.walk(cls):
        if (isinstance(n, ast.Tuple) and len(n.elts) == 2
                and isinstance(n.elts[0], ast.Constant)
                and isinstance(n.elts[0].value, str)):
            try:
                out[n.elts[0].value] = ast.literal_eval(n.elts[1])
            except ValueError:
                pass
    return out


class TestOffByDefault:

    def test_the_node_ships_with_the_gap_handler_disabled(self):
        d = node_defaults()
        assert d['gap_predict_s'] == 0.0
        assert d['gap_reacq_blend'] == 0.0
        assert d['gap_reacq_gate_m'] == 0.0

    def test_a_disabled_rider_never_rides_and_never_touches_anything(self):
        g = GapRider()
        assert not g.enabled
        assert not g.ride(10.0, 0.35)
        assert g.window(0.35) == 0.0
        assert g.v_cap(10.0, 0.35, 6.0) == 6.0
        g.see_pose(1.0, 0.35)
        g.see_pose(5.0, 0.35)            # a 4 s gap, guard still off
        assert g.filter_gains(1.0, 0.5) == (1.0, 0.5)

    def test_the_ride_law_defaults_to_the_solver(self):
        assert node_defaults()['gap_mode'] == 'solve'

    def test_the_harness_mirrors_the_node_parameters(self):
        src = open(SIM_PY).read()
        for k in ('gap_predict_s', 'gap_mode', 'gap_decel', 'gap_v_min',
                  'gap_reacq_blend', 'gap_reacq_gate_m', 'gap_reacq_n'):
            assert k in src, k


class TestRiding:

    def test_it_rides_a_gap_only_between_stale_and_the_window(self):
        g = GapRider(predict_s=0.6)
        assert not g.ride(0.30, 0.35)            # not stale yet: solve as usual
        assert g.ride(0.40, 0.35)
        assert g.ride(0.95, 0.35)
        assert not g.ride(0.96, 0.35)            # past it: the old coast

    def test_the_prediction_window_covers_the_whole_ride(self):
        g = GapRider(predict_s=0.6)
        assert abs(g.window(0.35) - 0.95) < 1e-12
        assert abs(g.window(0.195) - 0.795) < 1e-12

    def test_the_commanded_speed_ramps_down_while_blind(self):
        g = GapRider(predict_s=0.6, decel=3.0, v_min=1.0)
        assert g.v_cap(0.35, 0.35, 6.0) == 6.0   # nothing before stale
        assert abs(g.v_cap(0.55, 0.35, 6.0) - 5.4) < 1e-9
        assert g.v_cap(0.95, 0.35, 6.0) < 6.0
        assert g.v_cap(5.0, 0.35, 6.0) == 1.0    # never below the floor
        assert g.v_cap(0.55, 0.35, 0.8) == 0.8   # never ABOVE the plan


class TestReacquisition:

    def test_a_long_gap_arms_a_few_blended_poses(self):
        g = GapRider(predict_s=0.6, reacq_blend=0.5, reacq_gate_m=2.0,
                     reacq_n=2)
        g.see_pose(0.000, 0.35)
        g.see_pose(0.104, 0.35)
        assert g.filter_gains(1.0, 0.5) == (1.0, 0.5)   # no gap, untouched
        g.see_pose(0.700, 0.35)
        assert g.gaps == 1
        assert g.filter_gains(1.0, 0.5) == (0.5, 2.0)
        assert g.filter_gains(1.0, 0.5) == (0.5, 2.0)
        assert g.filter_gains(1.0, 0.5) == (1.0, 0.5)   # only `reacq_n` of them

    def test_the_guard_is_inert_without_a_blend(self):
        g = GapRider(predict_s=0.6)
        g.see_pose(0.0, 0.35)
        g.see_pose(1.0, 0.35)
        assert g.gaps == 0
        assert g.filter_gains(1.0, 0.5) == (1.0, 0.5)


class TestTheWindowTheGapHasToCover:
    """Why `gap_predict_s` has to exist at all: the stale window is a fixed
    NUMBER OF FRAMES, so the same three dropped frames trip it at every
    period, and 0.5 s of silence is over it at all of them."""

    def test_the_stale_window_is_about_three_frames_at_every_period(self):
        a = RateAdapter(terms=RATE_TERMS)
        for p in (0.052, 0.058, 0.088, 0.097, 0.104):
            frames = a.values(p)['stale_pose_s'] / p
            assert 3.3 < frames < 3.4, (p, frames)

    def test_half_a_second_of_silence_is_stale_at_every_period(self):
        a = RateAdapter(terms=RATE_TERMS)
        for p in (0.058, 0.097, 0.104):
            assert a.values(p)['stale_pose_s'] < 0.5
        g = GapRider(predict_s=0.6)
        for p in (0.058, 0.097, 0.104):
            assert g.window(a.values(p)['stale_pose_s']) > 0.5 + 2 * p


class TestOpenLoopFeedforward:
    """`gap_mode` 'ff': the plan's own angle, no cross-track term."""

    class Path:
        n = 100
        spacing = 0.1

        def __init__(self, k):
            import numpy as np
            self.kappa = np.full(self.n, k)

    def test_it_reads_the_plan_ahead_by_the_distance_travelled(self):
        p = self.Path(0.0)
        assert gap_feedforward(p, 10, 0.55)[0] == 16         # 0.55 / 0.1
        assert gap_feedforward(p, 95, 1.0)[0] == 5           # wraps

    def test_the_angle_is_the_bicycle_angle_of_the_plan_curvature(self):
        import math
        j, d = gap_feedforward(self.Path(0.8), 0, 0.0)
        assert abs(d - math.atan(vehicle.WHEELBASE * 0.8)) < 1e-12
        assert abs(gap_feedforward(self.Path(-0.8), 0, 0.0)[1] + d) < 1e-12

    def test_it_never_exceeds_the_cap_it_is_given(self):
        _j, d = gap_feedforward(self.Path(3.0), 0, 0.0, cap=0.12)
        assert abs(d - 0.12) < 1e-12
        _j, d = gap_feedforward(self.Path(3.0), 0, 0.0)
        assert abs(d - vehicle.MAX_STEER) < 1e-9

    def test_the_arc_travelled_uses_the_ramp_after_the_pose_goes_stale(self):
        g = GapRider(predict_s=0.6, decel=3.0, v_min=1.0)
        # 0.35 s at 6 m/s, then 0.2 s ramping 6.0 -> 5.4
        want = 6.0 * 0.35 + 0.5 * (6.0 + 5.4) * 0.2
        assert abs(g.arc_ahead(0.55, 0.35, 6.0) - want) < 1e-9
        assert g.arc_ahead(0.2, 0.35, 6.0) == 6.0 * 0.2


def lift(*names):
    """Compile the named MPC3Tracker methods out of mpc3.py (see
    tests/test_mpc3_startup.py -- the node itself needs rclpy)."""
    tree = ast.parse(open(MPC3_PY).read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'MPC3Tracker')
    want = [n for n in cls.body
            if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len(want) == len(names), [n.name for n in want]
    ns = {'math': math}
    exec(compile(ast.Module(body=want, type_ignores=[]), MPC3_PY, 'exec'), ns)
    return {n: ns[n] for n in names}


class TestTheWatchdogPath:
    """The one place the feature changes shipped behaviour: what the 20 Hz
    watchdog does with a stale pose."""

    def stub(self, **over):
        published, stepped = [], []
        node = types.SimpleNamespace(
            pose=(0.0, 0.0, 0.0, 0.0),
            gap=GapRider(),
            rate_ctl=types.SimpleNamespace(
                values={'stale_pose_s': 0.35, 'max_predict_s': 0.35},
                est=types.SimpleNamespace(period=0.104)),
            steer=0.21, _escape_until=None, _stale_warned=False,
            _gap_solve_t=-1e9, _gap_ff=False,
            publish=lambda s, t: published.append((s, t)),
            control_step=lambda: stepped.append(True),
            now=lambda: 0.60,
            get_logger=lambda: types.SimpleNamespace(
                error=lambda *a: None, warn=lambda *a: None),
        )
        for k, v in over.items():
            setattr(node, k, v)
        return node, published, stepped

    def test_with_the_feature_off_a_stale_pose_still_coasts(self):
        fns = lift('watchdog', 'gap_ride')
        node, pub, stepped = self.stub()
        node.gap_ride = types.MethodType(fns['gap_ride'], node)
        fns['watchdog'](node)
        assert pub == [(0.21, 0.0)] and stepped == []

    def test_with_the_feature_on_it_solves_instead_of_coasting(self):
        fns = lift('watchdog', 'gap_ride')
        node, pub, stepped = self.stub(gap=GapRider(predict_s=0.6))
        node.gap_ride = types.MethodType(fns['gap_ride'], node)
        fns['watchdog'](node)
        assert stepped == [True] and pub == []

    def test_it_solves_at_the_sensor_period_not_the_watchdog_rate(self):
        fns = lift('watchdog', 'gap_ride')
        node, pub, stepped = self.stub(gap=GapRider(predict_s=0.6))
        node.gap_ride = types.MethodType(fns['gap_ride'], node)
        for t in (0.40, 0.45, 0.50, 0.55):
            node.now = (lambda tt: (lambda: tt))(t)
            fns['watchdog'](node)
        assert len(stepped) == 2                  # 0.40 and 0.50, not 4 x 20 Hz

    def test_past_the_window_it_falls_back_to_the_coast(self):
        fns = lift('watchdog', 'gap_ride')
        node, pub, stepped = self.stub(gap=GapRider(predict_s=0.6),
                                       now=lambda: 1.20)
        node.gap_ride = types.MethodType(fns['gap_ride'], node)
        fns['watchdog'](node)
        assert pub == [(0.21, 0.0)] and stepped == []

    def test_the_prediction_window_is_untouched_with_the_feature_off(self):
        fns = lift('predict_window', 'gap_v_cmd')
        node, _p, _s = self.stub()
        assert fns['predict_window'](node) == 0.35
        assert fns['gap_v_cmd'](node, 6.0, 0.60) == 6.0
        node.gap = GapRider(predict_s=0.6)
        assert abs(fns['predict_window'](node) - 0.95) < 1e-12
        assert fns['gap_v_cmd'](node, 6.0, 0.60) < 6.0
