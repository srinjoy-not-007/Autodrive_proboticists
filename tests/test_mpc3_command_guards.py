"""What reaches the actuator when something upstream has gone wrong.

Three failure modes, all of which end a race rather than a lap:

* a NON-FINITE steering angle. Python's ``min``/``max`` return their first
  argument when a comparison against NaN is False, so the old
  ``max(-1.0, min(1.0, nan))`` produced **1.0** -- full left lock -- and
  ``np.clip`` then latched the NaN into ``self.steer``, which the watchdog
  republishes at 20 Hz for the rest of the race;
* an EXCEPTION in an rclpy callback, which unwinds ``spin()`` and kills the
  process. ``control_step`` was already guarded; nothing around it was;
* a non-finite THROTTLE, which reaches the simulator unclamped.

``MPC3Tracker`` cannot be imported on the host (rclpy), so the methods are
lifted out of the shipped source with ``ast`` and bound to a stub, exactly as
``tests/test_mpc3_startup.py`` does.
"""

import ast
import math
import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from pit_crew import vehicle                                    # noqa: E402

MPC3_PY = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__))), 'src', 'pit_crew', 'mpc3.py')


class _F32:
    def __init__(self, data=0.0):
        self.data = data


def lift(*names):
    tree = ast.parse(open(MPC3_PY).read())
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'MPC3Tracker')
    want = [n for n in cls.body
            if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len(want) == len(names), [n.name for n in want]
    ns = {'math': math, 'np': np, 'vehicle': vehicle, 'Float32': _F32}
    exec(compile(ast.Module(body=want, type_ignores=[]), MPC3_PY, 'exec'), ns)
    return {n: ns[n] for n in names}


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


class _Log:
    def __init__(self):
        self.errors = []

    def error(self, m):
        self.errors.append(m)

    warn = info = error


def stub(**over):
    node = types.SimpleNamespace(
        steer=0.0, pub_steer=_Pub(), pub_throttle=_Pub(),
        control_mode='per_pose', pose=None,
        P={'timer_sync_dead': True},
        act=types.SimpleNamespace(record=lambda t, c: None),
        _nonfinite_cmd=0, _cb_fail=0,
        now=lambda: 0.0,
        get_logger=lambda: _LOG,
    )
    for k, v in over.items():
        setattr(node, k, v)
    return node


_LOG = _Log()


# --------------------------------------------------------------------------
class TestSteerToCmd:
    """The normalisation every tracker's steering goes through."""

    @pytest.mark.parametrize('a,want', [(0.0, 0.0), (0.5236, 1.0),
                                        (-0.5236, -1.0), (0.2618, 0.5),
                                        (10.0, 1.0), (-10.0, -1.0)])
    def test_finite_angles_are_unchanged(self, a, want):
        assert vehicle.steer_to_cmd(a) == pytest.approx(want)

    @pytest.mark.parametrize('bad', [float('nan'), float('inf'),
                                     float('-inf')])
    def test_a_non_finite_angle_is_straight_and_not_full_lock(self, bad):
        assert vehicle.steer_to_cmd(bad) == 0.0

    def test_the_old_expression_really_did_publish_full_lock(self):
        """Why the guard exists, pinned so nobody 'simplifies' it away."""
        old = max(-1.0, min(1.0, float('nan') / vehicle.MAX_STEER))
        assert old == 1.0


# --------------------------------------------------------------------------
class TestPublishGuard:

    def test_a_finite_pair_takes_the_identical_path(self):
        n = stub()
        pub = lift('publish')['publish']
        pub(n, 0.2, 0.31)
        assert n.steer == pytest.approx(0.2)
        assert n.pub_steer.sent == [pytest.approx(0.2 / vehicle.MAX_STEER)]
        assert n.pub_throttle.sent == [pytest.approx(0.31)]
        assert n._nonfinite_cmd == 0

    def test_the_angle_is_still_clipped_to_full_lock(self):
        n = stub()
        lift('publish')['publish'](n, 9.0, 0.1)
        assert n.steer == pytest.approx(vehicle.MAX_STEER)

    @pytest.mark.parametrize('bad', [float('nan'), float('inf')])
    def test_a_non_finite_steer_holds_the_last_good_angle(self, bad):
        n = stub()
        pub = lift('publish')['publish']
        pub(n, 0.17, 0.30)
        pub(n, bad, 0.30)
        assert n.steer == pytest.approx(0.17)          # held, not latched NaN
        assert n.pub_steer.sent[-1] == pytest.approx(0.17 / vehicle.MAX_STEER)
        assert n.pub_throttle.sent[-1] == 0.0          # and the throttle is cut
        assert n._nonfinite_cmd == 1

    def test_a_non_finite_throttle_alone_is_also_refused(self):
        n = stub()
        pub = lift('publish')['publish']
        pub(n, 0.17, 0.30)
        pub(n, 0.21, float('nan'))
        assert n.pub_throttle.sent[-1] == 0.0
        assert n.steer == pytest.approx(0.17)

    def test_nothing_non_finite_ever_reaches_a_publisher(self):
        n = stub()
        pub = lift('publish')['publish']
        for s, t in ((0.1, 0.2), (float('nan'), 0.2), (0.1, float('inf')),
                     (float('-inf'), float('nan')), (0.3, 0.4)):
            pub(n, s, t)
        assert all(math.isfinite(v) for v in n.pub_steer.sent)
        assert all(math.isfinite(v) for v in n.pub_throttle.sent)


# --------------------------------------------------------------------------
class TestCallbackGuard:
    """One bad message must not end the race."""

    def test_a_raising_callback_is_swallowed_and_counted(self):
        n = stub()
        g = lift('_guarded')['_guarded']

        def boom(_m):
            raise IndexError('index 1081 is out of bounds')

        wrapped = g(n, boom, 'pose')
        assert wrapped(object()) is None
        assert wrapped(object()) is None
        assert n._cb_fail == 2
        assert 'pose callback failed' in _LOG.errors[-1]

    def test_a_working_callback_is_transparent(self):
        n = stub()
        g = lift('_guarded')['_guarded']
        seen = []
        wrapped = g(n, lambda m: seen.append(m) or 'ok', 'imu')
        assert wrapped(7) == 'ok'
        assert seen == [7]
        assert n._cb_fail == 0

    def test_the_guard_publishes_nothing_itself(self):
        """The watchdog timer is the ONE place a hold/coast comes from; a
        second publisher would race it."""
        src = ast.unparse(next(
            f for f in ast.parse(open(MPC3_PY).read()).body
            if isinstance(f, ast.ClassDef) and f.name == 'MPC3Tracker'
            for f in f.body
            if isinstance(f, ast.FunctionDef) and f.name == '_guarded'))
        assert 'self.publish' not in src

    def test_the_brake_mode_is_validated_and_logged(self):
        """`ServoThrottle` treats every string but 'none'/'pulse' as 'zero',
        so an unknown spelling never reports itself -- and losing the flag
        entirely is a DQ (10 collisions in 11-14 display laps without it)."""
        init = ast.unparse(next(
            f for f in ast.parse(open(MPC3_PY).read()).body
            if isinstance(f, ast.ClassDef) and f.name == 'MPC3Tracker'
            for f in f.body
            if isinstance(f, ast.FunctionDef) and f.name == '__init__'))
        assert "'none', 'zero' or 'pulse'" in init, 'brake_mode is unvalidated'
        assert "not in ('none', 'zero', 'pulse')" in init
        ready = init[init.index('mpc3 ready'):]
        assert 'brake_mode' in ready, 'the ready line does not say the brake mode'
        assert 'NO BRAKE' in ready

    def test_every_callback_the_node_registers_is_wrapped(self):
        """Source level: a new subscription added without the guard is the
        regression this is here to catch."""
        src = open(MPC3_PY).read()
        tree = ast.parse(src)
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == 'MPC3Tracker')
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                    and n.name == '__init__')
        bad = []
        for node in ast.walk(init):
            if not isinstance(node, ast.Call):
                continue
            fn = ast.unparse(node.func)
            if fn == 'self.create_subscription' and len(node.args) >= 3:
                cb = ast.unparse(node.args[2])
            elif fn == 'self.create_timer' and len(node.args) >= 2:
                cb = ast.unparse(node.args[1])
            else:
                continue
            if not cb.startswith('self._guarded('):
                bad.append(cb)
        assert not bad, f'unguarded rclpy callbacks: {bad}'
