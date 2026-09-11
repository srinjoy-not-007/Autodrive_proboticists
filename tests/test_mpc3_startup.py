"""Standing start, teleport recovery and throttle guards, without ROS.

``mpc3.MPC3Tracker`` cannot be imported on the host (rclpy), but the three
methods that decide what happens at a standing start and after a collision
teleport are plain functions of `self`.  They are lifted out of the shipped
source with ``ast`` and bound to a stub, so these tests exercise the real code
rather than a copy of it.
"""

import ast
import math
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from pit_crew.throttle2 import ServoThrottle                    # noqa: E402
from pit_crew.vehicle import SpeedEstimator, WHEEL_RADIUS       # noqa: E402

MPC3_PY = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__))), 'src', 'pit_crew', 'mpc3.py')


def lift(*names):
    """Compile the named MPC3Tracker methods out of mpc3.py."""
    tree = ast.parse(open(MPC3_PY).read())
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'MPC3Tracker')
    want = [n for n in cls.body
            if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len(want) == len(names), [n.name for n in want]
    ns = {'math': math}
    exec(compile(ast.Module(body=want, type_ignores=[]), MPC3_PY, 'exec'), ns)
    return {n: ns[n] for n in names}


def stub_node(**over):
    """A minimal stand-in for the node, with the shipped defaults."""
    node = types.SimpleNamespace(
        speed_est=SpeedEstimator(alpha=0.35),
        P={'v_meas_min': 0.2, 'v_start': 0.5, 'stuck_seconds': 1.5,
           'stuck_distance': 0.20},
        _enc_seen=False,
        v_plan0=0.0,
        _trail=[],
    )
    for k, v in over.items():
        setattr(node, k, v)
    return node


def roll(est, wheel, t0, v, n=4, dt=0.09):
    """Feed `n` encoder samples describing a steady speed `v`."""
    pos = 0.0
    est.update(wheel, t0, pos)
    for i in range(1, n + 1):
        pos += v * dt / WHEEL_RADIUS
        est.update(wheel, t0 + i * dt, pos)


class TestStandingStart:
    def test_before_the_encoder_speaks_the_plan_starts_at_walking_pace(self):
        f = lift('measured_speed', 'start_speed')
        n = stub_node(v_plan0=7.0)                # profile speed at the grid
        assert f['measured_speed'](n) is None
        assert f['start_speed'](n) == 0.5
        assert n._enc_seen is False

    def test_once_moving_the_encoder_is_used(self):
        f = lift('measured_speed', 'start_speed')
        n = stub_node(v_plan0=7.0)
        roll(n.speed_est, 'L', 0.0, 4.0)
        roll(n.speed_est, 'R', 0.0, 4.0)
        v = f['measured_speed'](n)
        assert v is not None and 3.0 < v < 4.1
        assert n._enc_seen is True

    def test_a_dropout_after_motion_falls_back_to_the_profile(self):
        f = lift('measured_speed', 'start_speed')
        n = stub_node(v_plan0=6.0)
        roll(n.speed_est, 'L', 0.0, 4.0)
        roll(n.speed_est, 'R', 0.0, 4.0)
        f['measured_speed'](n)
        n.speed_est.speed = 0.0                   # encoder goes quiet
        assert f['measured_speed'](n) is None
        assert f['start_speed'](n) == 6.0         # the profile, by design


class TestTeleportRecovery:
    def test_a_teleport_is_reported_as_no_measurement(self):
        f = lift('measured_speed', 'start_speed')
        n = stub_node(v_plan0=6.5)
        roll(n.speed_est, 'L', 0.0, 4.0)
        roll(n.speed_est, 'R', 0.0, 4.0)
        f['measured_speed'](n)
        n.speed_est.jumped = True                 # collision -> checkpoint
        assert f['measured_speed'](n) is None
        assert n.speed_est.speed == 0.0           # estimator state dropped

    def test_after_a_teleport_the_plan_restarts_from_walking_pace(self):
        """The car is AT REST at a checkpoint after a collision reset.

        `_enc_seen` used to latch forever, so the whole recovery was planned at
        the offline profile speed (6.5 m/s here) with the car stopped: horizon
        steps ~13x too long and `ay_deliver / v**2` saying the car cannot turn
        in at all.  Clearing the latch on the jump sends it back to the
        standing-start branch.
        """
        f = lift('measured_speed', 'start_speed')
        n = stub_node(v_plan0=6.5)
        roll(n.speed_est, 'L', 0.0, 4.0)
        roll(n.speed_est, 'R', 0.0, 4.0)
        f['measured_speed'](n)
        n.speed_est.jumped = True
        assert f['measured_speed'](n) is None
        assert n._enc_seen is False
        assert f['start_speed'](n) == 0.5

    def test_stuck_detection_reads_only_the_pose_trail(self):
        """`is_stuck` has no speed input, by design: a frozen or diverged
        localization pose is indistinguishable from a wedged car here.  The
        encoder gate that makes that safe lives at the call site in
        `_control_step` -- see `test_escape_requires_the_encoder_to_agree`."""
        f = lift('is_stuck')
        n = stub_node(v_plan0=6.0)
        n._trail = [(t, 1.0, 1.0) for t in (0.0, 0.4, 0.8, 1.2, 1.6)]
        assert f['is_stuck'](n) is True
        # ...and the encoder, an independent sensor, says 4 m/s
        roll(n.speed_est, 'L', 0.0, 4.0)
        roll(n.speed_est, 'R', 0.0, 4.0)
        assert n.speed_est.speed > 3.0
        assert f['is_stuck'](n) is True

    def test_a_moving_car_is_not_stuck(self):
        f = lift('is_stuck')
        n = stub_node(v_plan0=6.0)
        n._trail = [(t, t * 2.0, 0.0) for t in (0.0, 0.4, 0.8, 1.2, 1.6)]
        assert f['is_stuck'](n) is False

    def test_a_short_trail_is_never_stuck(self):
        f = lift('is_stuck')
        n = stub_node(v_plan0=6.0)
        n._trail = [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0)]
        assert f['is_stuck'](n) is False


class TestCallbackGuard:
    """A dead node used to be a dead race (the entrypoint now restarts it, but
    the cheapest fix is not to die)."""

    def test_control_step_holds_the_steering_and_coasts_on_any_exception(self):
        f = lift('control_step')
        published = []
        logged = []
        n = types.SimpleNamespace(
            steer=0.21,
            publish=lambda s, t: published.append((s, t)),
            get_logger=lambda: types.SimpleNamespace(
                error=logged.append, warn=logged.append),
            _control_step=lambda: (_ for _ in ()).throw(RuntimeError('boom')),
        )
        f['control_step'](n)                     # must not propagate
        assert published == [(0.21, 0.0)]
        assert logged and 'boom' in logged[0]

    def test_control_step_is_transparent_when_nothing_fails(self):
        f = lift('control_step')
        calls = []
        n = types.SimpleNamespace(
            steer=0.0, publish=lambda s, t: calls.append('publish'),
            get_logger=lambda: types.SimpleNamespace(error=print, warn=print),
            _control_step=lambda: calls.append('step'))
        f['control_step'](n)
        assert calls == ['step']


class TestEscapeIsGatedOnTheEncoder:
    """The escape manoeuvre publishes throttle -0.25 -- a REVERSE setpoint --
    at full lock.  `is_stuck` reads only the pose trail, so the gate that makes
    that safe is at the call site and has to stay there."""

    def test_the_call_site_requires_the_encoder_to_agree(self):
        tree = ast.parse(open(MPC3_PY).read())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == 'MPC3Tracker')
        step = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                    and n.name == '_control_step')
        guards = [ast.unparse(n.test) for n in ast.walk(step)
                  if isinstance(n, ast.If) and 'is_stuck' in ast.unparse(n.test)]
        assert guards, 'is_stuck() is no longer called from _control_step'
        assert all('v_meas' in g for g in guards), guards


class TestServoThrottle:
    """mpc3's only longitudinal actuator.  Never negative -- negative throttle
    is reverse drive in this simulator, which cost 176 collisions once."""

    def _thr(self, **kw):
        base = dict(K=18.0, drag=0.273, tau=0.135, a_auth=4.5, a_accel=5.0,
                    k_p=0.0, head_max=0.5, lead_max=0.0, ay_limit=7.5,
                    v_floor=1.0, u_max=0.60, servo_gain=24.6)
        base.update(kw)
        return ServoThrottle(**base)

    def test_never_negative_and_never_above_u_max(self):
        t = self._thr()
        for target, meas in ((0.0, 6.0), (1.0, 7.0), (30.0, 0.0), (7.0, 0.0)):
            u = t.command(target, meas, 0.0, now=1.0, kappa=0.1)
            assert 0.0 <= u <= 0.60

    def test_braking_is_a_low_forward_setpoint(self):
        t = self._thr()
        fast = t.command(7.0, 7.0, 0.0, now=1.0, kappa=0.0)
        slow = t.command(2.0, 7.0, 0.0, now=1.1, kappa=0.0)
        assert slow < fast
        assert slow > 0.0

    def test_nan_speed_target_commands_zero_throttle(self):
        """What a NaN `v_cmd` out of the solver actually does: the car coasts.

        `max(0.0, nan)` is 0.0 in Python, so the setpoint collapses to the
        floor and the throttle goes to zero -- silent, with no log anywhere.
        """
        t = self._thr()
        u = t.command(float('nan'), 5.0, 0.0, now=1.0, kappa=0.0)
        assert u == 0.0

    def test_setpoint_tracks_the_target_at_speed(self):
        t = self._thr()
        u = t.command(6.0, 6.0, 0.0, now=1.0, kappa=0.0)
        assert u == pytest.approx(6.0 / 24.6, abs=0.01)
