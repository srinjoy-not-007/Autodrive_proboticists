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
           'stuck_distance': 0.20,
           # opt-in teleport recovery; 0.0 / 0 is the shipping default
           'v_pose_gate': 0.0, 'v_pose_min_dt': 0.03, 'reindex_after': 0},
        _enc_seen=False,
        v_plan0=0.0,
        _trail=[],
        _v_pose=None,
        _gate_run=0,
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


class TestOptInWiring:
    """`rate_adaptive` and `ilc_enable` are opt-in, but the way the node wires
    them up is not covered by `tests/test_mpc3_core.py` -- the core classes are
    correct in isolation and were still wired wrongly.  These are source-level
    assertions for the same reason the `is_stuck` one above is."""

    def _cls(self):
        tree = ast.parse(open(MPC3_PY).read())
        return next(n for n in tree.body if isinstance(n, ast.ClassDef)
                    and n.name == 'MPC3Tracker')

    def _defaults(self):
        """The (name, default) pairs in `declare_parameters`."""
        out = {}
        for n in ast.walk(self._cls()):
            if isinstance(n, ast.Tuple) and len(n.elts) == 2 \
                    and isinstance(n.elts[0], ast.Constant) \
                    and isinstance(n.elts[0].value, str):
                try:
                    out[n.elts[0].value] = ast.literal_eval(n.elts[1])
                except ValueError:
                    pass
        return out

    def test_both_features_are_off_by_default(self):
        d = self._defaults()
        assert d['rate_adaptive'] is False
        assert d['ilc_enable'] is False

    def test_the_ilc_speed_table_is_off_by_default(self):
        """It costs more lap time than the steering table saves: 12 harness
        laps went 7.86 -> 8.09 s with it and 7.86 -> 7.76 s without."""
        assert self._defaults()['ilc_v_enable'] is False

    def test_ilc_persistence_is_opt_in(self):
        """A non-empty default store makes run N+1 start on run N's table,
        which the offline harness never does -- so the harness would stop
        measuring what races."""
        assert self._defaults()['ilc_store'] == ''

    def test_the_node_caps_the_ilc_target_by_the_line_own_clearance(self):
        """Without `set_nominal_clearance` the target is a flat 0.30 m, and on
        a line whose corridor is under that by construction every bin scores
        tight every lap and the table ratchets to the v_lo floor."""
        src = ast.unparse(self._cls())
        assert 'set_nominal_clearance' in src, \
            'the node builds an ILC but never caps its clearance target'

    def test_the_period_is_estimated_from_accepted_poses_only(self):
        """A rejected pose is not evidence about the sensor rate: feeding them
        in let a `stamp == 0` pose anchor the estimator."""
        on_pose = next(n for n in self._cls().body
                       if isinstance(n, ast.FunctionDef) and n.name == 'on_pose')
        body = on_pose.body
        upd = [i for i, n in enumerate(body)
               if 'rate_ctl.update' in ast.unparse(n)]
        drops = [i for i, n in enumerate(body)
                 if isinstance(n, ast.If) and 'drop_pose' in ast.unparse(n)]
        assert upd and drops, (upd, drops)
        assert min(upd) > max(drops), \
            'rate_ctl.update runs before on_pose has finished rejecting poses'


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


class TestPoseSpeedGate:
    """`v_pose_gate`: the wheel encoder does not reset with a checkpoint teleport.

    Measured over 100 collisions in data/logs (Sept 12): the encoder-derived
    speed reads a median 1.79 m/s (p90 3.50, max 6.01) above the IPS ground
    speed for a median 0.21 s (p90 0.82 s) after the reset, and
    `SpeedEstimator.jumped` -- which needs one sample above 18 m/s -- fired on
    0 of them.  The gate bounds the encoder by the distance the localization
    estimate itself covered, which needs no teleport detection (the published
    estimate does NOT jump at a reset: its frame-to-frame step is 0.23 m
    against 0.38 m in normal driving).
    """

    def test_default_is_off_and_the_encoder_is_believed(self):
        f = lift('measured_speed')
        n = stub_node(v_plan0=6.5)
        roll(n.speed_est, 'L', 0.0, 4.0)
        roll(n.speed_est, 'R', 0.0, 4.0)
        n._v_pose = 0.5                      # the car has really stopped
        v = f['measured_speed'](n)
        assert v is not None and v > 3.0     # unchanged: gate off by default
        assert n._gate_run == 0

    def test_an_over_reading_encoder_is_bounded_by_the_pose(self):
        f = lift('measured_speed')
        n = stub_node(v_plan0=6.5)
        n.P = dict(n.P, v_pose_gate=1.5)
        roll(n.speed_est, 'L', 0.0, 4.0)
        roll(n.speed_est, 'R', 0.0, 4.0)
        assert n.speed_est.speed > 3.0
        n._v_pose = 0.5
        assert f['measured_speed'](n) == pytest.approx(0.5)
        assert n._gate_run == 1

    def test_it_does_not_bind_in_normal_driving(self):
        f = lift('measured_speed')
        n = stub_node(v_plan0=6.5)
        n.P = dict(n.P, v_pose_gate=1.5)
        roll(n.speed_est, 'L', 0.0, 4.0)
        roll(n.speed_est, 'R', 0.0, 4.0)
        n._v_pose = 3.9                      # estimate agrees with the wheels
        v = f['measured_speed'](n)
        assert v == pytest.approx(n.speed_est.speed)
        assert n._gate_run == 0

    def test_the_gate_never_raises_the_speed(self):
        """One-sided by construction: a pose that reads FASTER than the wheels
        must not be used to speed the plan up."""
        f = lift('measured_speed')
        n = stub_node(v_plan0=6.5)
        n.P = dict(n.P, v_pose_gate=1.5)
        roll(n.speed_est, 'L', 0.0, 2.0)
        roll(n.speed_est, 'R', 0.0, 2.0)
        n._v_pose = 9.0
        assert f['measured_speed'](n) == pytest.approx(n.speed_est.speed)

    def test_no_pose_speed_yet_leaves_the_encoder_alone(self):
        f = lift('measured_speed')
        n = stub_node(v_plan0=6.5)
        n.P = dict(n.P, v_pose_gate=1.5)
        roll(n.speed_est, 'L', 0.0, 4.0)
        roll(n.speed_est, 'R', 0.0, 4.0)
        n._v_pose = None
        assert f['measured_speed'](n) > 3.0

    def test_reindex_is_wired_to_the_gate_and_off_by_default(self):
        """`reindex_after` only ever widens the reference search, and only
        while the gate is binding; a checkpoint reset moves the car up to
        4.5 m BACKWARDS along the line while `track_back` looks 0.3 m back."""
        src = open(MPC3_PY).read()
        tree = ast.parse(src)
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == 'MPC3Tracker')
        step = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                    and n.name == '_control_step')
        body = ast.unparse(step)
        assert "reindex_after" in body and "_gate_run" in body
        assert "('reindex_after', 0)" in src        # off unless asked for
        assert "('v_pose_gate', 0.0)" in src
