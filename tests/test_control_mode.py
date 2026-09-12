"""`control_mode`: the solve schedule, and the promise that it is opt-in.

mpc3 solves inside the pose callback, so its control rate IS the sensor rate --
and the sensor period is a per-start lottery of the simulator (0.088-0.106 s
measured on one host).  `control_mode='timer'` is the Autoware-style structural
alternative: the same solve on a fixed wall clock, with the pose callback only
storing the measurement.

Three things are tested here, in this order of importance:

1. **off means off.**  The default is the validated shipping controller and is
   bit-identical to it; the harness proves that by running the real thing.
2. the timer really does publish at `control_hz`, whatever the sensor does;
3. a tick with no fresh pose is skipped, so the stale-pose watchdog is still the
   one thing that coasts.

The node's own wiring is asserted at source level (the `ast` pattern
`test_mpc3_startup` established) because `mpc3.MPC3Tracker` needs rclpy, which
is not on the host.
"""

import ast
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

import sim_tracker as ST                                        # noqa: E402
from pit_crew.mpc2_core import RefPath                          # noqa: E402

MPC3_PY = os.path.join(ROOT, 'src', 'pit_crew', 'mpc3.py')

from test_mpc3_core import circle_path                          # noqa: E402


def mpc3_cls():
    tree = ast.parse(open(MPC3_PY).read())
    return next(n for n in tree.body
                if isinstance(n, ast.ClassDef) and n.name == 'MPC3Tracker')


def declared():
    """The `declare_parameters` (name, default) pairs of the node."""
    out = {}
    for n in ast.walk(mpc3_cls()):
        if (isinstance(n, ast.Tuple) and len(n.elts) == 2
                and isinstance(n.elts[0], ast.Constant)
                and isinstance(n.elts[0].value, str)):
            try:
                out[n.elts[0].value] = ast.literal_eval(n.elts[1])
            except ValueError:
                pass
    return out


def method(name):
    return next(n for n in mpc3_cls().body
                if isinstance(n, ast.FunctionDef) and n.name == name)


# --------------------------------------------------------------------------
# 1. off is off
# --------------------------------------------------------------------------

class TestTheDefaultIsTheShippingController:

    def test_the_node_defaults_to_the_validated_schedule(self):
        d = declared()
        assert d['control_mode'] == 'per_pose'
        assert d['control_hz'] == 20.0

    def test_the_node_and_the_harness_default_alike(self, tmp_path):
        """`MPC3Sim` exists so that what the harness measures is what races."""
        sim = ST.MPC3Sim(RefPath(circle_path(tmp_path)))
        d = declared()
        assert sim.P['control_mode'] == d['control_mode']
        assert sim.P['control_hz'] == d['control_hz']
        assert bool(sim.P['timer_sync_dead']) is bool(d['timer_sync_dead'])

    def test_nothing_the_mode_touches_moves_when_it_is_off(self, tmp_path):
        sim = ST.MPC3Sim(RefPath(circle_path(tmp_path)))
        assert sim.rate is None                     # solves once per pose
        assert sim.mpc.cfg['apply_ahead_s'] == pytest.approx(0.090)
        assert sim.act.dead_time == pytest.approx(0.135)
        assert sim.mpc.nd == 3                      # round(0.180 / 0.060)

    def test_the_pose_callback_still_solves_when_the_mode_is_off(self):
        """Source level: `on_pose` must keep calling `control_step` in
        'per_pose', and must NOT call it in 'timer' -- the tick does."""
        src = ast.unparse(method('on_pose'))
        assert 'control_step()' in src
        assert "control_mode == 'per_pose'" in src

    def test_a_default_run_is_the_same_run_as_an_explicit_per_pose_one(
            self, tmp_path):
        line = circle_path(tmp_path, R=4.0, n=400, v=3.0)
        recs = []
        for kw in ({}, dict(control_mode='per_pose')):
            path = RefPath(line)
            recs.append(ST.run(ST.MPC3Sim(path, **kw), path, laps=1,
                               frame=0.104, seed=0)['rec'])
        assert np.array_equal(recs[0], recs[1])


# --------------------------------------------------------------------------
# 2. the timer
# --------------------------------------------------------------------------

class TestTimerMode:

    def sim(self, tmp_path, **kw):
        kw.setdefault('control_mode', 'timer')
        return ST.MPC3Sim(RefPath(circle_path(tmp_path, R=4.0, n=400, v=3.0)),
                          **kw)

    def test_the_schedule_is_the_wall_clock_not_the_sensor(self, tmp_path):
        sim = self.sim(tmp_path, control_hz=20.0)
        assert sim.rate == 20.0

    def test_apply_ahead_becomes_one_tick(self, tmp_path):
        """The command is held for one TICK now, not for one sensor period."""
        sim = self.sim(tmp_path, control_hz=10.0)
        assert sim.mpc.cfg['apply_ahead_s'] == pytest.approx(0.100)
        assert self.sim(tmp_path, control_hz=20.0,
                        timer_apply_ahead_s=0.075).mpc.cfg['apply_ahead_s'] \
            == pytest.approx(0.075)

    def test_the_horizon_structure_is_untouched(self, tmp_path):
        """`nd = round(delay_s / step_dt)` is the actuator and the horizon, not
        the schedule."""
        sim = self.sim(tmp_path, control_hz=20.0)
        assert sim.mpc.nd == 3
        assert sim.mpc.cfg['delay_s'] == pytest.approx(0.180)
        assert sim.mpc.dt == pytest.approx(0.060)

    def test_rate_adaptivity_is_switched_off(self, tmp_path):
        """The two are alternative answers to the same problem: adapting the
        per-period constants means nothing once no constant is per-period."""
        sim = self.sim(tmp_path, control_hz=20.0, rate_adaptive=1)
        assert sim.rate_ctl.enabled is False

    @pytest.mark.parametrize('hz', (10.0, 20.0, 40.0))
    @pytest.mark.parametrize('frame', (0.088, 0.104))
    def test_it_publishes_at_control_hz_whatever_the_sensor_does(
            self, tmp_path, hz, frame):
        path = RefPath(circle_path(tmp_path, R=4.0, n=400, v=3.0))
        sim = ST.MPC3Sim(path, control_mode='timer', control_hz=hz)
        out = ST.run(sim, path, laps=1, frame=frame, seed=0)
        t_end = out['rec'][-1, 0]
        # Ticks before the first pose return the held command without solving.
        assert len(sim.solve_ms) == pytest.approx(hz * t_end, rel=0.05)

    def test_a_bad_mode_is_refused_rather_than_ignored(self, tmp_path):
        with pytest.raises(SystemExit):
            self.sim(tmp_path, control_mode='per-pose')

    def test_the_node_refuses_it_too(self):
        src = ast.unparse(mpc3_cls())
        assert "control_mode must be 'per_pose' or 'timer'" in src


# --------------------------------------------------------------------------
# 3. the stale-pose guard
# --------------------------------------------------------------------------

class TestStaleCoast:

    def sim(self, tmp_path, **kw):
        kw.setdefault('control_mode', 'timer')
        return ST.MPC3Sim(RefPath(circle_path(tmp_path, R=4.0, n=400, v=3.0)),
                          **kw)

    def test_a_tick_before_the_first_pose_holds_the_command(self, tmp_path):
        sim = self.sim(tmp_path)
        assert sim.update(0.0) == (0.0, 0.0)
        assert sim.solve_ms == []

    def test_a_stale_pose_coasts_and_holds_the_steering(self, tmp_path):
        sim = self.sim(tmp_path)
        sim.on_frame(0.0, 0.0, 3.0, 0.0)
        sim.on_pose(0.045, 0.0, 4.0, 0.0, math.pi / 2)
        steer, th = sim.update(0.05)
        assert th > 0.0                      # a live tick drives
        stale = sim.P['stale_pose_s']
        steer2, th2 = sim.update(stale + 0.1)
        assert th2 == 0.0                    # zero throttle is the brake here
        assert steer2 == steer               # ...and the angle is HELD

    def test_the_node_skips_the_tick_and_lets_the_watchdog_coast(self):
        """Two publishers of the same coast would race each other; the node's
        tick returns and `watchdog` (already running at `watchdog_hz`) is the
        one place the held-steer/zero-throttle command comes from."""
        fn = method('on_control_tick')
        body = [n for n in fn.body if not (isinstance(n, ast.Expr)
                                           and isinstance(n.value, ast.Constant))]
        src = ast.unparse(ast.Module(body=body, type_ignores=[]))
        assert 'stale_pose_s' in src and 'return' in src
        assert 'publish' not in src
        assert 'stale_pose_s' in ast.unparse(method('watchdog'))
        assert 'self.publish(self.steer, 0.0)' in ast.unparse(method('watchdog'))


# --------------------------------------------------------------------------
# 4. the quantised command sink
# --------------------------------------------------------------------------

class TestCommandPickup:
    """The bridge consumes ONE command per telemetry tick.  Per-pose publishing
    hides that (one publish per tick, at a fixed phase); a wall-clock schedule
    does not, and an `Actuator` with a constant dead time then models commands
    that were dropped.  Measured cost of getting this wrong, 12 laps x 3 seeds
    at a 0.104 s period: |e| p50 0.136 m and 19 collisions against 0.092 m and
    6 with it modelled."""

    def sim(self, tmp_path, **kw):
        kw.setdefault('control_mode', 'timer')
        return ST.MPC3Sim(RefPath(circle_path(tmp_path, R=4.0, n=400, v=3.0)),
                          **kw)

    def test_the_pickup_grid_comes_from_the_pose_stamps(self, tmp_path):
        sim = self.sim(tmp_path)
        for k in range(12):                  # teach the estimator the period
            sim.rate_ctl.update(k * 0.104)
        assert sim.rate_ctl.est.ready
        # A command issued anywhere inside a tick lands on the NEXT tick.
        assert sim.pickup_time(1.05, 1.04) == pytest.approx(1.144)
        assert sim.pickup_time(1.14, 1.04) == pytest.approx(1.144)
        # ...and never in the past, even exactly on the grid.
        assert sim.pickup_time(1.04, 1.04) == pytest.approx(1.144)

    def test_it_is_the_identity_before_the_period_is_known(self, tmp_path):
        sim = self.sim(tmp_path)
        sim.rate_ctl.est.period = 0.0
        assert sim.pickup_time(1.05, 1.04) == 1.05

    def test_superseded_commands_do_not_act(self, tmp_path):
        """Two publishes inside one tick: the actuator must chase the SECOND,
        because that is the one the bridge latches."""
        sim = self.sim(tmp_path, control_hz=20.0)
        for k in range(12):
            sim.rate_ctl.update(k * 0.104)
        t0 = 11 * 0.104
        sim.act.record(sim.pickup_time(t0 + 0.01, t0), 0.30)
        sim.act.record(sim.pickup_time(t0 + 0.06, t0), -0.30)
        # Well after the pickup + the bridge, the target is the last command.
        assert sim.act._target_at(t0 + 0.5) == pytest.approx(-0.30)

    def test_off_the_command_is_stamped_when_it_was_published(self, tmp_path):
        sim = self.sim(tmp_path, timer_sync_dead=0)
        assert sim.act.dead_time == pytest.approx(0.135)

    def test_the_node_stamps_commands_the_same_way(self):
        src = ast.unparse(method('publish'))
        assert 'pickup_time' in src and 'timer_sync_dead' in src
