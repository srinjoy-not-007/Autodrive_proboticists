"""Pure-python regression tests for the shipping mpc3 tracker's core.

No ROS: ``mpc3_core`` is rclpy-free on purpose (see its docstring), so the
solver that races can be exercised on the host with

    .venv/bin/python -m pytest tests

These tests are about the failure modes an UNSEEN finals track can produce --
a standing start, a collision teleport, the line's wraparound, a corridor
narrower than the barrier margin, a degenerate or reversed line, non-finite
input -- not about lap time.  Where a test documents a gap rather than a
guarantee it says so and is marked ``xfail(strict=True)``, so closing the gap
makes the suite fail until the test is updated deliberately.
"""

import ast
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from pit_crew import vehicle                                    # noqa: E402
from pit_crew.mpc3_core import (ILC, MPC3, RATE_ALL_TERMS,        # noqa: E402
                                RATE_NOMINAL, RATE_TERMS,
                                Actuator, PoseFilter, RateAdapter,
                                RateControl, RateEstimator, RefPath,
                                corridor_clearance, wrap)


# --------------------------------------------------------------------------
# synthetic reference lines
# --------------------------------------------------------------------------

def write_line(tmp_path, xy, v=4.0, name='line.csv'):
    """Write a raceline CSV in the pipeline's column order.

    s, x, y, psi, kappa, vx, ax -- exactly what ``make_track_lines.py`` emits
    and what ``RefPath`` indexes by column number.
    """
    xy = np.asarray(xy, dtype=float)
    d = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(d)])
    fwd = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    psi = np.arctan2(fwd[:, 1], fwd[:, 0])
    dpsi = (np.roll(psi, -1) - np.roll(psi, 1) + np.pi) % (2 * np.pi) - np.pi
    ds = np.roll(s, -1) - np.roll(s, 1)
    kappa = dpsi / np.where(np.abs(ds) > 1e-9, ds, 1e-9)
    vv = np.full(len(xy), float(v)) if np.isscalar(v) else np.asarray(v, float)
    rows = np.column_stack([s, xy[:, 0], xy[:, 1], psi, kappa, vv,
                            np.zeros(len(xy))])
    p = tmp_path / name
    np.savetxt(p, rows, delimiter=',')
    return str(p)


def circle_path(tmp_path, R=3.0, n=400, v=4.0, name='circle.csv'):
    t = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    return write_line(tmp_path, np.column_stack([R * np.cos(t), R * np.sin(t)]),
                      v=v, name=name)


def straight_path(tmp_path, length=60.0, n=600, v=4.0, name='straight.csv'):
    """A long straight closed by a far return leg, so wraparound stays sane."""
    x = np.linspace(0.0, length, n)
    return write_line(tmp_path, np.column_stack([x, np.zeros(n)]), v=v,
                      name=name)


def drive_circle(tmp_path, R=6.0, v=3.0, steps=120, feedback=True,
                 record_seam=False, **cfg):
    """Closed loop: the real solver against a kinematic plant + Actuator.

    The plant is the measured one the solver models (yaw gain, 0.135 s dead
    time, 3.2 rad/s slew) at a fixed speed, so this measures the TRACKER, not
    the speed plan.  ``feedback=False`` never calls ``Actuator.snap``, which is
    what happens if ``/autodrive/roboracer_1/steering`` does not arrive.
    """
    p = RefPath(circle_path(tmp_path, R=R, n=400, v=v,
                            name=f'drive_{R}_{v}_{feedback}.csv'))
    m = make_mpc(p, **cfg)
    act = Actuator(dead_time=0.135)
    dt = 0.090                                   # the ~11 Hz sensor period
    t, x, y, yaw, delta = 0.0, R, 0.0, math.pi / 2, 0.0
    idx, errs, seam = None, [], []
    for step in range(steps):
        prev = idx
        idx = p.nearest(x, y, idx, yaw=yaw)
        pyaw = float(p.yaw[idx])
        dx, dy = x - p.xy[idx, 0], y - p.xy[idx, 1]
        e = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
        if feedback:
            act.snap(t, delta)
        d0 = act.delta_at(t)
        committed = [act.delta_at(t + (k + 1) * m.dt) for k in range(m.nd)]
        cmd, _ = m.solve(e, wrap(yaw - pyaw), d0, idx, v, committed=committed)
        act.record(t, cmd)
        n = 9
        h = dt / n
        for k in range(n):
            tgt = act._target_at(t + (k + 0.5) * h)
            delta += max(-vehicle.STEER_RATE * h,
                         min(vehicle.STEER_RATE * h, tgt - delta))
            yr = m.cfg['yaw_gain'] * v * math.tan(delta) / vehicle.WHEELBASE
            x += v * math.cos(yaw + 0.5 * yr * h) * h
            y += v * math.sin(yaw + 0.5 * yr * h) * h
            yaw = wrap(yaw + yr * h)
        t += dt
        if step > 20:
            errs.append(abs(e))
            if record_seam and prev is not None and idx < prev:
                seam.append(abs(e))
    out = {'p50': float(np.percentile(errs, 50)),
           'p95': float(np.percentile(errs, 95)),
           'max': float(np.max(errs))}
    if record_seam:
        out['seam_max'] = float(max(seam)) if seam else 0.0
        out['seam_crossings'] = len(seam)
    return out


def make_mpc(path, **cfg):
    base = dict(horizon=20, step_dt=0.060, iters=6, outer=2)
    base.update(cfg)
    return MPC3(path, **base)


# --------------------------------------------------------------------------
# 1. the reference path itself
# --------------------------------------------------------------------------

class TestRefPath:
    def test_circle_curvature_and_spacing(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=3.0, n=400))
        assert p.n == 400
        assert p.spacing == pytest.approx(2 * math.pi * 3.0 / 400, rel=1e-3)
        assert np.abs(p.kappa).mean() == pytest.approx(1 / 3.0, rel=0.05)
        assert not p.has_corridor
        assert np.isinf(p.w_left).all()

    def test_rejects_short_nonfinite_and_negative_speed(self, tmp_path):
        two = tmp_path / 'two.csv'
        two.write_text('0,0,0,0,0,1,0\n1,1,0,0,0,1,0\n')
        with pytest.raises(ValueError):
            RefPath(str(two))
        bad = tmp_path / 'nan.csv'
        bad.write_text('0,0,0,0,0,1,0\n1,1,0,0,0,nan,0\n2,2,0,0,0,1,0\n')
        with pytest.raises(ValueError):
            RefPath(str(bad))
        neg = tmp_path / 'neg.csv'
        neg.write_text('0,0,0,0,0,1,0\n1,1,0,0,0,-1,0\n2,2,0,0,0,1,0\n')
        with pytest.raises(ValueError):
            RefPath(str(neg))

    def test_zero_speed_rows_are_accepted_by_refpath(self, tmp_path):
        """RefPath's gate is `< 0`, so a zero-speed row loads.

        `preflight.validate` rejects `<= 0`, but mpc3 loads the CSV itself and
        never calls preflight, so a line with a zero in column 5 reaching the
        node stalls the car there rather than being refused at startup.
        """
        z = tmp_path / 'zero.csv'
        z.write_text('0,0,0,0,0,1,0\n1,1,0,0,0,0,0\n2,2,0,0,0,1,0\n')
        p = RefPath(str(z))
        assert p.v_prof.min() == 0.0

    def test_nearest_wraps_and_is_heading_gated(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=3.0, n=400))
        # a point just past the seam, approached from index n-2
        j = p.nearest(3.0, 0.02, last=p.n - 2, yaw=math.pi / 2)
        assert j in (0, 1, 2, p.n - 1, p.n - 2)
        # a jump the window cannot cover falls back to the global search
        far = p.nearest(-3.0, 0.0, last=0, yaw=-math.pi / 2)
        assert abs(far - p.n // 2) < 5


# --------------------------------------------------------------------------
# 2. solver on the two shapes whose answer is known in closed form
# --------------------------------------------------------------------------

class TestSolverGeometry:
    def test_straight_offset_steers_back_towards_the_line(self, tmp_path):
        p = RefPath(straight_path(tmp_path))
        m = make_mpc(p)
        # e_lat > 0 means the car is LEFT of the path, so the command must be
        # negative (positive steering_command is left).
        cmd, info = m.solve(0.30, 0.0, 0.0, 10, 4.0)
        assert math.isfinite(cmd) and cmd < -1e-3
        cmd2, _ = m.solve(-0.30, 0.0, 0.0, 10, 4.0)
        assert cmd2 > 1e-3
        on, _ = make_mpc(p).solve(0.0, 0.0, 0.0, 10, 4.0)
        assert abs(on) < 1e-2

    def test_circle_command_matches_the_steady_state_angle(self, tmp_path):
        """On a constant-radius circle, on the line, the answer is analytic.

        `delta0` is held at the steady-state angle, which is what the actuator
        feedback reports once the car is settled.
        """
        R = 3.0
        p = RefPath(circle_path(tmp_path, R=R, n=400))
        m = make_mpc(p)
        # psi' = g tan(d)/L - kappa = 0  ->  d = atan(kappa L / g)
        want = math.atan(vehicle.WHEELBASE / (R * m.cfg['yaw_gain']))
        cmd = 0.0
        for _ in range(5):
            cmd, _ = m.solve(0.0, 0.0, want, 50, 4.0,
                             committed=[want] * m.nd)
        assert cmd == pytest.approx(want, abs=0.02)

    def test_the_loop_survives_losing_the_steering_feedback(self, tmp_path):
        """Closed loop on a circle, with and without `/steering`.

        `committed` comes from `Actuator`.  When feedback exists it is anchored
        on the measured angle; when none has ever arrived it now replays the
        command history through the same dead time and slew.  It used to return
        the last command as though it were already applied, and that assumption
        cost the loop everything: lateral p95 0.0041 m -> 0.171 m here, and
        0.019 -> 0.965 m on R 3 m at 4 m/s.  This test is the regression guard
        for that fallback.
        """
        good = drive_circle(tmp_path, R=6.0, v=3.0, feedback=True)
        blind = drive_circle(tmp_path, R=6.0, v=3.0, feedback=False)
        assert good['p95'] < 0.03, good
        assert blind['p95'] < 0.03, blind
        assert blind['p95'] < 3 * good['p95'] + 0.005, (good, blind)

    def test_closed_loop_error_decays_on_a_straight(self, tmp_path):
        """Drive the real solver against a kinematic plant; the error must fall."""
        p = RefPath(straight_path(tmp_path, length=120.0, n=1200))
        m = make_mpc(p)
        x, y, yaw, delta, v = 0.0, 0.40, 0.0, 0.0, 4.0
        dt = 0.090
        idx = None
        peak = 0.0
        for _ in range(60):
            idx = p.nearest(x, y, idx, yaw=yaw)
            e_lat = y - p.xy[idx, 1]
            cmd, _ = m.solve(e_lat, wrap(yaw - p.yaw[idx]), delta, idx, v)
            # actuator: slew towards the command over one frame
            delta += max(-vehicle.STEER_RATE * dt,
                         min(vehicle.STEER_RATE * dt, cmd - delta))
            yr = m.cfg['yaw_gain'] * v * math.tan(delta) / vehicle.WHEELBASE
            x += v * math.cos(yaw + 0.5 * yr * dt) * dt
            y += v * math.sin(yaw + 0.5 * yr * dt) * dt
            yaw = wrap(yaw + yr * dt)
            peak = max(peak, abs(y))
        assert abs(y) < 0.10, f'did not converge, |e|={abs(y):.3f}'
        assert peak < 1.0, 'overshot wildly before converging'


# --------------------------------------------------------------------------
# 3. wraparound, teleports and stale warm starts
# --------------------------------------------------------------------------

class TestWraparoundAndTeleport:
    def test_horizon_indices_stay_in_range_across_the_seam(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=3.0, n=400, v=5.0))
        m = make_mpc(p)
        for i0 in list(range(390, 400)) + list(range(0, 10)):
            cmd, info = m.solve(0.0, 0.0, 0.11, i0, 5.0, committed=[0.11] * m.nd)
            assert math.isfinite(cmd)
            assert info['idx'].min() >= 0 and info['idx'].max() < p.n
            assert np.isfinite(info['e']).all() and np.isfinite(info['v']).all()

    def test_driving_through_the_seam_is_not_a_disturbance(self, tmp_path):
        """A closed lap of a circle: index 0 is not a physical event, so the
        lateral error must not spike there."""
        r = drive_circle(tmp_path, R=6.0, v=4.0, steps=180, record_seam=True)
        assert r['seam_crossings'] >= 1, 'the test never reached the seam'
        assert r['p95'] < 0.05, r
        assert r['seam_max'] < 3 * r['p95'] + 0.02, r

    def test_teleport_to_a_far_index_stays_finite_and_bounded(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=3.0, n=400, v=6.0))
        m = make_mpc(p)
        for _ in range(5):                       # build up a warm start at speed
            m.solve(0.0, 0.0, 0.2, 10, 6.0)
        # collision reset: new pose half a lap away, car at rest, error large
        cap = 0.4
        cmd, info = m.solve(0.45, 0.9, 0.2, 200, 0.5, cap=cap)
        assert math.isfinite(cmd) and abs(cmd) <= cap + 1e-9
        assert np.isfinite(info['v']).all()
        assert info['v'][0] == pytest.approx(0.5, abs=1e-9)
        assert math.isfinite(info['v_cmd']) and info['v_cmd'] >= 0.0

    def test_speed_plan_starts_from_the_car_not_the_profile(self, tmp_path):
        """The standing-start guarantee: v[0] is the speed passed in."""
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=7.0))
        m = make_mpc(p)
        _, info = m.solve(0.0, 0.0, 0.0, 0, 0.5)
        assert info['v'][0] == pytest.approx(0.5)
        # and it accelerates at no more than a_accel over each step
        v, ds = info['v'], info['ds']
        for k in range(len(ds)):
            assert v[k + 1] ** 2 <= v[k] ** 2 + 2 * m.cfg['a_accel'] * ds[k] + 1e-6

    def test_ceiling_never_exceeds_the_offline_profile(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=5.0))
        m = make_mpc(p)
        _, info = m.solve(0.0, 0.0, 0.0, 0, 5.0)
        assert info['v_ceiling'].max() <= 5.0 + 1e-9
        assert info['v_cmd'] <= 5.0 + 1e-9

    def test_large_lateral_error_cuts_the_commanded_speed(self, tmp_path):
        p = RefPath(straight_path(tmp_path, v=6.0))
        base = make_mpc(p)
        _, clean = base.solve(0.0, 0.0, 0.0, 10, 6.0)
        wide = make_mpc(p)
        for _ in range(3):                       # let e_pred fill with the error
            _, off = wide.solve(0.9, 0.0, 0.0, 10, 6.0)
        assert off['v_cmd'] < clean['v_cmd'], 'speed cut did not engage'
        assert off['v_cmd'] >= base.cfg['v_mod_min'] * 6.0 - 1e-6


# --------------------------------------------------------------------------
# 4. corridor barrier
# --------------------------------------------------------------------------

class TestBarrier:
    def _corridor(self, p, half):
        p.w_left = np.full(p.n, half)
        p.w_right = np.full(p.n, half)
        p.has_corridor = True

    def test_barrier_pushes_back_from_the_wall(self, tmp_path):
        p = RefPath(straight_path(tmp_path))
        self._corridor(p, 0.80)
        m = make_mpc(p)
        _, info = m.solve(0.55, 0.0, 0.0, 10, 4.0)
        # hi = 0.80 - (0.135 + 0.24) = 0.425; the car is outside it and the
        # horizon must come back inside.
        assert info['hi'][0] == pytest.approx(0.425, abs=1e-6)
        assert info['e'][-1] < 0.55

    def test_corridor_narrower_than_the_margin_is_not_a_crash(self, tmp_path):
        """hw = 0.135 + barrier_margin = 0.375 m.  A 0.30 m half-width corridor
        makes lo > hi, i.e. an infeasible box.  An unseen track can easily have
        a pinch point that tight, so the solver must degrade to the corridor
        middle rather than produce nonsense."""
        p = RefPath(straight_path(tmp_path))
        self._corridor(p, 0.30)
        m = make_mpc(p)
        cmd, info = m.solve(0.20, 0.0, 0.0, 10, 4.0)
        assert math.isfinite(cmd)
        assert info['lo'][0] > info['hi'][0]     # the box really is empty
        assert np.isfinite(info['e']).all()
        assert abs(info['e'][-1]) < abs(0.20) + 0.05


# --------------------------------------------------------------------------
# 5. degenerate lines the pipeline is supposed to reject
# --------------------------------------------------------------------------

class TestDegenerateLines:
    def test_three_point_line(self, tmp_path):
        tiny = tmp_path / 'tiny.csv'
        tiny.write_text('0,0,0,0,0,2,0\n1,1,0,0,0,2,0\n2,0.5,0.8,0,0,2,0\n')
        p = RefPath(str(tiny))
        cmd, info = make_mpc(p).solve(0.05, 0.0, 0.0, 0, 2.0)
        assert math.isfinite(cmd)
        assert info['idx'].max() < p.n

    def test_duplicated_points_do_not_divide_by_zero(self, tmp_path):
        xy = np.repeat(np.column_stack([np.linspace(0, 10, 20),
                                        np.zeros(20)]), 3, axis=0)
        p = RefPath(write_line(tmp_path, xy, name='dup.csv'))
        assert p.spacing >= 0.0
        cmd, _ = make_mpc(p).solve(0.1, 0.0, 0.0, 5, 3.0)
        assert math.isfinite(cmd)

    def test_direction_reversal_line_stays_bounded(self, tmp_path):
        """A line that doubles back is rejected by make_track_lines'
        `drivability`, but PIT_CREW_LINE can still name one.  It must not make
        the solver diverge or emit anything outside the steering cap."""
        fwd = np.column_stack([np.linspace(0, 10, 100), np.zeros(100)])
        back = np.column_stack([np.linspace(10, 0, 100), np.full(100, 0.05)])
        p = RefPath(write_line(tmp_path, np.vstack([fwd, back]), name='rev.csv'))
        m = make_mpc(p)
        cmd = 0.0
        for i0 in (10, 50, 95, 99, 100, 105, 150):
            cmd, info = m.solve(0.1, 0.0, cmd, i0, 4.0)
            assert math.isfinite(cmd)
            assert abs(cmd) <= vehicle.MAX_STEER + 1e-9
            assert np.isfinite(info['e']).all()


# --------------------------------------------------------------------------
# 6. non-finite input
# --------------------------------------------------------------------------

class TestNonFinite:
    def test_nonfinite_cap_and_speed_do_not_crash(self, tmp_path):
        p = RefPath(straight_path(tmp_path))
        m = make_mpc(p)
        cmd, _ = m.solve(0.1, 0.0, 0.0, 10, 4.0, cap=float('inf'))
        assert math.isfinite(cmd) and abs(cmd) <= vehicle.MAX_STEER + 1e-9

    def test_out_of_range_delta0_is_clipped(self, tmp_path):
        p = RefPath(straight_path(tmp_path))
        m = make_mpc(p)
        cmd, info = m.solve(0.0, 0.0, 5.0, 10, 4.0)
        assert abs(info['delta'][0]) <= vehicle.MAX_STEER + 1e-9
        assert math.isfinite(cmd)

    @pytest.mark.parametrize('state', [
        (float('nan'), 0.0, 0.0, 4.0),
        (0.0, float('nan'), 0.0, 4.0),
        (0.0, 0.0, float('nan'), 4.0),       # used to publish FULL RIGHT LOCK
        (0.0, 0.0, 0.0, float('nan')),
        (float('inf'), 0.0, 0.0, 4.0),
    ])
    def test_non_finite_state_is_rejected_not_absorbed(self, tmp_path, state):
        """`_rollout` clips with min(cap, max(-cap, raw)), and Python's max
        returns -cap for a NaN, so a NaN steering angle used to come out as
        -MAX_STEER: full right lock, silently.  The solver now refuses."""
        p = RefPath(straight_path(tmp_path))
        e, psi, d, v = state
        with pytest.raises(ValueError):
            make_mpc(p).solve(e, psi, d, 10, v)

    def test_non_finite_committed_steering_is_rejected(self, tmp_path):
        p = RefPath(straight_path(tmp_path))
        m = make_mpc(p)
        with pytest.raises(ValueError):
            m.solve(0.1, 0.0, 0.0, 10, 4.0, committed=[float('nan')] * m.nd)

    def test_a_rejected_state_leaves_the_previous_plan_untouched(self, tmp_path):
        """The node holds its last good command, so the warm start it will
        solve from next frame must not have been half-updated."""
        p = RefPath(straight_path(tmp_path))
        m = make_mpc(p)
        cmd, _ = m.solve(0.20, 0.0, 0.0, 10, 4.0)
        u_before = m.u.copy()
        with pytest.raises(ValueError):
            m.solve(float('nan'), 0.0, 0.0, 10, 4.0)
        assert np.array_equal(m.u, u_before)
        again, _ = m.solve(0.20, 0.0, 0.0, 10, 4.0)
        assert math.isfinite(again)


# --------------------------------------------------------------------------
# 7. actuator / pose filter: what a teleport leaves behind
# --------------------------------------------------------------------------

class TestActuatorAndPose:
    def test_feedback_overrides_the_command_history(self):
        a = Actuator(dead_time=0.135)
        a.record(0.0, 0.30)
        a.snap(0.10, 0.05)
        assert a.delta_at(0.10) == pytest.approx(0.05)
        # after the feedback ends the slew model takes over, towards 0.30
        assert 0.05 < a.delta_at(0.25) <= 0.30

    def test_out_of_order_feedback_is_ignored(self):
        a = Actuator()
        a.snap(1.0, 0.2)
        a.snap(0.9, -0.2)
        assert a.delta_at(1.0) == pytest.approx(0.2)
        a.snap(float('nan'), 0.1)
        assert a.delta_at(1.0) == pytest.approx(0.2)

    def test_without_feedback_the_command_history_is_modelled(self):
        """If `/steering` never arrives the actuator is modelled, not assumed.

        It used to return the last command for every time, i.e. no dead time
        and no slew.  Now the same 0.135 s dead time and 3.2 rad/s slew apply.
        """
        a = Actuator(dead_time=0.135)
        a.record(0.0, 0.40)
        assert a.delta_at(0.0) == pytest.approx(0.0)      # still in the pipe
        assert a.delta_at(0.135) == pytest.approx(0.0)
        mid = a.delta_at(0.20)
        assert 0.0 < mid < 0.40                            # slewing
        assert a.delta_at(1.0) == pytest.approx(0.40)      # arrived
        assert a.delta_at(1.0) <= 0.40 + 1e-9              # never overshoots

    def test_the_modelled_fallback_cannot_loop_forever(self):
        """`hist` starts at -1e9; the fallback must clamp its start time."""
        a = Actuator(dead_time=0.135)
        a.record(0.0, 0.30)
        assert a.delta_at(1e6) == pytest.approx(0.30)

    def test_pose_filter_takes_a_teleport_whole(self):
        a = Actuator()
        f = PoseFilter(a, blend=0.55, gate_m=0.5)
        f.update(0.0, 0.0, 0.0, 0.0, 4.0)
        x, y, yaw, _ = f.update(0.1, 5.0, 5.0, 1.0, 4.0)
        assert (x, y) == pytest.approx((5.0, 5.0))        # gate -> take it whole
        # a small innovation is still blended
        g = PoseFilter(Actuator(), blend=0.55, gate_m=0.5)
        g.update(0.0, 0.0, 0.0, 0.0, 4.0)
        x2, y2, _, _ = g.update(0.1, 0.05, 0.05, 0.0, 4.0)
        assert 0.0 < y2 < 0.05

    def test_blend_one_always_takes_the_pose_whole(self):
        f = PoseFilter(Actuator(), blend=1.0, gate_m=0.5)
        f.update(0.0, 0.0, 0.0, 0.0, 4.0)
        x, y, _, _ = f.update(0.1, 0.4, 0.02, 0.0, 4.0)
        assert (x, y) == pytest.approx((0.4, 0.02))


# --------------------------------------------------------------------------
# 8. derive_map_prefix -- read out of mpc3.py without importing rclpy
# --------------------------------------------------------------------------

def _load_derive_map_prefix():
    src = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(
        __file__))), 'src', 'pit_crew', 'mpc3.py')
    tree = ast.parse(open(src).read())
    fn = next(n for n in tree.body
              if isinstance(n, ast.FunctionDef) and n.name == 'derive_map_prefix')
    ns = {'os': os}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), src, 'exec'), ns)
    return ns['derive_map_prefix']


class TestDeriveMapPrefix:
    def _tree(self, tmp_path, maps):
        (tmp_path / 'maps').mkdir()
        (tmp_path / 'racelines').mkdir()
        for m in maps:
            for ext in ('.yaml', '.pgm'):
                (tmp_path / 'maps' / (m + ext)).write_text('x')
        return tmp_path

    def test_drill_line_names_resolve_to_the_mesh_map(self, tmp_path):
        derive = _load_derive_map_prefix()
        root = self._tree(tmp_path, ['finals_mesh'])
        for name in ('finals_mesh_mc0.60_ay6.0.csv', 'finals_mesh_race.csv',
                     'finals_mesh_safe.csv', 'finals_mesh_centerline.csv',
                     'finals_mesh_raceline_anchored_w0.48.csv'):
            got = derive(str(root / 'racelines' / name))
            assert got == str(root / 'maps' / 'finals_mesh'), name

    def test_longest_matching_prefix_wins(self, tmp_path):
        derive = _load_derive_map_prefix()
        root = self._tree(tmp_path, ['finals', 'finals_mesh'])
        assert derive(str(root / 'racelines' / 'finals_mesh_race.csv')) == \
            str(root / 'maps' / 'finals_mesh')

    def test_missing_map_disables_the_barrier_silently(self, tmp_path):
        derive = _load_derive_map_prefix()
        root = self._tree(tmp_path, [])
        assert derive(str(root / 'racelines' / 'finals_mesh_race.csv')) == ''

    def test_half_a_map_is_not_a_map(self, tmp_path):
        derive = _load_derive_map_prefix()
        root = self._tree(tmp_path, [])
        (root / 'maps' / 'finals_mesh.yaml').write_text('x')      # no .pgm
        assert derive(str(root / 'racelines' / 'finals_mesh_race.csv')) == ''

    def test_line_outside_the_data_layout_finds_nothing(self, tmp_path):
        """PIT_CREW_LINE pointing anywhere but <root>/racelines/ silently
        disables the corridor barrier -- there is no error, only a warn."""
        derive = _load_derive_map_prefix()
        root = self._tree(tmp_path, ['finals_mesh'])
        loose = root / 'finals_mesh_race.csv'
        loose.write_text('')
        assert derive(str(loose)) == ''


# --------------------------------------------------------------------------
# 9. rate adaptivity
# --------------------------------------------------------------------------
#
# The point of every test here is the same one: the shipping configuration has
# `rate_adaptive` false and must be bit-identical to what raced, and switching
# it on at the period the tracker was validated at must ALSO be identical.  A
# feature that can only be trusted when it is off is worth nothing, and one
# that silently changes the shipping tune is worse than nothing.

class TestRateEstimator:

    def test_median_not_mean(self):
        """One dropped pose doubles an interval; the estimate must not move."""
        e = RateEstimator(nominal=0.104, window=20)
        t = 0.0
        for k in range(30):
            t += 0.176 if k == 15 else 0.088     # one dropped frame
            e.update(t)
        assert e.ready
        assert abs(e.period - 0.088) < 1e-6

    def test_ignores_implausible_intervals(self):
        e = RateEstimator(nominal=0.104)
        t = 0.0
        for k in range(20):
            t += 0.097
            e.update(t)
            e.update(t + 1e-4)                   # a duplicate: not evidence
        assert abs(e.period - 0.097) < 1e-6

    def test_not_ready_before_min_samples(self):
        e = RateEstimator(nominal=0.104, min_samples=8)
        t = 0.0
        for _ in range(4):
            t += 0.088
            e.update(t)
        assert not e.ready
        assert e.period == 0.104                 # falls back to the nominal

    def test_nonfinite_stamp_is_inert(self):
        e = RateEstimator(nominal=0.104)
        assert e.update(float('nan')) == 0.104


class TestRateAdapter:

    def test_identity_at_nominal(self):
        a = RateAdapter()
        v = a.values(RATE_NOMINAL)
        assert v['cmd_dead_time'] == pytest.approx(0.135)
        assert v['delay_s'] == pytest.approx(0.180)
        assert v['apply_ahead_s'] == pytest.approx(0.090)
        assert v['thr_lead_s'] == pytest.approx(0.225)
        assert v['enc_alpha'] == pytest.approx(0.35)

    def test_monotone_in_the_period(self):
        a = RateAdapter(terms=RATE_ALL_TERMS)    # incl. `delay`, non-default
        lo, hi = a.values(0.088), a.values(0.104)
        for k in ('cmd_dead_time', 'delay_s', 'apply_ahead_s', 'thr_lead_s',
                  'max_predict_s', 'stale_pose_s', 'enc_alpha'):
            assert lo[k] < hi[k], k

    def test_dead_time_is_half_a_frame_plus_the_bridge(self):
        a = RateAdapter(terms=RATE_ALL_TERMS)    # incl. `delay`, non-default
        assert a.values(0.088)['cmd_dead_time'] == pytest.approx(
            a.bridge + 0.044)

    def test_encoder_alpha_holds_its_lag_in_seconds(self):
        """alpha is per SAMPLE; (1-alpha) per unit time is what must be fixed."""
        a = RateAdapter()
        v = a.values(0.052)                      # exactly half the period
        assert (1.0 - v['enc_alpha']) ** 2 == pytest.approx(1.0 - 0.35)

    def test_terms_select(self):
        a = RateAdapter(terms=('apply',))
        v = a.values(0.088)
        assert v['apply_ahead_s'] != 0.090
        assert v['delay_s'] == 0.180 and v['thr_lead_s'] == 0.225

    def test_delay_is_not_a_default_term(self):
        """It double-counts the half-frame `apply` already scales, and nd
        quantisation hides that in the whole realistic band."""
        assert 'delay' not in RATE_TERMS
        assert 'delay' in RATE_ALL_TERMS
        v = RateAdapter().values(0.088)
        assert v['cmd_dead_time'] == pytest.approx(0.135)
        assert v['delay_s'] == pytest.approx(0.180)

    def test_predict_and_stale_never_grow(self):
        """Both are SAFETY gates: a slower sensor must not buy a longer
        open-loop dead-reckon or a later pose-loss coast.  Unclamped, the
        slowest period the estimator admits (0.39 s) gave 1.31 s of each."""
        a = RateAdapter()
        fast = a.values(0.088)
        assert fast['max_predict_s'] < 0.35 and fast['stale_pose_s'] < 0.35
        for p in (0.104, 0.150, 0.250, 0.390):
            v = a.values(p)
            assert v['max_predict_s'] <= 0.35 + 1e-12, p
            assert v['stale_pose_s'] <= 0.35 + 1e-12, p
        assert a.values(0.390)['max_predict_s'] == pytest.approx(0.35)

    def test_blend_one_stays_one(self):
        """mpc3 ships pose_blend 1.0 -- 'take every sample whole' has no lag
        to rescale, and rescaling it would quietly introduce a filter."""
        assert RateAdapter(pose_blend=1.0).values(0.088)['pose_blend'] == 1.0


class TestSetRate:

    def test_nd_follows_delay(self, tmp_path):
        m = make_mpc(RefPath(circle_path(tmp_path)), delay_s=0.180,
                     step_dt=0.060)
        assert m.nd == 3
        assert m.set_rate(delay_s=0.300) == 5
        assert m.set_rate(delay_s=0.180) == 3

    def test_nd_cannot_eat_the_horizon(self, tmp_path):
        m = make_mpc(RefPath(circle_path(tmp_path)), horizon=6, step_dt=0.060)
        assert m.set_rate(delay_s=10.0) == 4       # N - 2
        assert len(m.u) == 6                       # warm start keeps its shape

    def test_solve_survives_a_rate_change(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=6.0, v=3.0))
        m = make_mpc(p)
        for delay in (0.180, 0.240, 0.120, 0.180):
            m.set_rate(delay_s=delay, apply_ahead_s=0.09, thr_lead_s=0.225)
            cmd, info = m.solve(0.05, 0.0, 0.0, 0, 3.0,
                                committed=[0.0] * m.nd)
            assert math.isfinite(cmd)
            assert math.isfinite(info['v_cmd'])


class TestRateControl:

    def _rc(self, tmp_path, **kw):
        m = make_mpc(RefPath(circle_path(tmp_path)))
        return m, RateControl(m, Actuator(dead_time=0.135), **kw)

    def test_disabled_changes_nothing(self, tmp_path):
        m, rc = self._rc(tmp_path, enabled=False)
        t = 0.0
        for _ in range(40):
            t += 0.088
            rc.update(t)
        assert rc.est.ready and abs(rc.est.period - 0.088) < 1e-6
        assert m.cfg['apply_ahead_s'] == 0.090    # measured, not acted on
        assert m.cfg['delay_s'] == 0.180
        assert rc.act.dead_time == 0.135

    def test_enabled_at_nominal_changes_nothing(self, tmp_path):
        m, rc = self._rc(tmp_path, enabled=True)
        t = 0.0
        for _ in range(40):
            t += RATE_NOMINAL
            rc.update(t)
        assert m.cfg['apply_ahead_s'] == pytest.approx(0.090)
        assert m.cfg['delay_s'] == pytest.approx(0.180)
        assert m.nd == 3

    def test_enabled_follows_a_faster_sensor(self, tmp_path):
        m, rc = self._rc(tmp_path, enabled=True, terms=RATE_ALL_TERMS)
        t = 0.0
        for _ in range(40):
            t += 0.088
            rc.update(t)
        assert m.cfg['apply_ahead_s'] == pytest.approx(0.090 * 0.088 / 0.104)
        assert m.cfg['thr_lead_s'] == pytest.approx(0.225 * 0.088 / 0.104)
        assert rc.act.dead_time < 0.135
        assert m.nd == 3                          # still 3 steps at 0.060 s

    def test_a_wild_period_is_clamped_to_the_trust_band(self, tmp_path):
        """A stalled sim, a clock step or a bridge hiccup that survives the
        median must not extrapolate the whole tune: outside
        [0.5, 1.5] x nominal the validated numbers are the better answer."""
        m, rc = self._rc(tmp_path, enabled=True, terms=RATE_ALL_TERMS)
        t = 0.0
        for _ in range(40):
            t += 0.300                          # ~3 Hz, far outside the band
            rc.update(t)
        assert rc.est.period == pytest.approx(0.300)   # still MEASURED
        assert rc.clamped > 0
        assert rc.applied == pytest.approx(1.5 * RATE_NOMINAL)
        assert m.cfg['apply_ahead_s'] == pytest.approx(0.090 * 1.5)
        assert rc.values['stale_pose_s'] == pytest.approx(0.35)
        assert rc.values['max_predict_s'] == pytest.approx(0.35)

    def test_an_impossibly_fast_period_is_clamped_too(self, tmp_path):
        m, rc = self._rc(tmp_path, enabled=True)
        t = 0.0
        for _ in range(40):
            t += 0.035
            rc.update(t)
        assert rc.applied == pytest.approx(0.5 * RATE_NOMINAL)
        assert m.cfg['apply_ahead_s'] == pytest.approx(0.090 * 0.5)

    def test_min_change_suppresses_chatter(self, tmp_path):
        m, rc = self._rc(tmp_path, enabled=True, min_change=0.010)
        t = 0.0
        for _ in range(40):
            t += 0.100                            # 4 ms off nominal
            rc.update(t)
        assert m.cfg['apply_ahead_s'] == 0.090

    def test_speed_estimator_alpha_is_retimed(self, tmp_path):
        class Est:
            alpha = 0.35
        est = Est()
        m = make_mpc(RefPath(circle_path(tmp_path)))
        rc = RateControl(m, Actuator(), enabled=True, speed_est=est,
                         enc_alpha=0.35)
        t = 0.0
        for _ in range(40):
            t += 0.088
            rc.update(t)
        assert est.alpha == pytest.approx(1.0 - (1.0 - 0.35) ** (0.088 / 0.104))


# --------------------------------------------------------------------------
# 10. iterative learning control
# --------------------------------------------------------------------------

class TestILC:

    def _ilc(self, **kw):
        base = dict(total_s=28.6, bin_m=0.25, store='')
        base.update(kw)
        return ILC(**base)

    def test_starts_at_identity(self):
        il = self._ilc()
        assert np.all(il.kappa_ff == 0.0)
        assert np.all(il.v_scale == 1.0)
        assert il.kappa_offset(np.array([0.0, 5.0])).tolist() == [0.0, 0.0]

    def test_solver_with_an_identity_table_is_the_shipping_solver(self, tmp_path):
        """The gate on shipping this at all: attaching an untrained ILC must
        not move a single command."""
        p = RefPath(circle_path(tmp_path, R=6.0, v=3.0))
        a, b = make_mpc(p), make_mpc(p)
        b.ilc = self._ilc()
        for _ in range(5):
            ca, _ = a.solve(0.08, 0.02, 0.0, 10, 3.0, committed=[0.0] * a.nd)
            cb, _ = b.solve(0.08, 0.02, 0.0, 10, 3.0, committed=[0.0] * b.nd)
            assert ca == pytest.approx(cb)

    def test_learns_against_the_error(self):
        """A lap spent LEFT of the line must produce a RIGHT-hand offset."""
        il = self._ilc(k_gain=0.3, v_enable=False)
        for k in range(200):
            il.observe(28.6 * k / 200.0, +0.10, 1.0)
        st = il.finish_lap()
        assert st is not None
        assert il.kappa_ff.max() < 0.0
        assert il.kappa_ff.min() == pytest.approx(-0.03, abs=5e-3)

    def test_offset_is_clamped_both_ways(self):
        il = self._ilc(k_gain=5.0, v_enable=False)
        for lap in range(5):
            for k in range(200):
                il.observe(28.6 * k / 200.0, +1.0, 1.0)
            il.finish_lap()
        assert np.abs(il.kappa_ff).max() <= il.k_clamp + 1e-12
        # the lateral-shift clamp is the binding one at the shipping numbers
        assert il.k_clamp == pytest.approx(2 * 0.08 / 1.5 ** 2)

    def test_leak_makes_it_forget_a_bias_that_is_gone(self):
        il = self._ilc(k_gain=0.3, leak=0.1, v_enable=False)
        for k in range(200):
            il.observe(28.6 * k / 200.0, +0.10, 1.0)
        il.finish_lap()
        peak = float(np.abs(il.kappa_ff).max())
        for _ in range(20):                       # now perfectly on line
            for k in range(200):
                il.observe(28.6 * k / 200.0, 0.0, 1.0)
            il.finish_lap()
        assert np.abs(il.kappa_ff).max() < 0.3 * peak

    def test_speed_scale_is_bounded_and_slows_where_it_is_tight(self):
        il = self._ilc(clear_target=0.30, clear_loss=0.0)
        for k in range(200):
            s = 28.6 * k / 200.0
            il.observe(s, 0.0, 0.10 if 10.0 <= s < 11.0 else 1.0)
        il.finish_lap()
        b = int(10.5 / il.bin_m)
        assert il.v_scale[b] < 1.0
        assert il.v_scale.min() >= il.v_lo and il.v_scale.max() <= il.v_hi

    def test_a_tight_corner_does_not_slow_the_whole_lap(self):
        """The first version eroded the CUMULATIVE table, so one tight hairpin
        pulled all 114 bins to the floor over 29 laps and cost 1.2 s/lap."""
        il = self._ilc(clear_target=0.30, clear_loss=0.0)
        for _ in range(30):
            for k in range(200):
                s = 28.6 * k / 200.0
                il.observe(s, 0.0, 0.10 if 10.0 <= s < 11.0 else 1.0)
            il.finish_lap()
        far = il.v_scale[int(20.0 / il.bin_m)]
        assert far >= 1.0
        assert il.v_scale[int(10.5 / il.bin_m)] == pytest.approx(il.v_lo)

    def test_target_never_exceeds_what_the_line_offers(self):
        """A line whose own corridor is 0.25 m must not be slowed every lap
        for failing to reach a 0.30 m target it can never reach."""
        il = self._ilc(clear_target=0.30, clear_loss=0.05)
        il.nominal_clear = np.full(il.nb, 0.25)
        for _ in range(10):
            for k in range(200):
                il.observe(28.6 * k / 200.0, 0.0, 0.24)
            il.finish_lap()
        assert il.v_scale.min() >= 1.0

    def test_sparse_laps_are_interpolated_not_zero_filled(self):
        """At 10 Hz a 28.6 m lap is ~70 poses: most bins are never visited."""
        # `min_cover` is deliberately relaxed here: the subject is `_fill`,
        # not the fragment gate, and 286 bins cannot be covered in one lap.
        il = self._ilc(bin_m=0.10, k_gain=0.3, v_enable=False, min_cover=0.15)
        for k in range(70):
            il.observe(28.6 * k / 70.0, +0.10, 1.0)
        st = il.finish_lap()
        assert st is not None and st['cover'] < 0.35
        # every bin learned, including the ones no pose landed in
        assert np.abs(il.kappa_ff).min() > 0.5 * np.abs(il.kappa_ff).max()

    def test_a_lap_fragment_is_not_a_lap(self):
        """A collision teleport cuts a lap short.  `_fill` then interpolates
        the 80% of bins that were never driven from the 20% that were, so a
        fragment carrying a constant error pins the ENTIRE table to the clamp
        -- measured, all 114 bins to -0.0711 1/m off one fragment."""
        il = self._ilc(v_enable=False)
        for s_m in np.arange(0.0, 0.20 * 28.6, 0.25):
            il.observe(float(s_m), +0.30, 0.5)
        assert il.finish_lap() is None
        assert np.all(il.kappa_ff == 0.0)
        assert il.laps == 0

    def test_a_full_lap_is_still_a_lap(self):
        """The fragment gate must not reject the thing it is protecting."""
        il = self._ilc(v_enable=False)
        for s_m in np.arange(0.0, 28.6, 0.20):      # a 10 Hz lap is coarser
            il.observe(float(s_m), +0.10, 1.0)
        st = il.finish_lap()
        assert st is not None and st['cover'] >= il.min_cover
        assert il.laps == 1 and np.all(il.kappa_ff < 0.0)

    def test_a_torn_store_file_does_not_stop_the_car(self, tmp_path):
        """`save` runs in the control callback at a lap boundary; a run killed
        there used to leave a half-written zip that raised BadZipFile out of
        `MPC3Tracker.__init__` -- no tracker, no race."""
        store = str(tmp_path / 'ilc.npz')
        trained = self._ilc(store=store)
        trained.observe(0.0, 0.1, 1.0)
        trained.save()
        raw = open(store, 'rb').read()
        assert len(raw) > 0
        for corrupt in (raw[:len(raw) // 2], b'', b'not a zip at all'):
            with open(store, 'wb') as f:
                f.write(corrupt)
            il = ILC(total_s=28.6, bin_m=0.25, store=store)   # must not raise
            assert il.laps == 0
            assert np.all(il.kappa_ff == 0.0) and np.all(il.v_scale == 1.0)

    def test_save_is_atomic_and_leaves_no_debris(self, tmp_path):
        il = self._ilc(store=str(tmp_path / 'ilc.npz'))
        il.save()
        il.save()
        names = sorted(q.name for q in tmp_path.iterdir())
        assert names == ['ilc.npz']              # no *.tmp.npz left behind
        assert ILC(total_s=28.6, bin_m=0.25,
                   store=str(tmp_path / 'ilc.npz')).laps == 0

    def test_an_empty_lap_is_not_evidence(self):
        il = self._ilc()
        for k in range(5):
            il.observe(0.1 * k, 1.0, 1.0)
        assert il.finish_lap() is None
        assert np.all(il.kappa_ff == 0.0)

    def test_invalidate_drops_the_lap(self):
        il = self._ilc(v_enable=False)
        for k in range(200):
            il.observe(28.6 * k / 200.0, +0.10, 1.0)
        il.invalidate()
        assert il.finish_lap() is None
        assert np.all(il.kappa_ff == 0.0)
        # ...and the next lap learns normally again
        for k in range(200):
            il.observe(28.6 * k / 200.0, +0.10, 1.0)
        assert il.finish_lap() is not None

    def test_persists_and_reloads(self, tmp_path):
        store = str(tmp_path / 'ilc')
        il = self._ilc(store=store, v_enable=False)
        for k in range(200):
            il.observe(28.6 * k / 200.0, +0.10, 1.0)
        il.finish_lap()
        back = ILC(total_s=28.6, bin_m=0.25, store=store, v_enable=False)
        assert back.laps == 1
        assert np.allclose(back.kappa_ff, il.kappa_ff)

    def test_a_table_for_another_line_is_refused(self, tmp_path):
        store = str(tmp_path / 'ilc')
        il = self._ilc(store=store, v_enable=False)
        for k in range(200):
            il.observe(28.6 * k / 200.0, +0.10, 1.0)
        il.finish_lap()
        other = ILC(total_s=56.7, bin_m=0.25, store=store)
        assert other.laps == 0
        assert np.all(other.kappa_ff == 0.0)

    def test_a_missing_store_starts_from_identity(self, tmp_path):
        il = ILC(total_s=28.6, store=str(tmp_path / 'nothing-here'))
        assert il.laps == 0 and np.all(il.v_scale == 1.0)


class TestCorridorClearance:

    def test_infinite_without_a_map(self, tmp_path):
        p = RefPath(circle_path(tmp_path))
        assert corridor_clearance(p, 0, 0.0) == float('inf')

    def test_shrinks_as_the_car_moves_off_line(self, tmp_path):
        p = RefPath(circle_path(tmp_path))
        p.w_left = np.full(p.n, 0.6)
        p.w_right = np.full(p.n, 0.6)
        p.has_corridor = True
        on = corridor_clearance(p, 10, 0.0)
        off = corridor_clearance(p, 10, 0.2)
        assert on == pytest.approx(0.6 - 0.5 * vehicle.CAR_WIDTH)
        assert off == pytest.approx(on - 0.2)


class TestRateInvariantEstimatorConstants:
    """`vehicle.alpha_for_lag` / `noise_scale_for_dt`, used by `localization`.

    The localization node cannot be imported on the host (rclpy), so the two
    formulas live in `vehicle` and are tested here.  The property that makes
    them safe to ship is that both are the IDENTITY at the period every default
    was tuned at; what they buy is that the filter's speed lag and its
    diffusion stop depending on which machine the simulator runs on (measured
    periods: 0.088 s in the submission image, 0.097 s at the organisers,
    0.104 s on the dev host).
    """

    def test_alpha_is_the_identity_at_the_validated_period(self):
        assert vehicle.alpha_for_lag(vehicle.RATE_NOMINAL,
                                     vehicle.ENC_TAU) == pytest.approx(0.35, abs=5e-4)

    def test_alpha_holds_the_lag_in_seconds(self):
        # A first-order per-sample low pass has lag dt*(1-a)/a; that is what is
        # being held constant, not `a`.
        for dt in (0.060, 0.088, 0.097, 0.104, 0.140):
            a = vehicle.alpha_for_lag(dt, vehicle.ENC_TAU)
            assert 0.0 < a < 1.0
            assert dt * (1.0 - a) / a == pytest.approx(vehicle.ENC_TAU, rel=1e-9)

    def test_alpha_rises_with_the_period_and_survives_junk(self):
        assert (vehicle.alpha_for_lag(0.088) < vehicle.alpha_for_lag(0.104)
                < vehicle.alpha_for_lag(0.140))
        assert vehicle.alpha_for_lag(0.0) == 1.0      # no dt: no filtering
        assert vehicle.alpha_for_lag(-1.0) == 1.0
        assert vehicle.alpha_for_lag(0.1, 0.0) == 1.0

    def test_noise_scale_fixes_diffusion_per_unit_time(self):
        assert vehicle.noise_scale_for_dt(vehicle.RATE_NOMINAL) == pytest.approx(1.0)
        # 0.18 m per update at 0.088 s is 9% more diffusive per second than at
        # 0.104 s; scaling by sqrt(dt) removes exactly that.
        for dt in (0.060, 0.088, 0.104, 0.140):
            sigma = 0.18 * vehicle.noise_scale_for_dt(dt)
            assert sigma / math.sqrt(dt) == pytest.approx(
                0.18 / math.sqrt(vehicle.RATE_NOMINAL), rel=1e-9)
        assert vehicle.noise_scale_for_dt(0.088) < 1.0 < vehicle.noise_scale_for_dt(0.120)
        assert vehicle.noise_scale_for_dt(0.0) == 1.0
        assert vehicle.noise_scale_for_dt(0.1, 0.0) == 1.0


# --------------------------------------------------------------------------
# the measured dead-time law: `delay_tick` + `nd_mode`
# --------------------------------------------------------------------------

class TestDeadTimeTickLaw:
    """`period + c`, the law the live cmd->feedback logs identify.

    Fitted offline on the six race logs carrying both `cmd` and `steering`
    (slew-limited rack, 3.2 rad/s, per-20 s-segment median): pose-to-rack =
    period + 0.042 s at 0.058 and + 0.051-0.059 s at 0.094-0.100.
    """

    def test_delay_tick_scales_with_the_whole_period(self):
        a = RateAdapter(terms=('delay_tick',), dead_tick_c=0.045)
        assert a.values(0.058)['cmd_dead_time'] == pytest.approx(0.103)
        assert a.values(0.100)['cmd_dead_time'] == pytest.approx(0.145)
        # delay_s follows it by the estimation lag, exactly as `delay` does.
        assert a.values(0.058)['delay_s'] == pytest.approx(0.103 + 0.045)

    def test_delay_tick_beats_delay_when_both_are_listed(self):
        a = RateAdapter(terms=('delay', 'delay_tick'))
        assert a.values(0.058)['cmd_dead_time'] == pytest.approx(0.103)

    def test_default_terms_do_not_touch_the_dead_time(self):
        a = RateAdapter()                      # shipping terms
        for p in (0.058, 0.104, 0.140):
            assert a.values(p)['cmd_dead_time'] == pytest.approx(0.135)

    def test_nd_mode_fixed_is_the_shipping_structure(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=3.0, n=400))
        m = make_mpc(p)
        assert m.cfg['nd_mode'] == 'fixed' and m.nd == 3
        for period in (0.058, 0.104):
            m.period = period
            assert m.nd == 3                   # period does not enter

    def test_nd_mode_period_follows_the_measured_period(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=3.0, n=400))
        m = make_mpc(p, nd_mode='period', dead_tick_c=0.045)
        m.period = 0.058
        assert m.nd == round((0.058 + 0.045) / 0.060)
        m.period = 0.160
        assert m.nd == round((0.160 + 0.045) / 0.060)

    def test_nd_and_the_actuator_agree_under_the_pair(self, tmp_path):
        """`delay_tick` + `nd_mode=period` describe ONE delay, not two."""
        p = RefPath(circle_path(tmp_path, R=3.0, n=400))
        m = make_mpc(p, nd_mode='period', dead_tick_c=0.045)
        act = Actuator(dead_time=0.135)
        ctl = RateControl(m, act, enabled=True,
                          terms=('apply', 'thr', 'predict', 'enc', 'pose',
                                 'delay_tick'), dead_tick_c=0.045)
        t = 0.0
        for period in (0.058, 0.075, 0.104):
            for _ in range(40):
                t += period
                ctl.update(t)
            assert act.dead_time == pytest.approx(period + 0.045, abs=1e-9)
            assert m.nd == m._nd_for(act.dead_time)


# --------------------------------------------------------------------------
# thr_source / thr_brake_lead_s (OPT-IN throttle setpoint source)
# --------------------------------------------------------------------------

def _decel_line(tmp_path, name='decel.csv', n=600, R=40.0):
    """A closed circle whose PROFILE brakes over one arc and accelerates back.

    The point of the fixture is the ``ax`` column: ``thr_brake_lead_s`` is only
    allowed to act where the offline profile's ``ax`` is negative, so a line
    with an honest sign change in column 6 is what the test needs.
    """
    t = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    xy = np.column_stack([R * np.cos(t), R * np.sin(t)])
    d = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(d)])
    fwd = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    psi = np.arctan2(fwd[:, 1], fwd[:, 0])
    dpsi = (np.roll(psi, -1) - np.roll(psi, 1) + np.pi) % (2 * np.pi) - np.pi
    ds = np.roll(s, -1) - np.roll(s, 1)
    kappa = dpsi / np.where(np.abs(ds) > 1e-9, ds, 1e-9)
    half = n // 2
    v = np.empty(n)
    v[:half] = np.linspace(8.0, 3.0, half)          # braking arc
    v[half:] = np.linspace(3.0, 8.0, n - half)      # accelerating arc
    ax = np.gradient(0.5 * v ** 2, s)
    rows = np.column_stack([s, xy[:, 0], xy[:, 1], psi, kappa, v, ax])
    p = tmp_path / name
    np.savetxt(p, rows, delimiter=',')
    return str(p)


class TestThrottleSetpointSource:
    """`v_cmd` is the throttle's setpoint, and these two knobs choose it."""

    def test_defaults_are_the_shipping_ceiling(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=5.0))
        assert MPC3(p).cfg['thr_source'] == 'ceiling'
        assert MPC3(p).cfg['thr_brake_lead_s'] == 0.0
        base = make_mpc(p)
        explicit = make_mpc(p, thr_source='ceiling', thr_brake_lead_s=0.0)
        for _ in range(4):
            _, a = base.solve(0.05, 0.0, 0.0, 10, 5.0)
            _, b = explicit.solve(0.05, 0.0, 0.0, 10, 5.0)
        assert b['v_cmd'] == pytest.approx(a['v_cmd'], abs=1e-12)

    def test_min_is_the_ceiling_while_the_ILC_is_off(self, tmp_path):
        """The ceiling is built by CUTTING the profile, so it cannot exceed it.

        `v_lim` starts as `P.v_prof[idx]` and every term after that is a
        `min`, so with the ILC off `min(ceiling, profile) == ceiling` exactly
        and `thr_source='min'` is a no-op.  It is not redundant: `ILC.v_scale`
        goes to `v_hi` 1.05, and there the two differ.  `'profile'` is the
        source that genuinely ignores the cuts.
        """
        p = RefPath(_decel_line(tmp_path))
        outs = {}
        for src in ('ceiling', 'profile', 'min'):
            m = make_mpc(p, thr_source=src)
            for _ in range(4):
                _, info = m.solve(0.0, 0.0, 0.0, 60, 7.0)
            outs[src] = info['v_cmd']
            if src == 'ceiling':
                assert (info['v_ceiling']
                        <= p.v_prof[info['idx']] + 1e-9).all()
        assert outs['min'] == pytest.approx(outs['ceiling'], abs=1e-12)
        assert outs['profile'] >= outs['ceiling'] - 1e-9

    def test_brake_lead_only_acts_where_the_profile_brakes(self, tmp_path):
        p = RefPath(_decel_line(tmp_path))
        n = p.n

        def cmd_at(i0, v0, **cfg):
            m = make_mpc(p, **cfg)
            for _ in range(4):
                _, info = m.solve(0.0, 0.0, 0.0, i0, v0)
            return info['v_cmd']

        # braking arc: the extra lead must lower the setpoint
        brake_i = 60
        assert p.a_prof[brake_i] < 0.0
        base = cmd_at(brake_i, 7.0)
        lead = cmd_at(brake_i, 7.0, thr_brake_lead_s=0.30)
        assert lead < base - 1e-6
        # accelerating arc: profile ax > 0 ahead, so nothing may change
        acc_i = (n // 2) + 60
        assert p.a_prof[acc_i] > 0.0
        assert (cmd_at(acc_i, 4.0, thr_brake_lead_s=0.30)
                == pytest.approx(cmd_at(acc_i, 4.0), abs=1e-12))

    def test_brake_lead_is_one_sided(self, tmp_path):
        """It may only ever brake earlier, never command more speed."""
        p = RefPath(_decel_line(tmp_path))
        for i0 in range(0, p.n, 40):
            a = make_mpc(p)
            b = make_mpc(p, thr_brake_lead_s=0.30)
            for _ in range(4):
                _, ia = a.solve(0.0, 0.0, 0.0, i0, float(p.v_prof[i0]))
                _, ib = b.solve(0.0, 0.0, 0.0, i0, float(p.v_prof[i0]))
            assert ib['v_cmd'] <= ia['v_cmd'] + 1e-9

    def test_the_sources_are_read_through_one_index_set(self, tmp_path):
        """`solve` moves `idx` after `_speed_plan`; the sources must not mix.

        The ceiling is indexed by `v_ceiling_idx`.  Reading the profile through
        `solve`'s later `idx` compared two different points on the track and
        made `min` differ from `ceiling` by up to 0.04 s of harness lap time.
        """
        p = RefPath(_decel_line(tmp_path))
        m = make_mpc(p, thr_source='ceiling')
        for _ in range(4):
            _, info = m.solve(0.0, 0.0, 0.0, 60, 7.0)
        assert m.v_ceiling_idx is not None
        assert len(m.v_ceiling_idx) == len(info['v_ceiling'])
