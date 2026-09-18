"""The s-indexed parameter schedule (OPT-IN) for mpc3.

What these guard, in order of what would actually hurt on finals day:

* with no schedule, or with a schedule whose windows are somewhere else on the
  lap, the solver's output is BIT-for-bit what it was before the feature
  existed -- the shipping tune is not allowed to move;
* the solve's own config is restored afterwards, so nothing outside the solve
  (the rate adapter, the node's logging) ever sees an override;
* the blend is a real ramp, wraps the start/finish line, and an ambiguous
  (overlapping) file is REJECTED at load rather than silently resolved;
* a localization jump drops the schedule back on the global values and ramps
  it in again, because a schedule indexed by the wrong arc length would apply
  the hairpin's brake lead on the straight.
"""

import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from pit_crew.mpc3_core import (DEFAULTS, MPC3, RefPath,          # noqa: E402
                                ParamSchedule, SCHED_PARAMS)

from test_mpc3_core import circle_path, make_mpc                  # noqa: E402


def write_sched(tmp_path, windows, blend_m=1.0, total_s=0.0, name='s.json'):
    p = tmp_path / name
    p.write_text(json.dumps(dict(blend_m=blend_m, total_s=total_s,
                                 windows=windows)))
    return str(p)


def solve_seq(m, n=12, i0=0, v=3.0):
    """A short deterministic sequence of published angles."""
    out = []
    idx = i0
    for k in range(n):
        cmd, _info = m.solve(0.12, 0.05, 0.0, idx % m.path.n, v)
        out.append(cmd)
        idx += 3
    return np.array(out)


# --------------------------------------------------------------------------
# 1. the file format
# --------------------------------------------------------------------------

class TestFormat:
    def test_weight_is_one_inside_and_ramps_outside(self):
        s = ParamSchedule([dict(name='w', s0=10.0, s1=20.0, e_free=0.3)],
                          blend_m=2.0, total_s=100.0)
        w = s.windows[0]
        assert float(s.weight(w, 15.0)) == 1.0
        assert float(s.weight(w, 10.0)) == 1.0
        assert float(s.weight(w, 20.0)) == 1.0
        assert float(s.weight(w, 21.0)) == pytest.approx(0.5)
        assert float(s.weight(w, 9.0)) == pytest.approx(0.5)
        assert float(s.weight(w, 22.5)) == 0.0
        assert float(s.weight(w, 7.5)) == 0.0

    def test_window_wraps_the_start_line(self):
        s = ParamSchedule([dict(name='w', s0=95.0, s1=5.0, v_scale=0.9)],
                          blend_m=1.0, total_s=100.0)
        w = s.windows[0]
        assert float(s.weight(w, 99.0)) == 1.0
        assert float(s.weight(w, 0.0)) == 1.0
        assert float(s.weight(w, 3.0)) == 1.0
        assert float(s.weight(w, 5.5)) == pytest.approx(0.5)
        assert float(s.weight(w, 94.5)) == pytest.approx(0.5)
        assert float(s.weight(w, 50.0)) == 0.0

    def test_overrides_blend_from_the_global_value(self):
        s = ParamSchedule([dict(name='w', s0=10.0, s1=20.0, e_free=0.28)],
                          blend_m=2.0, total_s=100.0)
        base = dict(e_free=0.18)
        assert s.overrides(15.0, base)['e_free'] == pytest.approx(0.28)
        assert s.overrides(21.0, base)['e_free'] == pytest.approx(0.23)
        assert s.overrides(50.0, base) == {}

    def test_array_is_the_same_law_vectorized(self):
        s = ParamSchedule([dict(name='w', s0=10.0, s1=20.0, v_scale=1.10)],
                          blend_m=2.0, total_s=100.0)
        ss = np.array([5.0, 9.0, 15.0, 21.0, 50.0])
        got = s.array(ss, 'v_scale', 1.0)
        assert got == pytest.approx([1.0, 1.05, 1.10, 1.05, 1.0])
        assert s.array(ss, 'e_free', 0.18) is None      # nothing sets it

    def test_gain_scales_every_deviation(self):
        s = ParamSchedule([dict(name='w', s0=10.0, s1=20.0, k_e=4.0)],
                          blend_m=1.0, total_s=100.0)
        assert s.overrides(15.0, dict(k_e=2.0), gain=0.5)['k_e'] == 3.0
        # gain 0 names nothing at all, which is how the global value applies
        assert s.overrides(15.0, dict(k_e=2.0), gain=0.0) == {}

    def test_unschedulable_key_is_rejected(self):
        # `step_dt` would re-slice the warm start mid-lap.
        with pytest.raises(ValueError, match='not schedulable'):
            ParamSchedule([dict(s0=1.0, s1=2.0, step_dt=0.1)], total_s=10.0)
        assert 'horizon' not in SCHED_PARAMS and 'delay_s' not in SCHED_PARAMS

    def test_overlapping_windows_on_one_key_are_rejected(self):
        with pytest.raises(ValueError, match='overlap'):
            ParamSchedule([dict(name='a', s0=10.0, s1=20.0, e_free=0.2),
                           dict(name='b', s0=20.5, s1=30.0, e_free=0.3)],
                          blend_m=1.0, total_s=100.0)

    def test_adjacent_windows_on_different_keys_are_fine(self):
        s = ParamSchedule([dict(name='a', s0=10.0, s1=20.0, e_free=0.2),
                           dict(name='b', s0=20.5, s1=30.0, k_e=3.0)],
                          blend_m=1.0, total_s=100.0)
        assert len(s.windows) == 2

    def test_empty_or_degenerate_windows_are_rejected(self):
        with pytest.raises(ValueError, match='overrides nothing'):
            ParamSchedule([dict(s0=1.0, s1=2.0)], total_s=10.0)
        with pytest.raises(ValueError, match='zero width'):
            ParamSchedule([dict(s0=3.0, s1=3.0, k_e=1.0)], total_s=10.0)

    def test_load_reads_the_file(self, tmp_path):
        p = write_sched(tmp_path, [dict(name='w', s0=1.0, s1=2.0, k_e=3.0)],
                        total_s=10.0)
        s = ParamSchedule.load(p)
        assert s.keys == ['k_e'] and s.total == 10.0
        assert 'k_e=3' in s.describe()


# --------------------------------------------------------------------------
# 2. the shipping tune does not move
# --------------------------------------------------------------------------

class TestDefaultsUnchanged:
    def test_no_schedule_is_the_old_solver(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=3.0))
        assert make_mpc(p).sched is None
        assert DEFAULTS['param_schedule'] == ''
        assert DEFAULTS['v_scale'] == 1.0

    def test_a_window_elsewhere_on_the_lap_changes_nothing(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=3.0))
        total = float(p.s[-1] + p.spacing)
        far = write_sched(tmp_path,
                          [dict(name='far', s0=0.45 * total, s1=0.55 * total,
                                barrier_margin=0.60, v_scale=0.5)],
                          blend_m=0.5, total_s=total)
        a = solve_seq(make_mpc(p))
        b = solve_seq(make_mpc(p, param_schedule=far))
        assert np.array_equal(a, b)

    def test_cfg_is_restored_after_the_solve(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=3.0))
        total = float(p.s[-1] + p.spacing)
        here = write_sched(tmp_path,
                           [dict(name='here', s0=0.0, s1=0.2 * total,
                                 barrier_margin=0.60)],
                           blend_m=0.5, total_s=total)
        m = make_mpc(p, param_schedule=here)
        before = dict(m.cfg)
        cmd, info = m.solve(0.12, 0.05, 0.0, 0, 3.0)
        assert m.cfg == before
        assert info['sched']['barrier_margin'] == pytest.approx(0.60)
        assert math.isfinite(cmd)


# --------------------------------------------------------------------------
# 3. the overrides actually reach the solver
# --------------------------------------------------------------------------

class TestApplied:
    def test_v_scale_cuts_the_speed_plan_inside_its_window(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=4.0))
        total = float(p.s[-1] + p.spacing)
        sp = write_sched(tmp_path, [dict(name='slow', s0=0.0, s1=0.5 * total,
                                         v_scale=0.5)],
                         blend_m=0.5, total_s=total)
        m = make_mpc(p, param_schedule=sp)
        m.solve(0.0, 0.0, 0.0, 0, 4.0)
        inside = float(np.max(m.v_ceiling))
        m2 = make_mpc(p)
        m2.solve(0.0, 0.0, 0.0, 0, 4.0)
        assert inside < 0.75 * float(np.max(m2.v_ceiling))

    def test_v_scale_is_read_at_the_horizon_not_at_the_car(self, tmp_path):
        """A slow window AHEAD must already be in the ceiling, or the backward
        brake pass has nothing to brake with."""
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=4.0))
        total = float(p.s[-1] + p.spacing)
        # 1.5 m ahead of the car, well inside a ~1.2 s horizon at 4 m/s.
        sp = write_sched(tmp_path, [dict(name='ahead', s0=1.5, s1=6.0,
                                         v_scale=0.5)],
                         blend_m=0.2, total_s=total)
        m = make_mpc(p, param_schedule=sp)
        m.solve(0.0, 0.0, 0.0, 0, 4.0)
        assert m.last_sched == {}                  # nothing scheduled AT s=0
        assert float(np.min(m.v_ceiling)) < 3.0    # ...but the plan sees it

    def test_barrier_margin_override_moves_the_corridor(self, tmp_path):
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=3.0))
        p.w_left = np.full(p.n, 0.9)
        p.w_right = np.full(p.n, 0.9)
        p.has_corridor = True
        total = float(p.s[-1] + p.spacing)
        sp = write_sched(tmp_path, [dict(name='tight', s0=0.0, s1=0.3 * total,
                                         barrier_margin=0.60)],
                         blend_m=0.5, total_s=total)
        a = solve_seq(make_mpc(p), n=6)
        b = solve_seq(make_mpc(p, param_schedule=sp), n=6)
        assert not np.array_equal(a, b)


# --------------------------------------------------------------------------
# 4. robustness to a localization jump
# --------------------------------------------------------------------------

class TestJumpFallback:
    def sched_mpc(self, tmp_path, **cfg):
        p = RefPath(circle_path(tmp_path, R=6.0, n=400, v=3.0))
        total = float(p.s[-1] + p.spacing)
        sp = write_sched(tmp_path, [dict(name='here', s0=0.0, s1=0.9 * total,
                                         k_e=6.0)],
                         blend_m=0.5, total_s=total)
        return make_mpc(p, param_schedule=sp, sched_hold_n=4, **cfg)

    def test_hold_then_ramp_back(self, tmp_path):
        m = self.sched_mpc(tmp_path)
        _c, info = m.solve(0.0, 0.0, 0.0, 0, 3.0)
        assert info['sched_gain'] == 1.0
        assert info['sched']['k_e'] == pytest.approx(6.0)
        m.hold_schedule()
        gains = []
        for k in range(5):
            _c, info = m.solve(0.0, 0.0, 0.0, 3 * (k + 1), 3.0)
            gains.append(info['sched_gain'])
        # the first solve after the hold is back on the GLOBAL value, and the
        # way back is a ramp, not a step
        assert gains[0] == 0.0
        assert gains == sorted(gains) and gains[-1] == 1.0
        assert all(0.0 <= g <= 1.0 for g in gains)

    def test_an_arc_jump_trips_the_hold(self, tmp_path):
        m = self.sched_mpc(tmp_path)
        m.solve(0.0, 0.0, 0.0, 0, 3.0)
        # ~20 m of arc in one 0.104 s solve at 3 m/s: a relocalization, not
        # the car.
        _c, info = m.solve(0.0, 0.0, 0.0, 200, 3.0)
        assert info['sched_gain'] == 0.0
        assert info['sched'] == {}          # the global `k_e` is what solved

    def test_normal_motion_does_not_trip_it(self, tmp_path):
        m = self.sched_mpc(tmp_path)
        for k in range(8):
            _c, info = m.solve(0.0, 0.0, 0.0, (2 * k) % 400, 3.0)
            assert info['sched_gain'] == 1.0


# --------------------------------------------------------------------------
# 5. the shipping schedule file
# --------------------------------------------------------------------------

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.mark.parametrize('name', ['finals_mesh_sched.json',
                                  'finals_mesh_sched_fast.json'])
def test_shipping_schedule_loads_against_its_own_line(name):
    p = os.path.join(ROOT, 'data', 'racelines', name)
    if not os.path.exists(p):
        pytest.skip(f'{name} not in this tree')
    s = ParamSchedule.load(p)
    line = os.path.join(ROOT, 'data', 'racelines',
                        json.load(open(p))['line'])
    if os.path.exists(line):
        total = float(RefPath(line).s[-1]) + RefPath(line).spacing
        assert abs(s.total - total) < 0.05
    for w in s.windows:
        assert 0.0 <= w['s0'] < s.total and 0.0 < w['width'] < 0.5 * s.total
