"""Lap-checkpoint geometry: the constraint, the measurement and the veto.

Why this exists: on 2026-09-12 the CDC-TF `fast3_m53` run took 32 laps with no
collisions, and one of them read 20.4 s instead of 10.2 -- the simulator's own
`lap_time` ran straight through a start-line crossing without resetting,
because a checkpoint had not fired.  The rules say the lap timer does not stop
until every checkpoint is passed, and that the checkpoint locations will not be
revealed; `tools/map_from_mesh.py` reads them out of the scene anyway, so a
generated line can be held inside every trigger box instead of hoping.

The trigger geometry is measured, not assumed: every checkpoint in the scene is
a BoxCollider with `m_IsTrigger` true and `m_Size` 1x1x1, scaled by its
transform to `width` x 0.254 x 0.100 m -- so `width` is the FULL lateral extent
and the box is only 0.1 m deep along travel.
"""

import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
sys.path.insert(0, os.path.join(REPO, 'src', 'pit_crew'))

import optimize_raceline as opt                               # noqa: E402
import pick_config                                            # noqa: E402

CP = os.path.join(REPO, 'data', 'maps', 'cdctf2025_mesh_checkpoints.csv')


def line(name):
    return os.path.join(REPO, 'data', 'racelines', name + '.csv')


def test_load_drops_the_duplicate_finish_lines():
    cps = opt.load_checkpoints(CP)
    assert [c[0] for c in cps] == list(range(21))
    # Checkpoint 0 and "Agent-A Finish Line" are the same gate in the scene.
    assert cps[0][1:] == pytest.approx((0.675, 7.98, -1.5708, 1.5))


def test_margin_is_half_width_minus_the_crossing_offset():
    """A straight path through a gate, offset by a known amount."""
    cps = [(0, 0.0, 0.0, 0.0, 2.0)]        # gate on the y axis, 2 m wide
    path = np.column_stack([np.linspace(-1.0, 1.0, 21),
                            np.full(21, 0.30)])
    m = opt.checkpoint_margins(path, cps, closed=False)
    assert m[0] == pytest.approx(1.0 - 0.30, abs=1e-9)


def test_a_path_outside_the_box_reads_negative():
    cps = [(0, 0.0, 0.0, 0.0, 2.0)]
    path = np.column_stack([np.linspace(-1.0, 1.0, 21), np.full(21, 1.40)])
    assert opt.checkpoint_margins(path, cps, closed=False)[0] < 0.0


def test_a_path_that_never_reaches_the_plane_is_none():
    cps = [(0, 0.0, 0.0, 0.0, 2.0)]
    path = np.column_stack([np.linspace(-2.0, -1.0, 11), np.zeros(11)])
    assert opt.checkpoint_margins(path, cps, closed=False)[0] is None


@pytest.mark.parametrize('name,cp_map,floor', [
    ('cdctf2025_mesh_safe', 'cdctf2025_mesh', 0.35),
    ('cdctf2025_mesh_fast2_m50', 'cdctf2025_mesh', 0.35),
    ('cdctf2025_mesh_fast3_m53', 'cdctf2025_mesh', 0.35),
    ('porto_mesh_cr_dk0.8', 'porto_mesh', 0.35),
    ('icra2025_mesh_fast2_m50', 'icra2025_mesh', 0.35),
    ('icra2026_mesh_safe', 'icra2026_mesh', 0.35),
])
def test_every_shipped_line_passes_inside_every_trigger(name, cp_map, floor):
    """Regression on the whole line family, not just the one that was driven.

    Measured minima at the time of writing: cdctf safe 0.714, fast2_m50 0.694,
    fast3_m53 0.715, Porto cr_dk0.8 0.518 (the narrowest anything shipped
    leaves), ICRA-2025 fast2_m50 0.588, ICRA-2026 safe 0.917.
    """
    csv = line(name)
    cps_csv = os.path.join(REPO, 'data', 'maps', cp_map + '_checkpoints.csv')
    if not (os.path.exists(csv) and os.path.exists(cps_csv)):
        pytest.skip('line or checkpoint export not kept')
    cps = opt.load_checkpoints(cps_csv)
    xy = np.loadtxt(csv, delimiter=',')[:, 1:3]
    mg = opt.checkpoint_margins(xy, cps)
    assert not [k for k, v in mg.items() if v is None]
    assert min(mg.values()) >= floor


def one_gate(tmp_path, x, y, yaw, w):
    """A checkpoint export with a single gate, for the bounds arithmetic."""
    p = tmp_path / 'one_checkpoints.csv'
    p.write_text('kind,name,x,y,yaw,width\n'
                 f'spawn,Spawn 1,0,0,0,1.000\n'
                 f'checkpoint,0,{x:.4f},{y:.4f},{yaw:.5f},{w:.3f}\n')
    return str(p)


def test_bounds_pin_only_the_two_points_that_bracket_a_gate(tmp_path):
    class A:
        checkpoints = one_gate(tmp_path, 0.0, 0.0, 0.0, 1.5)
        checkpoint_margin = 0.25

    # Reference line along +x through the gate centre; normals across it.
    n = 41
    ref = np.column_stack([np.linspace(-2.0, 2.0, n), np.zeros(n),
                           np.full(n, 1.0), np.full(n, 1.0)])
    nv = np.tile(np.array([0.0, 1.0]), (n, 1))
    lo, hi = opt.checkpoint_bounds(ref, nv, A)
    tight = np.flatnonzero(np.isfinite(lo) | np.isfinite(hi))
    assert len(tight) == 2                      # the two bracketing points
    # 1.5 m wide, reference on the centre, 0.25 m of margin -> +/- 0.5 m.
    assert lo[tight[0]] == pytest.approx(-0.5, abs=1e-6)
    assert hi[tight[0]] == pytest.approx(0.5, abs=1e-6)


def test_bounds_are_one_sided_when_the_gate_is_off_centre(tmp_path):
    """A gate whose centre is 0.3 m left of the reference line."""
    class A:
        checkpoints = one_gate(tmp_path, 0.0, 0.3, 0.0, 1.5)
        checkpoint_margin = 0.25

    n = 41
    ref = np.column_stack([np.linspace(-2.0, 2.0, n), np.zeros(n),
                           np.full(n, 1.0), np.full(n, 1.0)])
    nv = np.tile(np.array([0.0, 1.0]), (n, 1))
    lo, hi = opt.checkpoint_bounds(ref, nv, A)
    j = int(np.flatnonzero(np.isfinite(lo))[0])
    # alpha is measured along +y; the box spans y in [0.3-0.5, 0.3+0.5].
    assert lo[j] == pytest.approx(-0.2, abs=1e-6)
    assert hi[j] == pytest.approx(0.8, abs=1e-6)


def test_a_margin_wider_than_the_gate_is_refused(tmp_path):
    class A:
        checkpoints = one_gate(tmp_path, 0.0, 0.0, 0.0, 1.5)
        checkpoint_margin = 0.80          # the gate is only 1.5 m wide

    n = 41
    ref = np.column_stack([np.linspace(-2.0, 2.0, n), np.zeros(n),
                           np.full(n, 1.0), np.full(n, 1.0)])
    nv = np.tile(np.array([0.0, 1.0]), (n, 1))
    with pytest.raises(SystemExit):
        opt.checkpoint_bounds(ref, nv, A)


def test_pick_config_reports_and_fails_below_the_floor(capsys):
    csv = line('cdctf2025_mesh_fast3_m53')
    if not os.path.exists(csv):
        pytest.skip('line not kept')
    geo = pick_config.checkpoint_geometry(csv, CP)
    assert geo['n_checkpoints'] == 21
    assert geo['checkpoints_skipped'] == []
    assert geo['checkpoint_margin_min'] == pytest.approx(0.715, abs=5e-3)

    # The floor is what makes it a gate rather than a readout.  Passing argv
    # keeps main() from calling sys.exit, so the record can be inspected.
    rec = pick_config.main(['cdctf2025_mesh', '--line', csv, '--no-write',
                            '--checkpoint-min-margin', '0.80'])
    assert 'checkpoint_fail' in rec
    assert 'FAIL' in capsys.readouterr().out


def test_the_floor_lets_every_shipped_line_through():
    """0.35 m is below everything we ship and above every clean driven pass."""
    assert pick_config.CHECKPOINT_MIN_MARGIN == 0.35
    csv = line('porto_mesh_cr_dk0.8')
    if not os.path.exists(csv):
        pytest.skip('line not kept')
    cps = os.path.join(REPO, 'data', 'maps', 'porto_mesh_checkpoints.csv')
    geo = pick_config.checkpoint_geometry(csv, cps)
    assert geo['checkpoint_margin_min'] > pick_config.CHECKPOINT_MIN_MARGIN
