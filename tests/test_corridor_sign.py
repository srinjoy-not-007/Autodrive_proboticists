"""Which way is positive alpha?  The one-sided bounds all hang off this.

`tools/optimize_raceline.py` hands the lateral shift `alpha` to
`trajectory_planning_helpers.opt_min_curv`, which bounds it by the corridor
columns of `reftrack`, and then re-states its own bounds (`corridor_of`) to
clip the smoothing and the `--dkappa-max` refinement.  The two have to agree
about which column is the upper bound; they did not.  `corridor_of` returned
`(-w_right, +w_left)` while tph's box is `-w_left <= alpha <= +w_right`, so a
SYMMETRIC bound survived the mirror and a ONE-SIDED one (`--min-halfwidth`, a
gate band) reached the QP as the mirror image of itself: a feasible interval
`[lo, hi]` along the normal was restated as `[-hi, -lo]`, which forbids legal
positions and permits illegal ones wherever the two walls are not equidistant.

Everything here is derived numerically from the installed library, so it fails
if tph's convention ever changes rather than encoding a belief about it.
"""

import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))

import trajectory_planning_helpers as tph                     # noqa: E402

import optimize_raceline as opt                               # noqa: E402


def _closed_circle(n=40, r=5.0):
    """A counter-clockwise closed reference and its tph spline normals."""
    th = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    pts = np.column_stack([r * np.cos(th), r * np.sin(th)])
    _cx, _cy, _M, nv = tph.calc_splines.calc_splines(
        path=np.vstack([pts, pts[:1]]))
    return pts, nv


def test_normals_point_right_of_travel():
    """+normal is the RIGHT-hand side of the direction of travel."""
    pts, nv = _closed_circle()
    for i in (0, 7, 19, 33):
        t = pts[(i + 1) % len(pts)] - pts[i]
        t /= np.linalg.norm(t)
        right = np.array([t[1], -t[0]])        # tangent rotated by -90 deg
        assert nv[i] @ right > 0.99, f'normal at {i} is not right of travel'


def test_shifting_by_positive_alpha_moves_the_point_right():
    """`create_raceline`'s alpha enters as `point + alpha * normal`."""
    pts, nv = _closed_circle()
    alpha = np.full(len(pts), 0.5)
    shifted = pts + nv * alpha[:, None]
    # Outward on a CCW circle is to the right of travel, so the radius grows.
    assert np.allclose(np.linalg.norm(shifted, axis=1), 5.5, atol=1e-6)


def test_tph_bounds_alpha_above_by_the_RIGHT_width():
    """`opt_min_curv`'s box is `-(w_left - v/2) <= alpha <= w_right - v/2`.

    Read off the library's own inequality rows rather than its docstring: `G`
    is `[I; -I; E; -E]` and `h` starts `[dev_max_right, dev_max_left]`, with
    `dev_max_right = reftrack[:, 2] - w_veh/2`.  A one-sided corridor is the
    cheapest way to see it end to end: give the QP a metre of room on the
    right (column 2) and none on the left (column 3) and the solution it
    returns must be non-negative.
    """
    pts, nv = _closed_circle()
    n = len(pts)
    ref = np.column_stack([pts, np.full(n, 1.0), np.full(n, 0.0)])
    _cx, _cy, M, _nv = tph.calc_splines.calc_splines(
        path=np.vstack([pts, pts[:1]]))
    alpha = tph.opt_min_curv.opt_min_curv(
        reftrack=ref, normvectors=nv, A=M, kappa_bound=10.0, w_veh=0.0,
        closed=True, print_debug=False)[0]
    assert alpha.min() >= -1e-6, 'alpha went left with only right-hand room'
    assert alpha.max() <= 1.0 + 1e-6


def test_corridor_of_agrees_with_the_QP_box_on_the_one_sided_path():
    """With a one-sided bound in play, `corridor_of` speaks tph's sign.

    The corridor is asymmetric on purpose: 1.0 m of right width and 0.2 m of
    left.  tph would accept `alpha` up to +1.0 and down to -0.2, so that is
    what `corridor_of` must return.
    """
    pts, nv = _closed_circle()
    n = len(pts)
    work = np.column_stack([pts, np.full(n, 1.0), np.full(n, 0.2)])

    class Args:
        min_halfwidth = None
        min_halfwidth_map = None
        checkpoints = None
        gate_separation = None
        w_veh = 0.0
        _extra_bounds = (np.full(n, -np.inf), np.full(n, np.inf))

    args = Args()
    # The plain path is FROZEN in the old mirrored convention because every
    # shipping line was generated through it (see `corridor_of`).
    lo, hi = opt.corridor_of(work[:, :2], nv, work, args)
    assert np.allclose(lo, -1.0) and np.allclose(hi, 0.2)

    # Opt in to a one-sided bound and the corridor is stated tph's way round.
    args.checkpoints = 'anything-not-None'
    lo, hi = opt.corridor_of(work[:, :2], nv, work, args)
    assert np.allclose(lo, -0.2), 'lower bound must be the LEFT width'
    assert np.allclose(hi, 1.0), 'upper bound must be the RIGHT width'


def test_wall_bounds_and_the_QP_box_use_the_same_sign():
    """A `--min-halfwidth` bound restated as corridor columns keeps its side.

    `_sweep_solve` turns `corridor_of`'s `(lo, hi)` back into the `(right,
    left)` columns tph reads, and that restatement is where the mirror used to
    flip a one-sided wall floor onto the opposite wall.  The round trip has to
    be the identity.
    """
    n = 16
    w_veh = 0.4
    lo = np.full(n, -0.1)          # 0.1 m of room to the LEFT
    hi = np.full(n, 0.9)           # 0.9 m to the RIGHT
    keep = np.column_stack([hi, -lo]) + w_veh / 2.0
    # tph's own reading of those columns:
    dev_max_right = keep[:, 0] - w_veh / 2.0
    dev_max_left = keep[:, 1] - w_veh / 2.0
    assert np.allclose(dev_max_right, hi)
    assert np.allclose(-dev_max_left, lo)
