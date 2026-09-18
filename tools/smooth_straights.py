#!/usr/bin/env python3
"""Flatten the STRAIGHTS of a finished racing line, leaving the corners alone.

Why this exists
---------------
On the finals circuit the generated lines reverse curvature sign 3.5-6.3 times
per 10 m wherever |kappa| < 0.3, against Porto's 2.4, and the user sees the car
making rapid small left-right corrections on a road that looks straight.  The
question this tool was built to answer is whether that comes from the LINE.

It is a post-pass, not a change to the QP, because the QP's own smoothing
(`--alpha-smooth`) only touches the min-curvature candidates' lateral shift:
when every one of those is rejected on clearance the drill falls back to the
safe centerline, which `--alpha-smooth` never sees.  This runs on whatever line
actually ships.

What it does
------------
Takes the finished line as a reference, parameterises a lateral offset n(s)
against it (positive left), and solves, on the STRAIGHT segments only,

    min  || d/ds (kappa_ref - n'') ||^2 + || (kappa_ref - n'')/L ||^2
         + lam * || n ||^2                       s.t.  lo <= n <= hi

i.e. flatten the curvature RATE -- the quantity the straight-noise metric
reports -- without leaving a large constant bend behind (the second term; a
constant curvature has zero rate, and would otherwise be free).  n is pinned to
0, value and slope, at both ends of every segment, so the corners come back
untouched and the joins are C1.  The bounds come from the map's own distance
field and are tightened iteratively until the smoothed line's worst BODY-corner
clearance is at least what the input line already held -- clearance is never
spent to buy smoothness.  The speed profile is then recomputed on the new
geometry with the same envelope `make_track_lines.PROFILE` uses.

What it measured
----------------
Nothing to remove.  On `finals_mesh_race.csv` the optimal move is **1 mm**: the
18.6 m main straight is already flat to |kappa| <= 0.006 1/m, and every large
|dkappa/ds| inside the |kappa| < 0.3 mask is a genuine corner-entry ramp.  The
sign reversals the metric counts are sign flips of a curvature of ~0.003 1/m,
i.e. the %.4f rounding of the CSV.  See docs/FINALS_TUNING.md.

Usage:
  .venv/bin/python tools/smooth_straights.py data/racelines/finals_mesh_race.csv \
      --map data/maps/finals_mesh --out data/racelines/finals_mesh_smooth_post
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from footprint_analysis import load_dist, clearance, CORNERS    # noqa: E402

# Length over which a curvature of 1/CHAR_LEN counts as much as a curvature
# RATE of 1/CHAR_LEN per metre.  1 m is the scale of this car's corners.
CHAR_LEN = 1.0


def body_clearance(dist, res, ox, oy, xy):
    """Worst body-corner clearance at every point of a closed line."""
    d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    head = np.arctan2(d[:, 1], d[:, 0])
    worst = np.full(len(xy), np.inf)
    for cx, cy in CORNERS:
        px = xy[:, 0] + cx * np.cos(head) - cy * np.sin(head)
        py = xy[:, 1] + cx * np.sin(head) + cy * np.cos(head)
        worst = np.minimum(worst, np.nan_to_num(
            clearance(dist, res, ox, oy, np.column_stack([px, py])),
            nan=np.inf))
    return worst


def segments(kappa, s, thr, min_len):
    """Closed-loop runs of |kappa| < thr that are at least min_len metres."""
    n = len(s)
    m = np.abs(kappa) < thr
    if m.all():
        return [(0, n - 1)]
    out = []
    start = None
    for i in range(n):
        if m[i] and start is None:
            start = i
        elif not m[i] and start is not None:
            if s[i - 1] - s[start] >= min_len:
                out.append((start, i - 1))
            start = None
    if start is not None and s[n - 1] - s[start] >= min_len:
        out.append((start, n - 1))
    return out


def _diff_matrix(k, order, ds):
    """Finite-difference operator of `order` on k samples spaced ds apart."""
    D = np.eye(k)
    for _ in range(order):
        D = np.diff(D, axis=0) / ds
    return D


def solve_segment(kappa, ds, lo, hi, lam, objective='dkappa'):
    """Bounded least squares for the lateral offset over one straight.

    For a small offset n(s) against the reference, the new curvature is
    kappa_ref - n''.  Two objectives:

    * `kappa`  drives the curvature itself to zero -- that straightens a
      genuinely curved section, which is re-optimising the line rather than
      smoothing it, and on this track it costs lap time;
    * `dkappa` (default) drives the curvature RATE to zero, with a scaled share
      of the curvature itself kept in the objective so the solver cannot leave
      a large constant bend behind (a constant has zero rate, so it is free).

    Unknowns are the interior offsets; the two points at each end are pinned to
    zero, which fixes both the value and the slope of n there, so the join with
    the corner is C1.
    """
    from scipy.optimize import lsq_linear
    k = len(kappa)
    if k < 9:
        return np.zeros(k)
    free = np.arange(2, k - 2)
    # new curvature at the interior points 1..k-2 is kappa[1:-1] - D2 @ n
    D2 = _diff_matrix(k, 2, ds)
    A0, b0 = D2[:, free], kappa[1:k - 1].copy()
    if objective == 'kappa':
        A, b = A0, b0
    else:
        E = _diff_matrix(k - 2, 1, ds)
        A = np.vstack([E @ A0, A0 / CHAR_LEN])
        b = np.concatenate([E @ b0, b0 / CHAR_LEN])
    # the ridge is relative to the smoothing term, so `lam` means the same
    # thing whatever the point spacing is
    scale = np.sqrt((A ** 2).sum() / max(len(free), 1))
    A = np.vstack([A, np.sqrt(lam) * scale * np.eye(len(free))])
    b = np.concatenate([b, np.zeros(len(free))])
    res = lsq_linear(A, b, bounds=(lo[free], hi[free]), max_iter=200, tol=1e-10)
    out = np.zeros(k)
    out[free] = res.x
    return out


def smooth(line, map_prefix, kappa_straight=0.30, min_len=2.0, lam=1e-3,
           max_shift=0.25, iters=6, objective='dkappa', verbose=True):
    """Returns (new xy, the input array, the straight segments, the offset n)."""
    r = np.loadtxt(line, delimiter=',')
    s, xy, kappa = r[:, 0], r[:, 1:3].copy(), r[:, 4].copy()
    dist, res, ox, oy = load_dist(map_prefix)
    base = body_clearance(dist, res, ox, oy, xy)
    floor = float(base.min())

    d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    tl = np.hypot(d[:, 0], d[:, 1])
    nx, ny = -d[:, 1] / tl, d[:, 0] / tl          # left normal
    ds = float(np.median(tl))

    segs = segments(kappa, s, kappa_straight, min_len)
    if verbose:
        print(f'{os.path.basename(line)}: {len(segs)} straight segment(s) '
              f'at |kappa| < {kappa_straight:g}, '
              f'{sum(s[b] - s[a] for a, b in segs):.1f} m of {s[-1]:.1f} m; '
              f'body-clearance floor {floor:.3f} m')

    n = np.zeros(len(s))
    slack = np.full(len(s), max_shift)
    new = xy
    for it in range(iters):
        n[:] = 0.0
        for a, b in segs:
            idx = np.arange(a, b + 1)
            n[idx] = solve_segment(kappa[idx], ds, -slack[idx], slack[idx],
                                   lam, objective)
        new = xy + np.column_stack([n * nx, n * ny])
        clr = body_clearance(dist, res, ox, oy, new)
        bad = clr < floor - 1e-4
        if not bad.any():
            break
        # shrink the bound at and around every violating point
        w = np.convolve(np.where(bad, floor - clr, 0.0), np.ones(5), 'same')
        slack = np.maximum(slack - np.maximum(w, 0.0) - 0.005, 0.0)
        if verbose:
            print(f'  pass {it + 1}: {int(bad.sum())} point(s) under the '
                  f'{floor:.3f} m floor, tightening the corridor bound')
    clr = body_clearance(dist, res, ox, oy, new)
    if verbose:
        print(f'  max lateral shift {np.abs(n).max():.4f} m, '
              f'rms {np.sqrt((n ** 2).mean()):.4f} m; body clearance '
              f'{base.min():.3f} -> {clr.min():.3f} m')
    return new, r, segs, n


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('line', help='finished 7-column raceline CSV')
    ap.add_argument('--map', required=True, metavar='PREFIX',
                    help='map stem, e.g. data/maps/finals_mesh')
    ap.add_argument('--out', required=True, metavar='STEM')
    ap.add_argument('--kappa-straight', type=float, default=0.30, metavar='K',
                    help='a point is on a "straight" when |kappa| < K '
                         '(default 0.30, the threshold the straight-noise '
                         'metric uses). Raising it reaches into the '
                         'corner-entry ramps.')
    ap.add_argument('--min-length', type=float, default=2.0, metavar='M',
                    help='ignore straight runs shorter than this (default 2 m)')
    ap.add_argument('--objective', default='dkappa',
                    choices=('dkappa', 'kappa'),
                    help='what to drive to zero on the straights: the '
                         'curvature RATE (default) or the curvature itself')
    ap.add_argument('--lam', type=float, default=1e-3,
                    help='ridge weight holding the line near its original '
                         'position, relative to the smoothing term '
                         '(default 1e-3)')
    ap.add_argument('--max-shift', type=float, default=0.25, metavar='M',
                    help='hard cap on the lateral move, before the corridor '
                         'constraint tightens it further (default 0.25 m)')
    ap.add_argument('--ay', type=float, default=6.0)
    ap.add_argument('--ax', type=float, default=6.5)
    ap.add_argument('--brake', type=float, default=4.0)
    ap.add_argument('--jerk-lat', type=float, default=40.0)
    ap.add_argument('--ay-tight', type=float, default=4.5)
    ap.add_argument('--kappa-tight', type=float, default=1.15)
    ap.add_argument('--vmax', type=float, default=8.0)
    a = ap.parse_args()

    new, old, _segs, n = smooth(a.line, a.map, a.kappa_straight, a.min_length,
                                a.lam, a.max_shift, objective=a.objective)

    # Geometry of the smoothed line, as a PERTURBATION of the input's own.
    # Re-estimating psi and kappa from scratch would swap the input file's
    # curvature estimator for this one, and the two disagree by rms 0.04 1/m on
    # a corner neither touched -- enough noise to crater the velocity profile
    # and make a 1 mm move look like a second of lap time.  To first order in
    # the offset, psi_new = psi + n' and kappa_new = kappa - n''.
    d = np.diff(np.vstack([new, new[:1]]), axis=0)
    el = np.hypot(d[:, 0], d[:, 1])
    s = np.concatenate([[0.0], np.cumsum(el)])[:len(new)]
    ds0 = float(np.median(np.hypot(*np.diff(
        np.vstack([old[:, 1:3], old[:1, 1:3]]), axis=0).T)))
    psi = old[:, 3] + (np.roll(n, -1) - np.roll(n, 1)) / (2.0 * ds0)
    kappa = old[:, 4] - (np.roll(n, -1) - 2.0 * n + np.roll(n, 1)) / ds0 ** 2

    import trajectory_planning_helpers as tph
    from optimize_raceline import (VEHICLE_MASS, limit_lat_jerk,
                                   limit_long_accel)
    v_steps = np.arange(0.0, a.vmax + 0.5, 0.5)
    ggv = np.column_stack([v_steps, np.full_like(v_steps, a.ax),
                           np.full_like(v_steps, a.ay)])
    axm = np.column_stack([v_steps, np.full_like(v_steps, a.ax)])
    vx = tph.calc_vel_profile.calc_vel_profile(
        ggv=ggv, ax_max_machines=axm, v_max=a.vmax, kappa=kappa,
        el_lengths=el, closed=True, drag_coeff=0.0, m_veh=VEHICLE_MASS)
    if a.ay_tight is not None and a.ay_tight < a.ay:
        k = np.abs(kappa)
        k0, k1 = a.kappa_tight, 1.5 * a.kappa_tight
        frac = np.clip((k - k0) / max(k1 - k0, 1e-6), 0.0, 1.0)
        v_cap = np.sqrt((a.ay + (a.ay_tight - a.ay) * frac)
                        / np.maximum(k, 1e-6))
        if np.any(v_cap < vx - 1e-6):
            vx = limit_long_accel(np.minimum(vx, v_cap), el, a.ax, a.brake)
    if a.jerk_lat > 0.0:
        vx = limit_lat_jerk(vx, kappa, el, a.jerk_lat, a.ax, a.brake)
        vx = limit_long_accel(vx, el, a.ax, a.brake)
    ax_prof = tph.calc_ax_profile.calc_ax_profile(
        vx_profile=np.append(vx, vx[0]), el_lengths=el, eq_length_output=False)

    np.savetxt(a.out + '.csv',
               np.column_stack([s, new[:, 0], new[:, 1], psi, kappa, vx,
                                ax_prof[:len(s)]]),
               delimiter=',', fmt='%.4f',
               header='s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2',
               comments='# ')
    lap = float(np.sum(el / np.maximum(vx, 0.1)))
    old_lap = float(np.sum(np.diff(old[:, 0]) / np.maximum(old[:-1, 5], 0.1)))
    print(f'wrote {a.out}.csv -- ideal lap {old_lap:.2f} -> {lap:.2f} s, '
          f'|kappa| max {np.abs(kappa).max():.2f} 1/m')


if __name__ == '__main__':
    main()
