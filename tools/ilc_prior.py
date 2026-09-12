#!/usr/bin/env python3
"""Build the ILC steering-feedforward table OFFLINE, from our own race logs.

Why this exists
---------------
``mpc3_core.ILC`` learns a curvature feedforward over arc length, one update per
completed lap.  That is the right law and it converges -- but a scored race is
one warm-up lap followed by ten counted ones, so the online loop spends the part
of the race that is scored still learning.  The error it learns is *repeatable*
(``tools/lap_report.py`` measures how much: it is the lap-mean offset curve over
s), so it does not have to be re-learned at all.  It can be measured from logs we
already have and handed to the node as a converged table:

    -p ilc_enable:=true -p ilc_store:=<prefix>

``mpc3.py`` appends ``_<raceline basename>.npz`` to that prefix, so the file this
tool writes must be named accordingly -- ``--out`` is checked against the line
and the tool says the exact path the node will open.

What the table is
-----------------
``ILC.finish_lap`` does, per lap::

    kff <- clip(Q[(1 - leak) * kff - k_gain * mean_e], -k_clamp, k_clamp)

with ``mean_e`` the mean signed lateral error per s bin (+ = left of travel) and
``Q`` a circular box average.  The sign is the whole content of the law: a lap
spent LEFT of the line is corrected by a NEGATIVE curvature offset, i.e. by
steering right.

The fixed point.  Linearise the plant over one lap as ``e = e0 + G kff`` (adding
left curvature moves the car left, so ``G > 0``).  Substituting into the update
and solving ``kff = (1 - leak) kff - k_gain (e0 + G kff)`` gives

    kff* = -k_gain e0 / (leak + k_gain G)  ->  -e0 / G   as leak -> 0,

so the converged table is proportional to the measured mean error with the
code's sign, and the constant is the INVERSE PLANT GAIN -- not the learning
rate, which cancels.  ``G`` is the same quantity ``ILC``'s own clamp is built
from: a constant curvature ``k`` held over ``shift_len_m`` of preview shifts the
car ``k * L^2 / 2``, so ``G = L^2 / 2`` and

    kff = -(2 / shift_len_m^2) * mean_e          (default gain 0.889 1/m per m)

which is why the default gain and the clamp agree: a bin whose mean offset is
exactly ``shift_max_m`` (0.08 m) lands exactly on ``k_clamp``.  ``--gain``
overrides it; a larger gain is a claim that the car responds to feedforward less
than the geometry says, and should be justified by driving it.

The leak-only fixed point ``-(k_gain / leak) e0 = -15 e0`` is what the recursion
converges to if the plant does NOT respond at all; at our measured errors it
pins every bin at the clamp, which is the documented full-clamp failure.  It is
available as ``--gain 15`` and is not the default for that reason.

Usage
-----
    .venv/bin/python tools/ilc_prior.py \
        data/racelines/porto_mesh_mc0.63_ay6.75.csv \
        data/logs/race_A.csv data/logs/race_B.csv \
        --out /tmp/ilc/prior_porto_mesh_mc0.63_ay6.75.npz

Laps that are not evidence are dropped, by the same tests the online loop uses:
a lap containing a collision, samples within 1.5 s of one (the simulator
teleports the car), a lap whose bin coverage is under ``--min-cover`` of what
its sample count could reach, and a lap with a circular unseen run longer than
``--max-gap`` of the lap (a fragment).  The remaining per-lap curves are
averaged with equal weight per lap, across every log.
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src'))

from pit_crew.mpc3_core import ILC, _max_circular_gap          # noqa: E402

TELEPORT_S = 1.5        # a collision moves the car; this much either side is junk
MIN_SPEED = 0.3         # below this the car is stopped, not tracking


# ---------------------------------------------------------------- geometry
def load_line(path):
    """``(s, xy, yaw_of_travel)`` for a raceline CSV."""
    rl = np.loadtxt(path, delimiter=',')
    xy = rl[:, 1:3]
    fwd = np.roll(xy, -1, 0) - np.roll(xy, 1, 0)
    return rl[:, 0], xy, np.arctan2(fwd[:, 1], fwd[:, 0])


def total_s(s):
    """The lap length ``mpc3`` hands ``ILC``: ``s[-1] + spacing``."""
    d = np.diff(s)
    spacing = float(np.median(d)) if len(d) else 0.1
    return float(s[-1] + spacing)


def project(xy, x, y):
    """Nearest line index per sample, chunked (same rule as lap_report)."""
    idx = np.empty(len(x), int)
    for a in range(0, len(x), 2000):
        b = min(a + 2000, len(x))
        d = np.hypot(x[a:b, None] - xy[None, :, 0], y[a:b, None] - xy[None, :, 1])
        idx[a:b] = d.argmin(1)
    return idx


def signed_offset(xy, yaw, x, y, idx):
    """Lateral offset of the car from the line, + = LEFT of travel.

    Identical to ``tools/lap_report.signed_offset`` and to the ``e_lat`` the
    tracker feeds ``ILC.observe`` (``mpc3.py``: ``-sin(pyaw) dx + cos(pyaw) dy``).
    """
    return -np.sin(yaw[idx]) * (x - xy[idx, 0]) + np.cos(yaw[idx]) * (y - xy[idx, 1])


# ---------------------------------------------------------------- log input
def load_log(path, pose='ips'):
    """Log columns as arrays, with the logger's duplicated IPS rows dropped.

    ``pose`` picks which position the offset is measured from.  ``ips`` is the
    simulator's truth, which is what we actually want the car to do; ``pf`` is
    the localization estimate the tracker steered on, i.e. what the ONLINE ILC
    would have learned.  They differ by the localization error, so a prior built
    from ``ips`` also corrects a repeatable localization bias and one built from
    ``pf`` reproduces the online loop exactly.
    """
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise ValueError(f'{path}: empty log')
    col = lambda k: np.array([float(r[k]) for r in rows])        # noqa: E731
    t, x, y = col('t'), col('x'), col('y')
    keep = np.ones(len(t), bool)
    keep[1:] = (np.diff(x) != 0) | (np.diff(y) != 0)
    if pose == 'pf':
        if 'px' not in rows[0]:
            raise ValueError(f'{path}: no px/py column for --pose pf')
        px, py = col('px'), col('py')
        keep &= np.isfinite(px) & np.isfinite(py)
        x, y = px, py
    out = dict(t=t, x=x, y=y, lap=col('lap_count').astype(int),
               coll=col('collision_count').astype(int))
    return {k: v[keep] for k, v in out.items()}


def true_speed(t, x, y):
    """Speed from the IPS pose -- the encoder channel is a filtered estimate."""
    if len(t) < 2:
        return np.zeros(len(t))
    v = np.hypot(np.diff(x), np.diff(y)) / np.maximum(np.diff(t), 1e-3)
    return np.concatenate([[v[0]], v])


# ---------------------------------------------------------------- per lap
def lap_curves(log, s_line, xy, yaw, nb, bin_m, min_cover, max_gap,
               fill, verbose=True, name=''):
    """Mean signed offset per s bin, one curve per usable lap.

    Returns ``(curves, stats)``; ``curves`` is a list of ``nb``-vectors.
    """
    idx = project(xy, log['x'], log['y'])
    off = signed_offset(xy, yaw, log['x'], log['y'], idx)
    sj = s_line[idx]
    b_all = (sj / bin_m).astype(int) % nb
    v = true_speed(log['t'], log['x'], log['y'])

    bad = np.zeros(len(sj), bool)
    for i in np.flatnonzero(np.diff(log['coll'])) + 1:
        bad |= np.abs(log['t'] - log['t'][i]) < TELEPORT_S
    ok = (~bad) & (v > MIN_SPEED)

    curves = []
    st = dict(laps_seen=0, dropped_collision=0, dropped_cover=0,
              dropped_gap=0, used=0)
    for lap in np.unique(log['lap']):
        m = log['lap'] == lap
        if m.sum() < 5:
            continue
        st['laps_seen'] += 1
        # A lap that contains a collision is not repeatable evidence about the
        # line: the car was somewhere else and was then teleported back.
        if log['coll'][m].max() != log['coll'][m].min() or bad[m].any():
            st['dropped_collision'] += 1
            continue
        g = m & ok
        if g.sum() < 5:
            st['dropped_cover'] += 1
            continue
        sum_e = np.zeros(nb)
        cnt = np.zeros(nb)
        np.add.at(sum_e, b_all[g], off[g])
        np.add.at(cnt, b_all[g], 1.0)
        seen = cnt >= 1.0
        # The ONLINE gates, verbatim (ILC.finish_lap): coverage measured
        # against what the sample count could reach, and the largest circular
        # run of unseen bins to catch a fragment.
        reach = min(float(nb), float(cnt.sum()))
        if seen.sum() < min_cover * reach:
            st['dropped_cover'] += 1
            continue
        if _max_circular_gap(seen) > max_gap * nb:
            st['dropped_gap'] += 1
            continue
        mean_e = fill(np.where(seen, sum_e / np.maximum(cnt, 1.0), 0.0), seen)
        curves.append(mean_e)
        st['used'] += 1
    if verbose:
        print(f'  {name or "log"}: {st["laps_seen"]} lap groups, '
              f'{st["used"]} used, dropped {st["dropped_collision"]} for a '
              f'collision, {st["dropped_cover"]} for coverage, '
              f'{st["dropped_gap"]} for a gap')
    return curves, st


# ---------------------------------------------------------------- filters
def gaussian_circular(a, sigma_bins):
    """Circular Gaussian smoothing, in bins.  Identity for ``sigma <= 0``."""
    if sigma_bins <= 0.0:
        return np.asarray(a, float)
    half = max(1, int(math.ceil(3.0 * sigma_bins)))
    k = np.exp(-0.5 * (np.arange(-half, half + 1) / sigma_bins) ** 2)
    k /= k.sum()
    pad = np.concatenate([a[-half:], a, a[:half]])
    return np.convolve(pad, k, mode='valid')


def build_table(curves, ilc, sigma_m=0.6, gain=None):
    """Mean curve -> the converged ``kappa_ff`` table.

    ``ilc`` supplies ``nb``, ``bin_m``, the Q filter and ``k_clamp``, so the
    prior is filtered and clamped by exactly the code that would have produced
    it online.
    """
    if not curves:
        raise ValueError('no usable laps in any log')
    mean_e = np.mean(np.vstack(curves), axis=0)
    if gain is None:
        gain = default_gain(ilc)
    e_s = gaussian_circular(mean_e, sigma_m / ilc.bin_m)
    kff = np.clip(ilc._q_filter(-float(gain) * e_s), -ilc.k_clamp, ilc.k_clamp)
    return kff, mean_e, e_s, float(gain)


def default_gain(ilc, shift_len_m=1.50):
    """Inverse plant gain ``2 / L^2``: see the module docstring's fixed point."""
    return 2.0 / max(float(shift_len_m), 1e-3) ** 2


# ---------------------------------------------------------------- reporting
def window_mean(a, bin_m, lo, hi):
    """Mean of a per-bin quantity over the arc window ``[lo, hi)``."""
    b = np.arange(len(a)) * bin_m
    m = (b >= lo) & (b < hi)
    return float(a[m].mean()) if m.any() else float('nan')


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('line', help='the raceline CSV the logs were driven on')
    ap.add_argument('logs', nargs='+', help='race log CSVs')
    ap.add_argument('--out', required=True, help='npz to write (ILC.load format)')
    ap.add_argument('--bin', type=float, default=0.25, dest='bin_m',
                    help='arc bin width, m (must match ilc_bin_m)')
    ap.add_argument('--gain', type=float, default=None,
                    help='1/m of feedforward per metre of mean offset '
                         '(default 2/shift_len_m^2 = 0.889)')
    ap.add_argument('--sigma_m', '--sigma-m', type=float, default=0.6,
                    dest='sigma_m',
                    help='extra circular Gaussian smoothing of the mean offset '
                         'curve, m (0 disables); the Q filter is applied on top')
    ap.add_argument('--clamp', type=float, default=None,
                    help="|kappa_ff| cap, 1/m (default ILC's own k_clamp)")
    ap.add_argument('--q-span-m', type=float, default=0.50)
    ap.add_argument('--k-max', type=float, default=0.15)
    ap.add_argument('--shift-max-m', type=float, default=0.08)
    ap.add_argument('--shift-len-m', type=float, default=1.50)
    ap.add_argument('--min-cover', type=float, default=0.60)
    ap.add_argument('--max-gap', type=float, default=0.25)
    ap.add_argument('--laps', type=int, default=0,
                    help='value to write in the npz (0 = the number of laps '
                         'averaged); must be > 0 so the table counts as trained')
    ap.add_argument('--pose', choices=('ips', 'pf'), default='ips',
                    help='ips = simulator truth (default), pf = the pose the '
                         'tracker steered on')
    ap.add_argument('--windows', default='10:12,24:26',
                    help='arc windows to report the mean offset over')
    args = ap.parse_args(argv)

    s_line, xy, yaw = load_line(args.line)
    L = total_s(s_line)
    ilc = ILC(L, bin_m=args.bin_m, q_span_m=args.q_span_m, k_max=args.k_max,
              shift_max_m=args.shift_max_m, shift_len_m=args.shift_len_m)
    if args.clamp is not None:
        ilc.k_clamp = float(args.clamp)
    print(f'{os.path.basename(args.line)}: {L:.3f} m, {ilc.nb} bins of '
          f'{ilc.bin_m:.2f} m, |kappa_ff| <= {ilc.k_clamp:.4f} 1/m')

    curves = []
    for path in args.logs:
        c, _st = lap_curves(load_log(path, args.pose), s_line, xy, yaw, ilc.nb, ilc.bin_m,
                            args.min_cover, args.max_gap, ilc._fill,
                            name=os.path.basename(path))
        curves.extend(c)
    kff, mean_e, e_s, gain = build_table(curves, ilc, sigma_m=args.sigma_m,
                                         gain=args.gain)

    laps = int(args.laps) if args.laps > 0 else len(curves)
    if laps <= 0:
        raise SystemExit('no laps to write')
    ilc.kappa_ff = kff
    ilc.v_scale = np.ones(ilc.nb)        # speed table untouched: ilc_v_enable=0
    ilc.laps = laps
    ilc.store = args.out
    ilc.save()
    out = ilc.store_path()

    k = int(np.argmax(np.abs(kff)))
    print(f'\n{len(curves)} laps averaged, gain {gain:.3f} 1/m per m, '
          f'sigma {args.sigma_m:.2f} m')
    print(f'mean |offset| {np.abs(mean_e).mean():.4f} m, '
          f'max {np.abs(mean_e).max():.4f} m at s '
          f'{ilc.bin_m * int(np.argmax(np.abs(mean_e))):.2f} m')
    print(f'max |kappa_ff| {np.abs(kff).max():.4f} 1/m at s '
          f'{ilc.bin_m * k:.2f} m (clamp {ilc.k_clamp:.4f}; '
          f'{int((np.abs(kff) >= ilc.k_clamp - 1e-12).sum())} of {ilc.nb} bins '
          f'clamped)')
    order = np.argsort(-np.abs(kff))[:8]
    print('largest bins (s, mean offset + = left, kappa_ff + = left turn):')
    for j in sorted(order, key=lambda j: j * ilc.bin_m):
        print(f'  s {ilc.bin_m * j:6.2f}  offset {mean_e[j]:+.4f} m  '
              f'kappa_ff {kff[j]:+.4f} 1/m')
    for w in args.windows.split(','):
        if not w:
            continue
        lo, hi = (float(v) for v in w.split(':'))
        print(f's {lo:g}-{hi:g}: mean offset {window_mean(mean_e, ilc.bin_m, lo, hi):+.4f} m'
              f', mean kappa_ff {window_mean(kff, ilc.bin_m, lo, hi):+.4f} 1/m')

    base = os.path.splitext(os.path.basename(args.line))[0]
    print(f'\nwrote {out}  (laps={laps})')
    expect = f'_{base}.npz'
    if not out.endswith(expect):
        print(f'NOTE: mpc3.py opens "<ilc_store>_{base}.npz". Rename this file '
              f'or pass -p ilc_store:={out[:-4]} only if the line differs; for '
              f'this line the node wants a path ending in "{expect}".')
    else:
        print(f'live:  -p ilc_enable:=true -p ilc_store:={out[:-len(expect)]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
