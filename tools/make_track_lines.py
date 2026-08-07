#!/usr/bin/env python3
"""Generate a track's racing lines, choosing the centerline method by measurement.

Neither centerline method wins everywhere: the skeleton is better on a simple
loop (Porto worst-corner clearance 0.325 m vs 0.079 m) while on a circuit with
an inner passage it cuts straight across the infield and is unusable, where the
trajectory-anchored method works (ICRA 0.215 m). Rather than pick by hand per
track — which is exactly the sort of choice that does not survive an unseen
track — build both and keep whichever actually measures better.

Usage: .venv/bin/python tools/make_track_lines.py <track_name>
"""
import subprocess
import sys

sys.path.insert(0, 'tools')
from footprint_analysis import load_dist, clearance, CORNERS   # noqa: E402

import numpy as np   # noqa: E402

VENV = '.venv/bin/python'


def worst_corner(map_prefix, line_csv):
    dist, res, ox, oy = load_dist(map_prefix)
    rl = np.loadtxt(line_csv, delimiter=',')
    xy = rl[:, 1:3]
    d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    head = np.arctan2(d[:, 1], d[:, 0])
    worst = np.full(len(xy), np.inf)
    for cx, cy in CORNERS:
        px = xy[:, 0] + cx * np.cos(head) - cy * np.sin(head)
        py = xy[:, 1] + cx * np.sin(head) + cy * np.cos(head)
        worst = np.minimum(worst, np.nan_to_num(
            clearance(dist, res, ox, oy, np.column_stack([px, py])), nan=np.inf))
    return float(worst.min())


MIN_TURN_R = 0.561      # wheelbase / tan(max steer)


def drivability(line_csv):
    """Reasons the car physically cannot follow this line, if any.

    Clearance alone is not enough: the ICRA line measured 0.414 m of corner
    clearance and was still undrivable, because a skeleton spur made it double
    back on itself at one point. Anything that reverses direction or turns
    tighter than the steering lock is disqualifying, however roomy it looks.
    """
    rl = np.loadtxt(line_csv, delimiter=',')
    xy = rl[:, 1:3]
    d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    head = np.arctan2(d[:, 1], d[:, 0])
    dh = np.abs((np.diff(np.concatenate([head, head[:1]])) + np.pi)
                % (2 * np.pi) - np.pi)
    bad = []
    n_rev = int((dh > 2.0).sum())
    if n_rev:
        bad.append(f'{n_rev} direction reversal(s)')
    kmax = float(np.abs(rl[:, 4]).max())
    if kmax > 1.0 / MIN_TURN_R:
        bad.append(f'min radius {1 / kmax:.3f} m < {MIN_TURN_R:.3f} m lock')
    return bad


def main():
    track = sys.argv[1]
    mp = f'data/maps/{track}'
    results = {}
    for tag, extra in (('skeleton', ['--skeleton']), ('anchored', [])):
        cl = f'data/racelines/{track}_centerline_{tag}'
        safe = f'data/racelines/{track}_safe_{tag}'
        r = subprocess.run([VENV, 'tools/extract_centerline.py', mp, cl] + extra,
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f'  {tag}: centerline extraction failed')
            continue
        r = subprocess.run([VENV, 'tools/optimize_raceline.py', cl + '.csv',
                            '--centerline-only', '--out', safe],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f'  {tag}: line generation failed')
            continue
        try:
            wc = worst_corner(mp, safe + '.csv')
        except Exception as e:
            print(f'  {tag}: clearance check failed ({e})')
            continue
        bad = drivability(safe + '.csv')
        if bad:
            print(f'  {tag:9s} REJECTED: {"; ".join(bad)}')
            continue
        results[tag] = (wc, cl, safe)
        print(f'  {tag:9s} worst-corner clearance {wc:.3f} m')

    if not results:
        sys.exit('no drivable line produced by either method')
    best = max(results, key=lambda k: results[k][0])
    wc, cl, safe = results[best]
    print(f'chosen: {best} ({wc:.3f} m)')
    for src, dst in ((cl + '.csv', f'data/racelines/{track}_centerline.csv'),
                     (safe + '.csv', f'data/racelines/{track}_safe.csv')):
        subprocess.run(['cp', src, dst], check=True)
    # optimized raceline from the chosen centerline
    subprocess.run([VENV, 'tools/optimize_raceline.py',
                    f'data/racelines/{track}_centerline.csv',
                    '--out', f'data/racelines/{track}_raceline'],
                   capture_output=True, text=True)
    print(f'wrote data/racelines/{track}_{{centerline,safe,raceline}}.csv')


if __name__ == '__main__':
    main()
