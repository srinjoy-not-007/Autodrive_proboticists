#!/usr/bin/env python3
"""How much margin a run or a raceline actually had -- not whether it crashed.

WHY THIS EXISTS
---------------
"36 laps, 0 collisions" was treated as evidence the shipping config was safe.
It was one sample of a configuration with no margin left. Three points on the
Porto lap sat at exactly 100% of the ay cap, and when the sim was moved to a
real display -- slower, so nominally SAFER -- the same config took 4 collisions
in 38 laps at those points. A clean run and a marginal run look identical in
the lap table. They do not look identical here.

Two independent margins, because a run can be short of either:

  GRIP margin (needs a raceline)  1 - |ay_plan| / ay_cap at each point.
      Zero means the plan is asking for every bit of lateral grip it has
      budgeted, so any tracking error at all is unpayable.

  SPACE margin (needs a run log + the map)  distance from the swept body
      corners to the nearest wall. Reported as the per-lap minimum, because
      the number that matters is the worst moment of the worst lap, not an
      average over a lap that is mostly straight.

Usage:
  margin_report.py --line data/racelines/track_v2_safe.csv [--ay 4.5]
  margin_report.py --log data/logs/race_X.csv --map data/maps/track_v2
                   [--line ...] [--near 0.10]
"""

import argparse
import csv
import sys

import numpy as np

sys.path.insert(0, 'tools')
from footprint_analysis import CORNERS, clearance, load_dist   # noqa: E402


def grip_margin(line_csv, ay_cap):
    a = np.loadtxt(line_csv, delimiter=',')
    s, kappa, vx = a[:, 0], a[:, 4], a[:, 5]
    ay = np.abs(vx * vx * kappa)
    margin = 1.0 - ay / ay_cap
    print(f'\nGRIP MARGIN  {line_csv}   (ay cap {ay_cap})')
    print(f'  peak ay {ay.max():.3f} m/s2   min margin {margin.min()*100:+.1f}%')
    for pct in (1.0, 2.0, 5.0):
        n = int((margin < pct / 100.0).sum())
        print(f'  points with < {pct:.0f}% margin : {n:3d} / {len(s)}')
    tight = margin < 0.01
    if tight.any():
        print(f'  at s = {np.round(s[tight], 1)}')
    # Where the profile ACCELERATES into a tightening corner: the shape that
    # produced every Porto collision. kappa rising while v is also rising.
    dk = np.gradient(np.abs(kappa))
    dv = np.gradient(vx)
    bad = (dk > 0.005) & (dv > 0.0)
    if bad.any():
        print(f'  accelerating into a tightening corner at {int(bad.sum())} '
              f'points, s = {np.round(s[bad], 1)[:20]}')
    return margin


def space_margin(log_csv, map_prefix, near, line_csv=None):
    rows = list(csv.DictReader(open(log_csv)))
    t = np.array([float(r['t']) for r in rows])
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    col = np.array([float(r['collision_count']) for r in rows])
    dist, res, ox, oy = load_dist(map_prefix)

    d = np.diff(xy, axis=0)
    step = np.hypot(d[:, 0], d[:, 1])
    good = (step > 0.01) & (step < 0.5)
    head = np.full(len(xy), np.nan)
    head[:-1][good] = np.arctan2(d[good, 1], d[good, 0])
    ok = ~np.isnan(head)
    head = np.interp(np.arange(len(xy)), np.flatnonzero(ok), head[ok])

    worst = np.full(len(xy), np.inf)
    for cx, cy in CORNERS:
        px = xy[:, 0] + cx * np.cos(head) - cy * np.sin(head)
        py = xy[:, 1] + cx * np.sin(head) + cy * np.cos(head)
        c = clearance(dist, res, ox, oy, np.column_stack([px, py]))
        worst = np.minimum(worst, np.nan_to_num(c, nan=np.inf))
    moving = np.hypot(*np.gradient(xy, axis=0).T) > 1e-4
    w = worst[np.isfinite(worst) & moving]

    print(f'\nSPACE MARGIN  {log_csv}')
    print(f'  worst-corner clearance: min {w.min():.3f}  p1 {np.percentile(w,1):.3f}'
          f'  p5 {np.percentile(w,5):.3f}  median {np.median(w):.3f}')
    print(f'  samples within {near:.2f} m of a wall: {int((w < near).sum())} '
          f'({100*(w < near).mean():.2f}%)')
    print(f'  collisions in log: {int(col[-1]-col[0])}')

    # Per-lap worst moment. A config that is safe on average but has one lap
    # scraping the wall is the config that fails on somebody else's machine.
    if line_csv:
        a = np.loadtxt(line_csv, delimiter=',')
        cx_, cy_ = a[:, 1], a[:, 2]
        idx = np.array([((cx_-p[0])**2 + (cy_-p[1])**2).argmin() for p in xy])
        s_line = a[idx, 0]
        wrap = np.flatnonzero(np.diff(s_line) < -0.5 * a[-1, 0]) + 1
        bounds = np.concatenate([[0], wrap, [len(xy)]])
        mins = []
        for k in range(len(bounds) - 1):
            seg = worst[bounds[k]:bounds[k+1]]
            seg = seg[np.isfinite(seg)]
            if len(seg) > 20:
                mins.append(seg.min())
        if mins:
            mins = np.array(mins)
            print(f'  per-lap minimum clearance over {len(mins)} laps: '
                  f'worst {mins.min():.3f}  median {np.median(mins):.3f}  '
                  f'best {mins.max():.3f}')
            print(f'  laps whose closest approach was under {near:.2f} m: '
                  f'{int((mins < near).sum())} / {len(mins)}')
        worst_i = int(np.nanargmin(np.where(np.isfinite(worst), worst, np.inf)))
        print(f'  closest approach at t={t[worst_i]-t[0]:.1f}s, '
              f's={s_line[worst_i]:.2f} m, xy=({xy[worst_i,0]:.2f},'
              f'{xy[worst_i,1]:.2f})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--line')
    ap.add_argument('--log')
    ap.add_argument('--map')
    ap.add_argument('--ay', type=float, default=4.5)
    ap.add_argument('--near', type=float, default=0.10,
                    help='clearance below this counts as a near miss')
    args = ap.parse_args()
    if args.line:
        grip_margin(args.line, args.ay)
    if args.log:
        if not args.map:
            sys.exit('--log needs --map')
        space_margin(args.log, args.map, args.near, args.line)
    if not args.line and not args.log:
        sys.exit('nothing to do: pass --line and/or --log')


if __name__ == '__main__':
    main()
