#!/usr/bin/env python3
"""Per-lap breakdown of a race log against the line it drove.

Usage:
  .venv/bin/python tools/lap_report.py data/logs/race_X.csv \
      data/racelines/porto_mesh_mc0.63_ay6.75.csv data/maps/porto_mesh \
      [--corners 10:12,24:26] [--csv out.csv] [--bin 1.0]

One row per completed lap: lap time (the simulator's own `last_lap_time` when
the logger recorded it, else the running `lap_time` read on the last sample
before the crossing -- one sensor period early), collisions inside the lap,
signed lateral offset against the line (+ = left of travel), |e_lat| p95,
minimum worst-body-corner clearance to the walls from the map distance field,
speed error against the planned profile, the entry overspeed inside each corner
window, and the logger's sensor period for that lap.

Truth for position and speed is the logged IPS pose, differentiated (the
encoder speed is a filtered estimate). Samples within 1.5 s of a collision are
dropped as teleports, and rows repeating the previous IPS pose -- the logger
writes faster than the ~10 Hz bridge -- are dropped so the period statistic
measures the sensor and not the writer.

The summary reports how much of the lateral error is *repeatable*: the variance
of the lap-averaged offset curve over s bins as a fraction of the total offset
variance. A high fraction means the error is a property of the line and the
controller, not of noise, so a feedforward correction can remove it.
"""

import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import racelog  # noqa: E402
from footprint_analysis import CORNERS, clearance, load_dist  # noqa: E402

TELEPORT_S = 1.5   # collisions move the car; this much either side is junk
MIN_SPEED = 0.3    # below this the car is stopped, not tracking


def load_log(path):
    """`x, y` (the IPS truth) and friends, with duplicated IPS rows removed.

    Column semantics -- truth vs estimate, and the c-2 collision rule -- live in
    `tools/racelog.py`. `raw` keeps the unfiltered log, because de-duplication
    deletes the collision row that attribution needs.
    """
    raw = racelog.load_log(path)
    keep = racelog.dedupe_ips_mask(raw)
    out = dict(t=raw['t'], x=raw['truth_x'], y=raw['truth_y'], lap=raw['lap'],
               lap_time=raw['lap_time'], coll=raw['coll'])
    if 'last_lap_time' in raw:
        out['last_lap_time'] = raw['last_lap_time']
    out = {k: v[keep] for k, v in out.items()}
    out['raw'] = raw
    return out


def project(line, x, y):
    """Nearest line point index for every sample (chunked to stay small)."""
    return racelog.project(line[:, 1:3], np.column_stack([x, y]))


def signed_offset(line, x, y, idx):
    """Lateral offset, + = left of the line's travel direction."""
    return racelog.signed_offset(line[:, 1:3], np.column_stack([x, y]), idx)


def true_speed(t, x, y):
    v = np.hypot(np.diff(x), np.diff(y)) / np.maximum(np.diff(t), 1e-3)
    v = np.concatenate([[v[0]], v])
    return np.convolve(v, np.ones(3) / 3.0, 'same')


def body_clearance(dist, res, ox, oy, x, y):
    """Worst body-corner clearance per sample, heading from travel."""
    xy = np.column_stack([x, y])
    d = np.diff(xy, axis=0)
    step = np.hypot(d[:, 0], d[:, 1])
    good = (step > 0.01) & (step < 0.5)
    head = np.full(len(xy), np.nan)
    head[:-1][good] = np.arctan2(d[good, 1], d[good, 0])
    ok = ~np.isnan(head)
    if not ok.any():
        return np.full(len(xy), np.nan)
    head = np.interp(np.arange(len(xy)), np.flatnonzero(ok), head[ok])
    worst = np.full(len(xy), np.inf)
    for cx, cy in CORNERS:
        px = x + cx * np.cos(head) - cy * np.sin(head)
        py = y + cx * np.sin(head) + cy * np.cos(head)
        c = clearance(dist, res, ox, oy, np.column_stack([px, py]))
        worst = np.minimum(worst, np.nan_to_num(c, nan=np.inf))
    return worst


def official_times(log):
    """{lap: simulator-latched duration} or {} -- see score_race.py."""
    if 'last_lap_time' not in log:
        return {}
    v, c = log['last_lap_time'], log['lap']
    steps = {}
    for i in np.flatnonzero(np.diff(c)) + 1:
        steps.setdefault(int(c[i]), i)
    out, prev = {}, None
    for lap in sorted(steps):
        stop = steps.get(lap + 1, len(c))
        for k in range(max(0, steps[lap] - 1), max(stop, steps[lap] + 1)):
            if np.isfinite(v[k]) and v[k] > 0 and v[k] != prev:
                out[lap], prev = float(v[k]), float(v[k])
                break
    return out


def lap_rows(log, line, dist_args, corners, bins):
    idx = project(line, log['x'], log['y'])
    sj = line[idx, 0]
    off = signed_offset(line, log['x'], log['y'], idx)
    v = true_speed(log['t'], log['x'], log['y'])
    dv = v - line[idx, 5]
    clr = body_clearance(*dist_args, log['x'], log['y'])
    bad = np.zeros(len(sj), bool)
    for i in np.flatnonzero(np.diff(log['coll'])) + 1:
        bad |= np.abs(log['t'] - log['t'][i]) < TELEPORT_S
    ok = (~bad) & (v > MIN_SPEED) & np.isfinite(clr)
    # Charge each collision to the lap it was being driven on at the PRE-IMPACT
    # sample (racelog's c-2 rule); the rows the counter steps on already carry
    # the teleported pose and can fall on the far side of a lap boundary.
    hits = {}
    if 'raw' in log:
        for rec in racelog.collisions(log['raw']):
            hits[rec['lap'] + 1] = hits.get(rec['lap'] + 1, 0) + 1
    official = official_times(log)
    laps, lap_bin_mean = [], {}
    steps = {}
    for i in np.flatnonzero(np.diff(log['lap'])) + 1:
        steps.setdefault(int(log['lap'][i]), int(i))
    for lap in sorted(steps):
        end = steps[lap]
        m = (log['lap'] == lap - 1)
        if not m.any():
            continue
        secs = official.get(lap)
        if secs is None:
            secs = float(log['lap_time'][end - 1]) if end else float('nan')
        g = m & ok
        if g.sum() < 5:
            continue
        e, dt = off[g], np.diff(log['t'][m])
        row = dict(lap=lap, time=secs, coll=hits.get(lap, 0),
                   e_mean=e.mean(), e_p95=np.percentile(e, 95),
                   abs_p95=np.percentile(np.abs(e), 95), clr_min=clr[g].min(),
                   dv_mean=dv[g].mean(),
                   dt_p50=float(np.median(dt)) if len(dt) else float('nan'))
        for n, (lo, hi) in enumerate(corners, 1):
            w = g & (sj >= lo) & (sj <= hi)
            row[f'ovsp{n}'] = dv[w].mean() if w.sum() else float('nan')
        laps.append(row)
        b = np.digitize(sj[g], bins) - 1
        lap_bin_mean[lap] = np.array(
            [e[b == k].mean() if np.any(b == k) else np.nan
             for k in range(len(bins) - 1)])
    return laps, lap_bin_mean, off[ok], sj[ok]


def repeatable(lap_bin_mean, off, sj, bins):
    """Fraction of offset variance carried by the lap-averaged s curve."""
    if not lap_bin_mean:
        return float('nan'), []
    curve = np.nanmean(np.vstack(list(lap_bin_mean.values())), axis=0)
    b = np.digitize(sj, bins) - 1
    b = np.clip(b, 0, len(curve) - 1)
    model = curve[b]
    use = np.isfinite(model)
    total = off[use].var()
    frac = model[use].var() / total if total > 0 else float('nan')
    order = np.argsort(-np.abs(np.nan_to_num(curve)))[:5]
    top = [(bins[k], bins[k + 1], curve[k], int((b == k).sum())) for k in order]
    return frac, top


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log')
    ap.add_argument('line')
    ap.add_argument('map_prefix')
    ap.add_argument('--corners', default='10:12,24:26',
                    help='comma-separated s windows lo:hi (default the two '
                         'Porto hairpins)')
    ap.add_argument('--bin', type=float, default=1.0, help='s bin width (m)')
    ap.add_argument('--csv', help='write the per-lap table here')
    args = ap.parse_args()

    corners = [tuple(float(v) for v in w.split(':'))
               for w in args.corners.split(',') if w]
    line = np.loadtxt(args.line, delimiter=',')
    log = load_log(args.log)
    dist_args = load_dist(args.map_prefix)
    bins = np.arange(0.0, line[-1, 0] + args.bin, args.bin)
    laps, per_lap_curve, off, sj = lap_rows(log, line, dist_args, corners, bins)
    if not laps:
        print('no complete laps in this log')
        return 1

    keys = ['lap', 'time', 'coll', 'e_mean', 'e_p95', 'abs_p95', 'clr_min',
            'dv_mean'] + [f'ovsp{n}' for n in range(1, len(corners) + 1)] \
        + ['dt_p50']
    head = (f'{"lap":>4} {"time":>7} {"col":>4} {"e_mean":>7} {"e_p95":>7} '
            f'{"|e|p95":>7} {"clr_min":>8} {"dv_mean":>8} '
            + ' '.join(f'ovsp{n}@{lo:g}-{hi:g}'.rjust(14)
                       for n, (lo, hi) in enumerate(corners, 1))
            + f' {"dt_p50":>7}')
    print(f'{args.log}   vs {os.path.basename(args.line)}   '
          f'({"official" if "last_lap_time" in log else "sim-timer"} lap times)')
    print(head)
    print('-' * len(head))
    for r in laps:
        print(f'{r["lap"]:4d} {r["time"]:7.3f} {r["coll"]:4d} {r["e_mean"]:+7.3f} '
              f'{r["e_p95"]:+7.3f} {r["abs_p95"]:7.3f} {r["clr_min"]:8.3f} '
              f'{r["dv_mean"]:+8.3f} '
              + ' '.join(f'{r[f"ovsp{n}"]:+14.3f}'
                         for n in range(1, len(corners) + 1))
              + f' {r["dt_p50"]:7.4f}')
    print('-' * len(head))
    med = {k: float(np.nanmedian([r[k] for r in laps])) for k in keys[1:]}
    print(f'{"med":>4} {med["time"]:7.3f} {med["coll"]:4.1f} {med["e_mean"]:+7.3f} '
          f'{med["e_p95"]:+7.3f} {med["abs_p95"]:7.3f} {med["clr_min"]:8.3f} '
          f'{med["dv_mean"]:+8.3f} '
          + ' '.join(f'{med[f"ovsp{n}"]:+14.3f}'
                     for n in range(1, len(corners) + 1))
          + f' {med["dt_p50"]:7.4f}')

    frac, top = repeatable(per_lap_curve, off, sj, bins)
    total_coll = sum(r['coll'] for r in laps)
    logged = int(log['coll'][-1] - log['coll'][0])
    print(f'\n{len(laps)} complete laps, {total_coll} collisions in them; '
          f'lap-time sum {sum(r["time"] for r in laps):.2f} s')
    if logged != total_coll:
        print(f'({logged} collisions in the whole log; the rest fall outside a '
              f'completed lap, e.g. in the unfinished last one)')
    print(f'repeatable lateral error: {100 * frac:.1f}% of the offset variance '
          f'is the lap-mean curve over {args.bin:g} m s bins '
          f'(the rest is lap-to-lap scatter)')
    print('largest |mean signed offset| bins (+ = left of travel):')
    for lo, hi, mean, n in top:
        print(f'  s {lo:6.1f}-{hi:5.1f}  mean {mean:+.3f} m  n={n}')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(laps)
        print(f'wrote {args.csv}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
