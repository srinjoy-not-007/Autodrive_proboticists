#!/usr/bin/env python3
"""Cross-track error of a race log against the line it was driving.

Usage: .venv/bin/python tools/crosstrack.py data/logs/race_X.csv data/racelines/track_v2_safe.csv [s_lo s_hi]

Reports lap-wide |e_lat| p50/p95/max and the same inside an s window (default
14.0-17.5 m, the corner that produces every collision on Porto). Samples within
1 s of a collision are dropped (teleport).
"""
import csv, sys
import numpy as np

def main(log, line, s_lo=14.0, s_hi=17.5):
    rl = np.loadtxt(line, delimiter=',')
    s, xy = rl[:, 0], rl[:, 1:3]
    rows = list(csv.DictReader(open(log)))
    t = np.array([float(r['t']) for r in rows])
    p = np.array([[float(r['x']), float(r['y'])] for r in rows])
    col = np.array([int(r['collision_count']) for r in rows])
    sp = np.array([float(r['speed']) for r in rows])
    bad = np.zeros(len(t), bool)
    for i in np.flatnonzero(np.diff(col)) + 1:
        bad |= np.abs(t - t[i]) < 1.0
    ok = (~bad) & (sp > 0.5)
    d = np.hypot(p[:, None, 0] - xy[None, :, 0], p[:, None, 1] - xy[None, :, 1])
    j = d.argmin(1)
    e = d[np.arange(len(p)), j]
    sj = s[j]
    def stats(m):
        v = e[m]
        return f'n={m.sum():5d}  p50={np.median(v):.3f}  p95={np.percentile(v,95):.3f}  max={v.max():.3f}'
    print('lap-wide   ', stats(ok))
    w = ok & (sj >= s_lo) & (sj <= s_hi)
    print(f's {s_lo}-{s_hi}  ', stats(w))
    print(f'speed in window mean {sp[w].mean():.2f}')

if __name__ == '__main__':
    a = sys.argv
    main(a[1], a[2], *(float(x) for x in a[3:5]))
