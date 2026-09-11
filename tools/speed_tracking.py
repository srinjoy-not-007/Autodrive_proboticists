#!/usr/bin/env python3
"""Speed tracking of a race log against the planned profile, binned by s.

Usage: .venv/bin/python tools/speed_tracking.py data/logs/race_X.csv data/racelines/<line>.csv
True speed is differentiated from the logged IPS position (not the encoder),
smoothed over 3 samples; teleports (collisions) are dropped. Also prints every
collision with the local s, curvature, planned and true speed.
"""
import csv, sys
import numpy as np

def main(log, line):
    rl = np.loadtxt(line, delimiter=',')
    s, xy, kap, vp = rl[:, 0], rl[:, 1:3], rl[:, 4], rl[:, 5]
    rows = list(csv.DictReader(open(log)))
    t = np.array([float(r['t']) for r in rows])
    p = np.array([[float(r['x']), float(r['y'])] for r in rows])
    col = np.array([int(r['collision_count']) for r in rows])
    d = np.hypot(p[:, None, 0] - xy[None, :, 0], p[:, None, 1] - xy[None, :, 1])
    j = d.argmin(1); sj = s[j]
    v = np.hypot(*np.diff(p, axis=0).T) / np.maximum(np.diff(t), 1e-3)
    v = np.concatenate([[v[0]], v]); v = np.convolve(v, np.ones(3) / 3, 'same')
    bad = np.zeros(len(t), bool)
    for i in np.flatnonzero(np.diff(col)) + 1:
        bad |= np.abs(t - t[i]) < 1.5
        print(f'collision #{col[i]-col[0]} at s={sj[i]:.1f} kappa={kap[j[i]]:.2f} '
              f'v_plan={vp[j[i]]:.2f} v_true(1s before)={v[max(0,i-11)]:.2f} lap={rows[i]["lap_count"]}')
    ok = ~bad & (v > 0.3)
    err = v - vp[j]
    print(f'overall: true-plan mean {err[ok].mean():+.3f} m/s, p05 {np.percentile(err[ok],5):+.2f}, p95 {np.percentile(err[ok],95):+.2f}')
    for a in np.arange(0, s[-1], 2.0):
        m = ok & (sj >= a) & (sj < a + 2)
        if m.sum() < 5: continue
        print(f's {a:4.0f}-{a+2:4.0f} kmax {np.abs(kap[(s>=a)&(s<a+2)]).max():.2f} v_plan {vp[j][m].mean():.2f} v_true {v[m].mean():.2f} err {err[m].mean():+.2f}')

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
