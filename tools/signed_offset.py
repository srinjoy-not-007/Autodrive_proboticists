#!/usr/bin/env python3
"""Signed lateral offset (+ = left of travel) of a driven log or a line CSV
against a reference line, binned by s. Usage:
  signed_offset.py <ref line csv> <log csv | line csv> [s_lo s_hi]
"""
import csv, sys, numpy as np
def load_xy(path):
    if path.endswith('.csv') and open(path).readline().startswith('t,'):
        rows = list(csv.DictReader(open(path)))
        p = np.array([[float(r['x']), float(r['y'])] for r in rows])
        col = np.array([int(r['collision_count']) for r in rows]); t = np.array([float(r['t']) for r in rows])
        bad = np.zeros(len(t), bool)
        for i in np.flatnonzero(np.diff(col)) + 1: bad |= np.abs(t - t[i]) < 1.5
        return p[~bad]
    return np.loadtxt(path, delimiter=',')[:, 1:3]
def main(ref, other, s_lo=10.0, s_hi=14.0):
    rl = np.loadtxt(ref, delimiter=','); s, xy = rl[:, 0], rl[:, 1:3]
    fwd = np.roll(xy, -1, 0) - np.roll(xy, 1, 0); yaw = np.arctan2(fwd[:, 1], fwd[:, 0])
    p = load_xy(other)
    d = np.hypot(p[:, None, 0] - xy[None, :, 0], p[:, None, 1] - xy[None, :, 1]); j = d.argmin(1)
    dx, dy = p[:, 0] - xy[j, 0], p[:, 1] - xy[j, 1]
    e = -np.sin(yaw[j]) * dx + np.cos(yaw[j]) * dy
    for a in np.arange(s_lo, s_hi, 1.0):
        m = (s[j] >= a) & (s[j] < a + 1)
        if m.sum(): print(f's {a:4.0f}-{a+1:4.0f} kappa {rl[(s>=a)&(s<a+1),4].mean():+.2f}  offset mean {e[m].mean():+.3f} p10 {np.percentile(e[m],10):+.3f} p90 {np.percentile(e[m],90):+.3f}  n={m.sum()}')
if __name__ == '__main__':
    a = sys.argv; main(a[1], a[2], *(float(x) for x in a[3:5]))
