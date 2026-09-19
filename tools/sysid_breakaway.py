#!/usr/bin/env python3
"""What the sysid staircase says about breakaway, and what the track says back.

`tools/lateral_id.py` fits the ATTACHED lateral response and then says, in so
many words, that racing logs cannot answer where the car lets go -- the
controller never asks for what it has never been given.  The staircase in the
`-explore` image asks: hold a speed, step the steering, measure what comes
back.  This tool turns that log into the three constants
`tools/sim_tracker.py` needs, and then checks them against the live race logs,
which is where the answer gets interesting.

    tools/sysid_breakaway.py data/logs/sysid_20260919_131340.csv
    tools/sysid_breakaway.py <sysid.csv> --live data/logs/race_*.csv

The finding, in one line: **breakaway is a constant ASKED YAW RATE**
`v * tan(delta) / L`, not a lateral-acceleration ceiling.  Over the 51 usable
cells the attached ones reach at most 2.627 rad/s and the ones that let go by
yaw rate start at 2.793; a second, independent ay ceiling near 10.5 m/s2 binds
only above 5 m/s.  Together they classify 50 of 51 cells.  Past breakaway the
car holds `2.30 * sqrt(v)` m/s2 whatever the steering.

The catch, which is why `--plant-breakaway` is opt-in and not in `--plant-fit`:
the staircase HOLDS each cell to steady state, and on track the excursions
past that threshold last 0.19-0.29 s.  Over those the live car does NOT let
go.  Run with `--live` to see it.

Offline only.  Pin it: taskset -c 22-27 nice -n 19 env OMP_NUM_THREADS=1 ...
"""

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src'))

import racelog as RL                                           # noqa: E402
from pit_crew import vehicle                                   # noqa: E402

L = vehicle.WHEELBASE
BROKEN = 0.75            # delivery ratio below which a cell has let go
ATTACHED = 0.80          # ...and above which it clearly has not


def cells(path, drop_frac=1.0 / 3.0):
    """One row per (v_target, delta_target) hold, transient third dropped."""
    d = np.genfromtxt(path, delimiter=',', names=True, dtype=None,
                      encoding='utf-8')
    ph = np.array([str(x) for x in d['phase']])
    m = ph == 'hold'
    if not m.sum():
        raise SystemExit(f'{path}: no "hold" phase; is this a sysid log?')
    vt, dt_ = d['v_target'][m], d['delta_target'][m]
    da, wz = d['delta_act'][m], d['wz'][m]
    ay, ve = d['imu_ay'][m], d['v_enc'][m]
    groups = {}
    for i in range(int(m.sum())):
        groups.setdefault((round(float(vt[i]), 2), round(float(dt_[i]), 4)),
                          []).append(i)
    out = []
    for key in sorted(groups):
        idx = np.array(groups[key])
        if len(idx) < 4:
            continue
        idx = idx[int(len(idx) * drop_frac):]
        v = float(np.median(ve[idx]))
        if v < 0.5:
            continue
        delta = float(np.median(da[idx]))
        w = float(np.median(np.abs(wz[idx])))
        a = float(np.median(np.abs(ay[idx])))
        k_ask = math.tan(abs(delta)) / L
        out.append(dict(v_target=key[0], delta=delta, n=len(idx), v=v, wz=w,
                        ay=a, k_ach=w / v, k_ask=k_ask,
                        ratio=(w / v) / max(k_ask, 1e-9),
                        r_ask=v * k_ask, ay_ask=v * v * k_ask))
    return out


def classify(rows, r_crit, ay_max):
    tp = fp = tn = fn = 0
    for r in rows:
        pred = (r['r_ask'] > r_crit) or (r['ay_ask'] > ay_max)
        obs = r['ratio'] < BROKEN
        tp += obs and pred
        fn += obs and not pred
        fp += (not obs) and pred
        tn += (not obs) and not pred
    return tp, fp, tn, fn


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__)
    ap.add_argument('sysid')
    ap.add_argument('--live', nargs='*', default=[],
                    help='race logs, to test the threshold against the track')
    ap.add_argument('--line',
                    default='data/racelines/finals_mesh_exitm_m10t6_h3_race.csv')
    a = ap.parse_args()
    rows = cells(a.sysid)
    print('%d usable cells from %s' % (len(rows), os.path.basename(a.sysid)))
    print()
    print('%6s %8s %7s %7s %7s %8s %8s %7s'
          % ('v', 'delta', 'wz', 'ay', 'k_ask', 'r_ask', 'ay_ask', 'ratio'))
    for r in rows:
        print('%6.2f %8.4f %7.3f %7.3f %7.4f %8.3f %8.3f %7.3f%s'
              % (r['v'], r['delta'], r['wz'], r['ay'], r['k_ask'],
                 r['r_ask'], r['ay_ask'], r['ratio'],
                 '  BROKEN' if r['ratio'] < BROKEN else ''))

    att = [r['r_ask'] for r in rows if r['ratio'] > ATTACHED]
    brk = [r['r_ask'] for r in rows
           if r['ratio'] < BROKEN and r['ay_ask'] <= 10.5]
    print()
    print('SEPARATION: attached cells reach at most %.3f rad/s of asked yaw '
          'rate;' % max(att))
    print('            cells that let go by yaw rate start at %.3f.'
          % min(brk))
    print()
    print('%8s %8s  %3s %3s %3s %3s' % ('r_crit', 'ay_max', 'TP', 'FP', 'TN',
                                        'FN'))
    best = None
    for r_crit in np.arange(2.5, 3.01, 0.1):
        for ay_max in (9.5, 10.0, 10.5, 11.0):
            tp, fp, tn, fn = classify(rows, r_crit, ay_max)
            print('%8.2f %8.1f  %3d %3d %3d %3d' % (r_crit, ay_max, tp, fp,
                                                    tn, fn))
            if best is None or (fp + fn) < best[0]:
                best = (fp + fn, r_crit, ay_max)
    print('best: r_crit %.2f  ay_max %.1f  (%d cells misclassified of %d)'
          % (best[1], best[2], best[0], len(rows)))

    post = [r for r in rows if r['ratio'] < 0.60]
    c = np.array([r['ay'] / math.sqrt(r['v']) for r in post])
    print()
    print('POST-BREAKAWAY: ay = C * sqrt(v) with C = %.3f +- %.3f over %d '
          'cells' % (c.mean(), c.std(), len(post)))

    if not a.live:
        return
    print()
    print('=== and now the track, which holds those demands only BRIEFLY ===')
    for p in a.live:
        log = RL.load_log(p)
        keep = RL.dedupe_ips_mask(log)
        t, xy = log['t'][keep], log['truth_xy'][keep]
        d = log['steer_fb_rad'][keep]
        v = np.hypot(np.gradient(xy[:, 0], t), np.gradient(xy[:, 1], t))
        yaw = np.unwrap(np.arctan2(np.gradient(xy[:, 1], t),
                                   np.gradient(xy[:, 0], t)))
        wz = np.gradient(yaw, t)
        k_ask = np.tan(np.abs(d)) / L
        r_ask = v * k_ask
        ok = np.isfinite(r_ask) & (v > 1.0) & (v < 9.0)
        print(' %s  (n=%d)' % (os.path.basename(p), ok.sum()))
        for thr in (2.4, 2.71, 3.0, 3.5, 4.0):
            m = ok & (r_ask > thr)
            if m.sum() < 5:
                print('   above %.2f rad/s: %d samples' % (thr, m.sum()))
                continue
            print('   above %.2f rad/s: %5d samples (%4.2f %%)  delivery '
                  'ratio %.3f' % (thr, m.sum(), 100.0 * m.sum() / ok.sum(),
                                  np.median(np.abs(wz[m]) / v[m] / k_ask[m])))
        above = (r_ask > 2.71) & ok
        runs, cur = [], 0
        for x in above:
            cur = cur + 1 if x else (runs.append(cur) or 0) if cur else 0
        if cur:
            runs.append(cur)
        if runs:
            dt = float(np.median(np.diff(t)))
            r = np.array(runs) * dt
            print('   excursions above 2.71 last p50 %.3f s, p90 %.3f, max '
                  '%.3f' % (np.median(r), np.quantile(r, .9), r.max()))


if __name__ == '__main__':
    main()
