#!/usr/bin/env python3
"""Hairpin-only speed edits on a FIXED geometry (Sep 20 branch experiment).

Why this exists: the per-0.25 m anatomy of the two v9 centuries
(`docs/research/hairpin_anatomy_sep20.md`) showed that the shipped profile's own
YAW-RATE demand, v*kappa, reaches 3.12 rad/s at s 19.0 and 3.19 at s 43.6 --
past the 2.7 rad/s steady breakaway measured by `tools/sysid_breakaway.py`.  The
ay cap never binds there (ay is 4.6-6.4 against a cap of 9.0), so no `reprofile`
rung can fix it: the binding quantity is v*kappa, not v^2*kappa.

Three edits, each restricted to an explicit s-window so no other corner moves
(s 24-27 in particular is closed -- see FINALS_TUNING section 3):

  --cap R        v <= R / |kappa|        (the yaw-rate envelope)
  --flat K       inside a window, hold v at the window minimum wherever
                 |kappa| >= K, i.e. no acceleration while the rack is at lock
  --advance D    v(s) <- min(v(s), v(s+D)) on the approach, so the planned brake
                 finishes D metres earlier and is over before turn-in
  --release L    v <- max(v, v_L) after the corner (L = a faster profile of the
                 same geometry), to spend the corner saving on the exit

Then the same reachability pass `scripts/partial_blend.py` uses
(`limit_long_accel`, ax 6.5 / brake 4.6), so the joins are drivable.

usage: hairpin_profile.py <base.csv> <out.csv> [opts]  (windows are lo:hi, wrapping)
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, 'tools'))
from optimize_raceline import limit_long_accel                   # noqa: E402

AX, BRAKE = 6.5, 4.6


def windows(spec, s, L):
    """Boolean mask for a comma list of wrapping lo:hi windows."""
    m = np.zeros(len(s), bool)
    for w in spec.split(','):
        lo, hi = (float(x) for x in w.split(':'))
        m |= ((s - lo) % L) <= ((hi - lo) % L)
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('base')
    ap.add_argument('out')
    ap.add_argument('--zones', default='18.2:20.2,42.7:44.5',
                    help='hairpin windows the cap/flat edits act in')
    ap.add_argument('--approach', default='17.0:18.8,41.4:43.0',
                    help='windows the --advance edit acts in')
    ap.add_argument('--exits', default='20.2:21.5,44.2:2.0',
                    help='windows the --release edit acts in')
    ap.add_argument('--cap', type=float, default=0.0, help='yaw-rate cap, rad/s')
    ap.add_argument('--flat', type=float, default=0.0, help='hold v where |kappa| >= this')
    ap.add_argument('--advance', type=float, default=0.0, help='metres to advance the brake')
    ap.add_argument('--release', default='', help='faster line to raise the exits to')
    args = ap.parse_args()

    c = np.loadtxt(args.base, delimiter=',')
    s, kap, v = c[:, 0], np.abs(c[:, 4]), c[:, 5].copy()
    ds = np.r_[np.diff(s), s[1] - s[0]]
    L = s[-1] + ds[0]
    zone = windows(args.zones, s, L)

    if args.cap > 0.0:
        v = np.where(zone, np.minimum(v, args.cap / np.maximum(kap, 1e-6)), v)

    if args.flat > 0.0:
        # contiguous runs of (zone & kappa >= flat), wrapping
        hot = zone & (kap >= args.flat)
        idx = np.flatnonzero(hot)
        if len(idx):
            breaks = np.flatnonzero(np.diff(idx) > 1) + 1
            for run in np.split(idx, breaks):
                v[run] = v[run].min()

    if args.advance > 0.0:
        appr = windows(args.approach, s, L)
        step = int(round(args.advance / ds[0]))
        v = np.where(appr, np.minimum(v, np.roll(v, -step)), v)

    if args.release:
        r = np.loadtxt(args.release, delimiter=',')
        ex = windows(args.exits, s, L)
        v = np.where(ex, np.maximum(v, r[:, 5]), v)

    v = limit_long_accel(v, ds, AX, BRAKE)
    vv = np.r_[v, v[0]]
    c[:, 5] = v
    c[:, 6] = (vv[1:] ** 2 - vv[:-1] ** 2) / (2 * ds)
    np.savetxt(args.out, c, delimiter=',', fmt='%.4f')
    yaw = v * kap
    print('%s: ideal %.3f s  v %.2f-%.2f  max yaw %.2f rad/s (hairpins %.2f)'
          % (os.path.basename(args.out), (ds / v).sum(), v.min(), v.max(),
             yaw.max(), yaw[zone].max()))


if __name__ == '__main__':
    main()
