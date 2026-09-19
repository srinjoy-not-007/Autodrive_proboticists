#!/usr/bin/env python3
"""Unused-width lines: raise the planned SPEED only in the corners the Sep 19
corner-by-corner audit proved carry unused wall margin.

Geometry (s,x,y,psi,kappa) is copied byte-for-byte from the base line; only
vx/ax change.  Inside a named corner window the speed is raised to a HOT
ay-profile of the same geometry (`tools/reprofile.py`'s `reprofile`); outside
it the base (shipping) profile is kept untouched -- in particular the three
capped slow zones (s 17.5-21.5, s 36-40, s 42-44.7 + 0-2) and the last hairpin
are never raised.  The result then goes through the same reachability pass
`scripts/partial_blend.py` uses (`limit_long_accel`, ax 6.5 / brake 4.6), so
every raise is reachable from, and sheddable into, its un-raised neighbours.

That last pass is the whole story of this experiment: it eats the s 40.9 raise
to +0.00 m/s and the s 22.2 raise to +0.03, because both corners are boxed in
between capped zones.  Only s 30-33 (and marginally s 27.6) has longitudinal
room.  See docs/FINALS_TUNING.md.

    ./scripts/build_unused_width.py
"""

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from reprofile import reprofile                                  # noqa: E402
from optimize_raceline import limit_long_accel                   # noqa: E402

AX, BRAKE = 6.5, 4.6
OUTDIR = os.path.join(ROOT, 'data/racelines')

# The UNUSED-WIDTH corner windows from the audit.  Live min wall distance
# (car centre), display / Xvfb, against a ~0.30 m limit:
#   A s 22.2 R      0.51 / 0.48
#   B s 27.6 apex   0.34 / 0.45   (marginal: display driven ay p99 already 8.3)
#   C s 30-33       0.57 / 0.62
#   D s 40.9 R      0.61 / 0.47
WINDOWS = {
    'A': (21.4, 23.6),
    'B': (27.5, 29.2),
    'C': (29.5, 33.2),
    'D': (40.2, 41.8),
}


def build(base_path, hot_ay, vmax, keys, out_stem):
    r = np.loadtxt(base_path, delimiter=',')
    s, kappa = r[:, 0], r[:, 4]
    el = np.diff(np.r_[s, s[-1] + (s[-1] - s[-2])])
    total = s[-1] + (s[1] - s[0])
    vbase = r[:, 5].copy()
    vhot, _ = reprofile(kappa, el, hot_ay, AX, BRAKE, 40.0, vmax)

    v = vbase.copy()
    for key in keys:
        lo, hi = WINDOWS[key]
        inside = ((s - lo) % total) <= ((hi - lo) % total)
        # raise only where the hot profile is faster; never lower the base
        v = np.where(inside, np.maximum(vbase, vhot), v)

    v2 = limit_long_accel(v, el, AX, BRAKE)
    vv = np.r_[v2, v2[0]]
    out = r.copy()
    out[:, 5] = v2
    out[:, 6] = (vv[1:] ** 2 - vv[:-1] ** 2) / (2 * el)
    path = os.path.join(OUTDIR, out_stem + '.csv')
    np.savetxt(path, out, delimiter=',', fmt='%.4f')

    survived = ', '.join(
        '%s +%.2f' % (k, float(np.max(v2[((s - WINDOWS[k][0]) % total)
                                        <= ((WINDOWS[k][1] - WINDOWS[k][0]) % total)]
                                     - vbase[((s - WINDOWS[k][0]) % total)
                                             <= ((WINDOWS[k][1] - WINDOWS[k][0]) % total)])))
        for k in keys)
    print('%-38s ideal %.3f (base %.3f)  raised: %s'
          % (out_stem, float(np.sum(el / v2)), float(np.sum(el / vbase)), survived))
    return path


def main():
    v86 = os.path.join(OUTDIR, 'finals_mesh_exitm_m10t6_h3v86_race.csv')
    h3 = os.path.join(OUTDIR, 'finals_mesh_exitm_m10t6_h3_race.csv')
    for ay in (10.0, 10.5):
        tag = 'a%d' % round(ay * 10)
        for keys in ('cd', 'acd', 'abcd'):
            build(v86, ay, 8.6, keys.upper(),
                  'finals_mesh_uw_%s_%s_v86_race' % (tag, keys))
    build(h3, 10.5, 8.0, 'ACD', 'finals_mesh_uw_a105_acd_h3_race')


if __name__ == '__main__':
    main()
