#!/usr/bin/env python3
"""Signed lateral offset (+ = left of travel) of a driven log or a line CSV
against a reference line, binned by s. Usage:
  signed_offset.py <ref line csv> <log csv | line csv> [s_lo s_hi]

For a log the positions are the IPS **ground truth** (`x, y`); pass
--estimate to use the localization estimate (`px, py`) instead, which is what
the controller believed. Column semantics live in `tools/racelog.py`.
"""
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import racelog  # noqa: E402


def load_xy(path, estimate=False):
    """Positions from a race log (teleports dropped) or from a line CSV."""
    if path.endswith('.csv') and open(path).readline().startswith('t,'):
        log = racelog.load_log(path)
        p = log['est_xy'] if estimate else log['truth_xy']
        return p[~racelog.teleport_mask(log)]
    return racelog.load_line(path)[1]


def main(ref, other, s_lo=10.0, s_hi=14.0, estimate=False):
    s, xy, _psi, kappa, _v = racelog.load_line(ref)
    p = load_xy(other, estimate)
    j = racelog.project(xy, p)
    e = racelog.signed_offset(xy, p, j)
    for a in np.arange(s_lo, s_hi, 1.0):
        m = (s[j] >= a) & (s[j] < a + 1)
        if m.sum():
            print(f's {a:4.0f}-{a+1:4.0f} kappa {kappa[(s>=a)&(s<a+1)].mean():+.2f}  offset mean {e[m].mean():+.3f} p10 {np.percentile(e[m],10):+.3f} p90 {np.percentile(e[m],90):+.3f}  n={m.sum()}')


if __name__ == '__main__':
    a = [v for v in sys.argv if v != '--estimate']
    main(a[1], a[2], *(float(x) for x in a[3:5]),
         estimate='--estimate' in sys.argv)
