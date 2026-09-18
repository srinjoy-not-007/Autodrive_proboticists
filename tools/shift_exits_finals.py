#!/usr/bin/env python3
"""One-off: shift the finals line toward the INSIDE on the three corner exits
where the ay-9.0 car runs wide into the right wall (s 19.9, s 26.8, last
hairpin into s 0).  Same method as tools/fix_s39_finals.py: a smoothed lateral
shift along the line normal, psi/kappa re-derived inside the shifted zones only.
Speeds are copied from the base line (reprofile afterwards).
usage: shift_exits_finals.py <base.csv> <out.csv> <shift_m> "lo:hi,lo:hi,..." [ramp]
Windows may wrap (lo > hi)."""
import sys
import numpy as np
from scipy.ndimage import gaussian_filter1d
base, out, shift_m, wins = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
ramp = float(sys.argv[5]) if len(sys.argv) > 5 else 1.2
a = np.loadtxt(base, delimiter=','); s = a[:, 0]; xy = a[:, 1:3]; n = len(s)
L = s[-1] + (s[1] - s[0])
tg = np.roll(xy, -1, 0) - np.roll(xy, 1, 0)
nrm = np.c_[-tg[:, 1], tg[:, 0]] / np.linalg.norm(tg, axis=1)[:, None]
w = np.zeros(n)
for win in wins.split(','):
    lo, hi = (float(x) for x in win.split(':'))
    span = (hi - lo) % L
    d = (s - lo) % L                      # distance past the window start
    w = np.maximum(w, np.clip(np.minimum(d / ramp, (span - d) / ramp), 0, 1) * (d <= span))
sv = gaussian_filter1d((shift_m * w)[:, None] * nrm, 2.0, axis=0, mode='wrap')
xy2 = xy + sv
d = np.linalg.norm(np.roll(xy2, -1, 0) - xy2, axis=1); s2 = np.r_[0, np.cumsum(d[:-1])]
tg2 = np.roll(xy2, -1, 0) - np.roll(xy2, 1, 0); psi2 = np.arctan2(tg2[:, 1], tg2[:, 0])
dpsi = np.diff(np.unwrap(np.r_[psi2, psi2[0]])); kap2 = dpsi / ((d + np.roll(d, 1)) / 2)
zz = np.linalg.norm(sv, axis=1) > 1e-4
kap2 = np.where(zz, kap2, a[:, 4]); psi2 = np.where(zz, psi2, a[:, 3])
np.savetxt(out, np.c_[s2, xy2, psi2, kap2, a[:, 5], a[:, 6]], delimiter=',', fmt='%.4f')
print('%s: length %.2f -> %.2f, |kappa| max %.2f -> %.2f, shifted nodes %d' % (out, L, s2[-1] + d[-1], abs(a[:, 4]).max(), abs(kap2).max(), zz.sum()))
