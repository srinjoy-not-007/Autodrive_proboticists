#!/usr/bin/env python3
"""v = ref + alpha*(cand - ref) inside s-windows (full strength inside), cand elsewhere;
then the same reachability pass win_stack_blend.py uses.  Branch experiment.
usage: partial_blend.py <cand.csv> <ref.csv> <out.csv> "lo:hi:alpha,lo:hi:alpha,..." """
import sys
import numpy as np
sys.path.insert(0, 'tools')
from optimize_raceline import limit_long_accel                   # noqa: E402
AX, BRAKE = 6.5, 4.6
cand, ref, out = sys.argv[1:4]
c = np.loadtxt(cand, delimiter=','); r = np.loadtxt(ref, delimiter=',')
s = c[:, 0]; v = c[:, 5].copy(); L = s[-1] + (s[1] - s[0])
for w in sys.argv[4].split(','):
    lo, hi, al = (float(x) for x in w.split(':'))
    span = (hi - lo) % L; d = (s - lo) % L
    inside = d <= span          # full strength inside, nothing outside: the
    # same semantics as win_stack_blend.py (its taper never binds), so alpha 0
    # reproduces its output and the reachability pass makes the joins drivable.
    target = np.minimum(v, r[:, 5] + al * np.maximum(c[:, 5] - r[:, 5], 0.0))
    v = np.where(inside, target, v)
ds = np.r_[np.diff(s), s[1] - s[0]]
v2 = limit_long_accel(v, ds, AX, BRAKE)
vv = np.r_[v2, v2[0]]
c[:, 5] = v2; c[:, 6] = (vv[1:] ** 2 - vv[:-1] ** 2) / (2 * ds)
np.savetxt(out, c, delimiter=',', fmt='%.4f')
print('%s: ideal %.3f s  v %.2f-%.2f' % (out, (ds / v2).sum(), v2.min(), v2.max()))
