#!/usr/bin/env python3
"""Re-draw a raceline's LONGITUDINAL ramps at the MEASURED brake, and (option)
clamp its corner speeds to the MEASURED speed-dependent lateral envelope.

Geometry is untouched: identical s/x/y/psi/kappa, new vx/ax.  Unlike
`reprofile.py` this does NOT recompute the corner caps from `--ay`; it starts
from the profile you hand it (so hand-held slow zones and `--ay-tight` survive)
and only changes what the measurement says is wrong:

1. **The brake.**  `data/logs/brake_20260919_132335.csv` (the open-floor brake
   probe): throttle exactly 0.0 locks the wheels and the car holds **6.0-6.8
   m/s2 from 7.7 m/s all the way down to 1.7 m/s** (84 pooled samples, per-bin
   medians 6.00 / 6.06 / 6.34 / 6.62 / 6.78 / 6.29 at 1-2 ... 6-7 m/s; the 7-8
   bin is the command-latency transient, not a limit).  The shipping profile
   plans **4.6**.  42 % of the ideal lap is spent braking and the two big zones
   (s 10.8-18.7, 8.00 m, |kappa| p50 **0.00**; s 31.5-37.3, 5.80 m, p50 0.16)
   are straight, so the unused 1.4 m/s2 is free time with no clearance cost.
   A locked wheel steers nothing, so the higher rate is applied only where the
   line is straight: `a_brake(s)` ramps from `--brake-hi` at `|kappa| <=
   --kappa-lo` to `--brake-lo` at `|kappa| >= --kappa-hi`.

2. **The breakaway envelope** (`--r-max` / `--r-dwell`, both off by default).
   Breakaway is a constant asked YAW RATE `r = v*|kappa|`, not an ay ceiling
   (`tools/sysid_breakaway.py`): attached cells reach 2.63 rad/s, cells that
   let go start at 2.79, and past breakaway the car holds only 2.30*sqrt(v)
   whatever the steering.  But the staircase holds each cell to steady state,
   and on track a 0.19-0.29 s excursion past that still delivers 0.95 of
   asked.  So `--r-max` is the hard cap (usually too conservative) and
   `--r-dwell` is the useful one: it shaves only the excursions that last
   longer than a demonstrated-safe burst.  On the shipping line exactly four
   runs go over 2.4 rad/s -- s 18.8-19.5 (**0.400 s**, r 3.12), s 24.1-24.7
   (0.210 s), s 37.4-37.6 (0.126 s) and s 43.3-43.6 (0.236 s, r 3.19) -- which
   are the four known run-wide/collision sites, and the first hairpin is the
   only one held past breakaway for longer than any excursion the live car has
   ever been shown to survive.

Usage:
  grip_envelope.py data/racelines/finals_mesh_apex_f42k_race.csv \
      --brake-hi 6.0 --out data/racelines/finals_mesh_f42k_brk60
  grip_envelope.py <line> --brake-hi 6.0 --r-dwell 0.25 --out <stem>
"""

import os as _os
for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS'):
    _os.environ.setdefault(_k, '1')
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# The breakaway law, from `tools/sysid_breakaway.py` on the 61-cell staircase
# (`data/logs/sysid_20260919_131340.csv`): breakaway is a constant ASKED YAW
# RATE `r = v*tan(delta)/L`, not a lateral-acceleration ceiling.  Attached
# cells reach at most 2.627 rad/s and the ones that let go start at 2.793; a
# second, independent ay ceiling near 10.5 m/s2 binds only above 5 m/s.
#
# The catch, and it is the whole design here: the staircase HOLDS each cell to
# steady state (~1 s).  On track the excursions past that threshold last
# 0.19-0.29 s and the live car does NOT let go -- it still delivers 0.95 of
# asked.  So the usable limit is a DWELL, not a level: a short burst to ~3.2
# rad/s is demonstrated safe (hairpin 2 asks 3.19 for 0.236 s and is clean
# over 100 laps), a sustained one is not.
R_STEADY = 2.70          # rad/s -- the steady-state breakaway threshold
R_BURST = 3.30           # rad/s -- demonstrated safe for a short excursion
AY_CEIL = 10.5           # m/s2 -- the second, independent ceiling (v > 5 m/s)


def v_yaw_env(kappa, r_max, ay_cap=AY_CEIL, v_hi=12.0):
    """Speed cap from the yaw-rate law plus the independent ay ceiling."""
    k = np.maximum(np.abs(np.asarray(kappa, dtype=float)), 1e-6)
    return np.minimum(np.minimum(r_max / k, np.sqrt(ay_cap / k)), v_hi)


def dwell_cap(v, kappa, ds, r_steady, dwell_s):
    """Shave the MIDDLE of any excursion above `r_steady` that lasts longer
    than `dwell_s`, until it does not.

    Only ever reduces speed.  Leaves short bursts -- the ones the live car is
    demonstrated to survive -- completely alone, which is why this is not the
    same thing as clamping every corner to the steady-state threshold.
    """
    v = np.asarray(v, dtype=float).copy()
    k = np.abs(np.asarray(kappa, dtype=float))
    for _ in range(60):
        r = v * k
        idx = np.flatnonzero(r > r_steady)
        if not len(idx):
            break
        worst, worst_dur = None, 0.0
        for g in np.split(idx, np.flatnonzero(np.diff(idx) > 2) + 1):
            dur = float(np.sum(ds[g] / v[g]))
            if dur > worst_dur:
                worst, worst_dur = g, dur
        if worst_dur <= dwell_s:
            break
        # Cut from the middle outwards: the burst keeps its entry and exit,
        # which is where the demonstrated-safe excursions live.
        n_cut = max(1, int(round(len(worst) * (1.0 - dwell_s / worst_dur))))
        c = len(worst) // 2
        lo = max(0, c - n_cut // 2)
        sel = worst[lo:lo + n_cut]
        v[sel] = np.minimum(v[sel], r_steady / np.maximum(k[sel], 1e-6))
    return v


def v_lat_generated(kappa, ay=9.0, ay_tight=4.5, kappa_tight=1.15):
    """The corner cap the SHIPPING line was generated with.

    `make_track_lines.PROFILE` ramps the lateral cap from `ay` down to
    `ay_tight` over |kappa| in [kappa_tight, 1.5*kappa_tight].  Reconstructing
    it lets the braking ramps be redrawn without ever letting a mid-corner
    braking point rise above the cap it was built under.
    """
    k = np.abs(np.asarray(kappa, dtype=float))
    t = np.clip((k - kappa_tight) / max(0.5 * kappa_tight, 1e-6), 0.0, 1.0)
    ay_k = ay + t * (ay_tight - ay)
    return np.sqrt(ay_k / np.maximum(k, 1e-6))


def brake_profile(kappa, brake_hi, brake_lo, kappa_lo, kappa_hi):
    """Per-point deceleration limit: the locked wheel only where it is straight."""
    k = np.abs(np.asarray(kappa, dtype=float))
    t = np.clip((k - kappa_lo) / max(kappa_hi - kappa_lo, 1e-6), 0.0, 1.0)
    return brake_hi + t * (brake_lo - brake_hi)


def long_passes(ceiling, ds, ax_max, a_brake, passes=4):
    """Forward/backward reachability with PER-POINT limits (closed lap)."""
    v = np.asarray(ceiling, dtype=float).copy()
    n = len(v)
    for _ in range(passes):
        for i in range(n):
            j = (i + 1) % n
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * ax_max[i] * ds[i]))
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v[i] = min(v[i], np.sqrt(v[j] ** 2 + 2.0 * a_brake[i] * ds[i]))
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('raceline')
    ap.add_argument('--out', required=True, help='output stem (.csv appended)')
    ap.add_argument('--brake-hi', type=float, default=6.0,
                    help='deceleration planned where the line is straight')
    ap.add_argument('--brake-lo', type=float, default=4.6,
                    help='deceleration kept where the line is curved')
    ap.add_argument('--kappa-lo', type=float, default=0.15)
    ap.add_argument('--kappa-hi', type=float, default=0.50)
    ap.add_argument('--ax', type=float, default=6.5,
                    help='forward-pass acceleration limit (plan, not plant)')
    ap.add_argument('--r-max', type=float, default=0.0,
                    help='>0: clamp v to r_max/|kappa| (and the 10.5 ay '
                         'ceiling) everywhere -- the hard yaw-rate envelope')
    ap.add_argument('--r-dwell', type=float, default=0.0,
                    help='>0: instead of a hard cap, shave only excursions '
                         'above --r-steady that last longer than this (s)')
    ap.add_argument('--r-steady', type=float, default=R_STEADY)
    ap.add_argument('--vmax', type=float, default=None)
    ap.add_argument('--gen-ay', type=float, default=9.0,
                    help='the lateral cap the input line was generated with')
    ap.add_argument('--gen-ay-tight', type=float, default=4.5)
    ap.add_argument('--gen-kappa-tight', type=float, default=1.15)
    ap.add_argument('--only', default='',
                    help='comma list lo:hi of s-windows OUTSIDE which the '
                         'input profile is restored unchanged')
    ap.add_argument('--protect', default='',
                    help='comma list lo:hi of s-windows whose speed may not '
                         'rise above the input profile')
    a = ap.parse_args()

    r = np.loadtxt(a.raceline, delimiter=',')
    s, x, y, psi, kappa = (r[:, i] for i in range(5))
    v0 = r[:, 5].copy()
    ds = np.diff(np.r_[s, s[-1] + (s[-1] - s[-2])])
    vmax = a.vmax if a.vmax is not None else float(v0.max())

    # Ceiling: hold the input profile wherever it was NOT brake-limited, and
    # open it to vmax inside the braking ramps so they can be redrawn.  Nothing
    # that sets a corner cap is touched, so this can only raise pre-corner
    # speed -- never an apex.
    ax_in = r[:, 6]
    v_cap = np.minimum(v_lat_generated(kappa, a.gen_ay, a.gen_ay_tight,
                                       a.gen_kappa_tight), vmax)
    ceiling = np.where(ax_in < -0.3, v_cap, v0)

    bad = v0 - v_cap
    if bad.max() > 0.05:
        i = int(np.argmax(bad))
        print(f'  WARNING: input exceeds the reconstructed corner cap by '
              f'{bad.max():.2f} m/s at s {s[i]:.1f} — check --gen-ay*')

    if a.r_max > 0.0:
        ceiling = np.minimum(ceiling, v_yaw_env(kappa, a.r_max, v_hi=vmax))

    a_brake = brake_profile(kappa, a.brake_hi, a.brake_lo, a.kappa_lo, a.kappa_hi)
    ax_max = np.full_like(ceiling, a.ax)
    vx = long_passes(ceiling, ds, ax_max, a_brake)

    def windows(spec):
        m = np.zeros(len(s), bool)
        for w in spec.split(','):
            lo, hi = (float(z) for z in w.split(':'))
            m |= (s >= lo) & (s <= hi) if lo <= hi else (s >= lo) | (s <= hi)
        return m

    if a.only:
        # Everything outside the named windows is put back to the input
        # profile byte-for-byte, so the arm is a single-variable experiment.
        keep = ~windows(a.only)
        vx[keep] = v0[keep]
        vx = long_passes(vx, ds, ax_max, a_brake)
    if a.protect:
        vx[windows(a.protect)] = np.minimum(vx[windows(a.protect)],
                                            v0[windows(a.protect)])
        vx = long_passes(vx, ds, ax_max, a_brake)

    if a.r_dwell > 0.0:
        vx = dwell_cap(vx, kappa, ds, a.r_steady, a.r_dwell)
        vx = long_passes(vx, ds, ax_max, a_brake)

    vn = np.roll(vx, -1)
    ax_out = (vn ** 2 - vx ** 2) / (2.0 * np.maximum(ds, 1e-6))

    out = np.column_stack([s, x, y, psi, kappa, vx, ax_out])
    np.savetxt(a.out + '.csv', out, delimiter=',', fmt='%.4f')
    ideal0 = float(np.sum(ds / v0))
    ideal = float(np.sum(ds / vx))
    print(f'{os.path.basename(a.out)}: brake {a.brake_hi}->{a.brake_lo} over '
          f'|kappa| {a.kappa_lo}-{a.kappa_hi}, ax {a.ax}, '
          f'r_max {a.r_max or "off"}, r_dwell {a.r_dwell or "off"}')
    print(f'  ideal lap {ideal:.3f} s  (input {ideal0:.3f} s, '
          f'{ideal - ideal0:+.3f})   v {vx.min():.2f}-{vx.max():.2f} m/s   '
          f'peak ay {np.max(vx * vx * np.abs(kappa)):.2f}')
    up = vx > v0 + 1e-3
    dn = vx < v0 - 1e-3
    print(f'  raised on {ds[up].sum():.1f} m (max +{np.max(vx - v0):.2f} m/s), '
          f'lowered on {ds[dn].sum():.1f} m (max {np.min(vx - v0):.2f} m/s)')
    if dn.any():
        idx = np.flatnonzero(dn)
        for g in np.split(idx, np.flatnonzero(np.diff(idx) > 3) + 1):
            print('    slower s %.1f-%.1f  %.2f -> %.2f m/s'
                  % (s[g[0]], s[g[-1]], v0[g].max(), vx[g].min()))


if __name__ == '__main__':
    main()
