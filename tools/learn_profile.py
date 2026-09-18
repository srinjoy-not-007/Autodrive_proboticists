#!/usr/bin/env python3
"""LMPC-style speed-profile learning from our OWN clean laps, for a known map.

The idea, in the form that survives contact with this stack:

Rosolia & Borrelli's LMPC improves a trajectory by keeping every previously
driven, feasible lap as a *sampled safe set* and only moving the plan where the
recorded data proves the move is safe.  We cannot run their online set-based
terminal cost (fixed offline profile in the CSV, ~10 Hz sensing, bang-bang
brake), but the **sampled-safe-set argument transfers offline**: for each arc
length ``s`` the clean laps say how much room the CAR ACTUALLY USED, and where
it did not use the room, the plan may take some of it.

So the update is per-``s`` and evidence-gated, never a global ay ladder:

* **margin** ``m(s)``   worst-body-corner distance to the wall, p2 over all
  clean samples in the bin (``footprint_analysis`` corners + the map distance
  transform).  This is the LMPC safe-set boundary: demonstrated, not modelled.
* **feasibility** ``f(s)``  driven speed / planned speed, p50.  Raising the plan
  where the car is already **under** it buys nothing (measured repeatedly here:
  the car is below plan on most samples and closing the gap makes it slower),
  so a raise needs the lap to have *tracked* the plan.
* **veto**  s-bands around a collision in a FAILED iteration's log (the
  ay-6.75 run), and bands where ``|cte|`` p95 is large.  LMPC's infeasible
  iterations are discarded; here they are kept as hard no-raise evidence.

The gain is then

    g(s) = 1 + alpha * clip((m - m_lo) / (m_hi - m_lo), 0, 1)   [gated]
         - beta  * clip((m_lo - m) / m_lo, 0, 1)

smoothed over ``--smooth`` metres, applied to ``vx``, and finally made
*dynamically consistent* the same way a profile is built: cap at the planned-ay
ceiling, forward pass at ``ax``, backward pass at ``brake``.  Geometry
(``s,x,y,psi,kappa``) is copied from the input line unchanged.

Usage:
  learn_profile.py <line.csv> --map data/maps/finals_mesh \\
      --log data/logs/race_A.csv --log data/logs/race_B.csv \\
      [--veto-log data/logs/race_F.csv] [--report] [--out <stem>]
"""

import argparse
import os
import sys

import numpy as np
from scipy.ndimage import gaussian_filter1d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import racelog                                                  # noqa: E402
from footprint_analysis import CORNERS, clearance, load_dist    # noqa: E402


def body_clearance(dist, res, ox, oy, xy):
    """Worst body-corner clearance, heading from the direction of travel."""
    d = np.diff(xy, axis=0)
    step = np.hypot(d[:, 0], d[:, 1])
    good = (step > 0.01) & (step < 0.5)
    head = np.full(len(xy), np.nan)
    head[:-1][good] = np.arctan2(d[good, 1], d[good, 0])
    ok = ~np.isnan(head)
    if not ok.any():
        return np.full(len(xy), np.nan)
    head = np.interp(np.arange(len(xy)), np.flatnonzero(ok), head[ok])
    worst = np.full(len(xy), np.inf)
    for cx, cy in CORNERS:
        px = xy[:, 0] + cx * np.cos(head) - cy * np.sin(head)
        py = xy[:, 1] + cx * np.sin(head) + cy * np.cos(head)
        c = clearance(dist, res, ox, oy, np.column_stack([px, py]))
        worst = np.minimum(worst, np.nan_to_num(c, nan=np.inf))
    return worst


def gather(log_paths, line_xy, dist_args, warmup=1):
    """Per-sample (s index, margin, offset, v, ay, brake) pooled over logs."""
    out = {k: [] for k in ('idx', 'marg', 'off', 'v', 'ay', 'brake')}
    for p in log_paths:
        log = racelog.load_log(p)
        fresh_mask = racelog.dedupe_ips_mask(log)
        keep = fresh_mask & ~racelog.teleport_mask(log)
        keep &= log['lap'] >= warmup           # drop the standing-start lap
        xy = log['truth_xy']
        v = racelog.truth_speed(log)
        marg = body_clearance(*dist_args, xy)
        idx = racelog.project(line_xy, xy)
        off = racelog.signed_offset(line_xy, xy, idx)
        # lateral acceleration of the DRIVEN path: v * heading rate, heading
        # rate from the IPS trace (the log has no yaw-rate channel).
        fresh = np.flatnonzero(fresh_mask)
        ay = np.zeros(len(xy))
        if len(fresh) > 8:
            fx, fy, ft = xy[fresh, 0], xy[fresh, 1], log['t'][fresh]
            hd = np.unwrap(np.arctan2(np.diff(fy), np.diff(fx)))
            hd = gaussian_filter1d(hd, 1.5)
            dt = np.maximum(np.diff(ft), 1e-3)
            r = np.r_[0.0, np.diff(hd) / dt[1:], 0.0]
            ayf = np.abs(r) * v[fresh]
            src = np.searchsorted(fresh, np.arange(len(xy)), 'right') - 1
            ay = ayf[np.clip(src, 0, len(ayf) - 1)]
        thr = log.get('throttle')
        if thr is None:
            thr = np.full(len(xy), np.nan)
        out['idx'].append(idx[keep])
        out['marg'].append(marg[keep])
        out['off'].append(off[keep])
        out['v'].append(v[keep])
        out['ay'].append(ay[keep])
        out['brake'].append((np.abs(thr[keep]) < 1e-9).astype(float))
    return {k: np.concatenate(vv) for k, vv in out.items()}


def bin_stats(d, s, nbin_m, total_s):
    """Aggregate per-sample data into s bins of width ``nbin_m``."""
    b = np.floor(s[d['idx']] / nbin_m).astype(int)
    nb = int(np.ceil(total_s / nbin_m))
    b = np.clip(b, 0, nb - 1)
    keys = ('n', 'marg_p2', 'marg_p5', 'off_p95', 'v_p50', 'ay_p95', 'ay_p99',
            'brake', 'off_p50')
    st = {k: np.full(nb, np.nan) for k in keys}
    for i in range(nb):
        m = b == i
        if m.sum() < 5:
            continue
        st['n'][i] = m.sum()
        st['marg_p2'][i] = np.percentile(d['marg'][m], 2)
        st['marg_p5'][i] = np.percentile(d['marg'][m], 5)
        st['off_p95'][i] = np.percentile(np.abs(d['off'][m]), 95)
        st['off_p50'][i] = np.percentile(d['off'][m], 50)
        st['v_p50'][i] = np.percentile(d['v'][m], 50)
        st['ay_p95'][i] = np.percentile(d['ay'][m], 95)
        st['ay_p99'][i] = np.percentile(d['ay'][m], 99)
        st['brake'][i] = d['brake'][m].mean()
    return st, nb


def veto_bands(paths, line_xy, s, nbin_m, nb, halfwidth):
    """Bins within ``halfwidth`` m of a collision in a failed iteration."""
    total = s[-1] + (s[-1] - s[-2])
    veto = np.zeros(nb, bool)
    where = []
    for p in paths:
        log = racelog.load_log(p)
        for c in racelog.collision_rows(log):
            k = max(int(c) - 2, 0)
            j = racelog.project(line_xy, log['truth_xy'][k:k + 1])[0]
            where.append(float(s[j]))
    centres = (np.arange(nb) + 0.5) * nbin_m
    for sc in where:
        dd = np.abs(centres - sc)
        dd = np.minimum(dd, total - dd)
        veto |= dd <= halfwidth
    return veto, sorted(where)


def guard(v, v0, el, guard_brake):
    """Let a RAISE decay backwards at ``guard_brake`` toward the old profile.

    The forward/backward passes assume the plan's own brake (4.6), which the
    closed loop does not always deliver: a raise 6 m before a corner then
    arrives as extra entry speed.  This second pass sheds a raise at a
    deceleration the car demonstrably holds, and never drops below ``v0``.
    """
    if guard_brake <= 0:
        return v
    n = len(v)
    for _ in range(3):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v[i] = min(v[i], max(v0[i], np.sqrt(v[j] ** 2 + 2 * guard_brake * el[i])))
    return v


def consistent(v, kappa, el, ay_cap, ax, brake, vmax, passes=4):
    v = np.minimum(v, vmax)
    v = np.minimum(v, np.sqrt(ay_cap / np.maximum(np.abs(kappa), 1e-4)))
    n = len(v)
    for _ in range(passes):
        for i in range(n):
            j = (i + 1) % n
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2 * ax * el[i]))
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v[i] = min(v[i], np.sqrt(v[j] ** 2 + 2 * brake * el[i]))
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('line')
    ap.add_argument('--map', default='data/maps/finals_mesh')
    ap.add_argument('--log', action='append', default=[],
                    help='clean lap log (repeat); these are the safe set')
    ap.add_argument('--veto-log', action='append', default=[],
                    help='failed-iteration log; its collisions veto a raise')
    ap.add_argument('--veto-halfwidth', type=float, default=3.0)
    ap.add_argument('--bin', type=float, default=0.5, help='s bin width, m')
    ap.add_argument('--m-lo', type=float, default=0.30,
                    help='margin floor: below this the speed is CUT')
    ap.add_argument('--m-hi', type=float, default=0.45,
                    help='margin at which the full raise is allowed')
    ap.add_argument('--alpha', type=float, default=0.10, help='max speed raise')
    ap.add_argument('--beta', type=float, default=0.10, help='max speed cut')
    ap.add_argument('--feas', type=float, default=0.93,
                    help='driven/planned speed needed before a raise')
    ap.add_argument('--cte-veto', type=float, default=0.30,
                    help='no raise where driven |offset| p95 exceeds this')
    ap.add_argument('--smooth', type=float, default=1.5, help='gain smoothing, m')
    ap.add_argument('--ay-cap', type=float, default=9.0)
    ap.add_argument('--ax', type=float, default=6.5)
    ap.add_argument('--brake', type=float, default=4.6)
    ap.add_argument('--guard-brake', type=float, default=3.2,
                    help='a raise must be sheddable at this decel before the '
                         'next un-raised bin (0 disables)')
    ap.add_argument('--vmax', type=float, default=8.0)
    ap.add_argument('--report', action='store_true')
    ap.add_argument('--out', default='')
    a = ap.parse_args()

    s, xy, psi, kappa, v0 = racelog.load_line(a.line)
    el = np.diff(np.r_[s, s[-1] + (s[-1] - s[-2])])
    total = s[-1] + el[-1]
    dist_args = load_dist(a.map)

    d = gather(a.log, xy, dist_args)
    st, nb = bin_stats(d, s, a.bin, total)
    veto, coll_s = veto_bands(a.veto_log, xy, s, a.bin, nb, a.veto_halfwidth)

    bidx = np.clip(np.floor(s / a.bin).astype(int), 0, nb - 1)
    vplan_bin = np.array([v0[bidx == i].mean() if (bidx == i).any() else np.nan
                          for i in range(nb)])
    kap_bin = np.array([np.abs(kappa[bidx == i]).max() if (bidx == i).any()
                        else np.nan for i in range(nb)])
    feas = st['v_p50'] / np.maximum(vplan_bin, 1e-3)

    m = st['marg_p2']
    up = np.clip((m - a.m_lo) / max(a.m_hi - a.m_lo, 1e-6), 0.0, 1.0)
    dn = np.clip((a.m_lo - m) / max(a.m_lo, 1e-6), 0.0, 1.0)
    gate = np.isfinite(m) & (feas >= a.feas) & (st['off_p95'] <= a.cte_veto)
    gate &= ~veto
    # A raise is smoothed (it must not put a step in the profile); a CUT is
    # eroded instead, so that smoothing can never dilute the one bin that says
    # the car came close to a wall.
    rise = np.ones(nb)
    rise[gate] = 1.0 + a.alpha * up[gate]
    cut = np.ones(nb)
    bad = np.isfinite(m) & (dn > 0)
    cut[bad] = 1.0 - a.beta * dn[bad]

    sig = max(a.smooth / a.bin, 1e-3)
    half = max(int(round(a.smooth / a.bin)), 1)
    cutw = np.array([cut.take(range(i - half, i + half + 1), mode='wrap').min()
                     for i in range(nb)])
    gs = (gaussian_filter1d(rise, sig, mode='wrap')
          * gaussian_filter1d(cutw, sig * 0.5, mode='wrap'))
    gain = rise * cut
    centres = (np.arange(nb) + 0.5) * a.bin
    g = np.interp(s, centres, gs, period=total)

    v = consistent(v0 * g, kappa, el, a.ay_cap, a.ax, a.brake, a.vmax)
    v = guard(v, v0, el, a.guard_brake)
    vv = np.r_[v, v[0]]
    ax_prof = (vv[1:] ** 2 - vv[:-1] ** 2) / (2 * el)

    ideal0 = float(np.sum(el / v0))
    ideal = float(np.sum(el / v))
    print(f'{os.path.basename(a.line)}: ideal {ideal0:.3f} s  ->  {ideal:.3f} s '
          f'({ideal - ideal0:+.3f})')
    print(f'  speed gain  min {gs.min():.3f}  max {gs.max():.3f}  '
          f'moved {np.mean(np.abs(v / v0 - 1) > 0.005) * 100:.0f}% of nodes')
    print(f'  planned peak ay {np.max(v * v * np.abs(kappa)):.2f} '
          f'(was {np.max(v0 * v0 * np.abs(kappa)):.2f})   '
          f'v {v.min():.2f}-{v.max():.2f} (was {v0.min():.2f}-{v0.max():.2f})')
    if coll_s:
        print('  veto bands from failed logs at s = '
              + ', '.join(f'{x:.1f}' for x in coll_s))
    print(f'  samples {int(np.nansum(st["n"]))} from {len(a.log)} logs')

    if a.report:
        print(f'\n{"s":>6} {"kap":>5} {"vplan":>6} {"vdrv":>6} {"feas":>5} '
              f'{"wall2":>6} {"|e|95":>6} {"ay95":>5} {"brk":>4} {"gain":>6} '
              f'{"vnew":>6}')
        for i in range(nb):
            if not np.isfinite(st['n'][i]):
                continue
            flag = 'V' if veto[i] else ('+' if gs[i] > 1.002 else
                                        ('-' if gs[i] < 0.998 else ' '))
            j = int(np.argmin(np.abs(s - centres[i])))
            print(f'{centres[i]:6.2f} {kap_bin[i]:5.2f} {vplan_bin[i]:6.2f} '
                  f'{st["v_p50"][i]:6.2f} {feas[i]:5.2f} {m[i]:6.3f} '
                  f'{st["off_p95"][i]:6.3f} {st["ay_p95"][i]:5.2f} '
                  f'{st["brake"][i]:4.2f} {gs[i]:5.3f}{flag} {v[j]:6.2f}')

    if a.out:
        hdr = 's_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2'
        out = np.column_stack([s, xy[:, 0], xy[:, 1], psi, kappa, v, ax_prof])
        np.savetxt(a.out + '.csv', out, delimiter=',', fmt='%.4f',
                   header=hdr, comments='# ')
        print(f'  wrote {a.out}.csv')


if __name__ == '__main__':
    main()
