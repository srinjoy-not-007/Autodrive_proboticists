#!/usr/bin/env python3
"""Identify the PUBLISHED-COMMAND -> RACK transfer from a race log, per regime.

`tools/steer_residual_id.py` fits one (dead time, tau) pair over a whole log
with a pure first-order lag and no slew limit.  This tool does three more
things, all of which the Sep 20 chain audit needed:

* it simulates the model the code actually contains -- dead time, first-order
  lag AND the 3.2 rad/s slew clamp, stepped exactly as `Actuator.advance` does
  -- so the fit can be compared against `Actuator` parameters directly;
* it evaluates the residual on a MASK, so the same trace can be fitted on
  straights (|cmd| small), on mid corners and at the hairpins (|cmd| at lock)
  and the answers compared.  A chain that is the same in all three is linear;
* it reports the decomposition (dead-only, lag-only, joint) and the effective
  lag `Td + tau`, which is the number the solver's model has to match.

The fit is column-to-column in LOG time: `cmd` is the published command as the
race logger saw it and `steering` is the simulator's rack feedback in the same
row.  Both columns are the latest value at the row's timestamp, so the absolute
dead time carries the logger's own sampling grid (+-T/2); the PERIOD DEPENDENCE
of the fit does not, which is what the audit is after.

Usage:
  steer_chain_id.py <race log> [...] [--regimes] [--slew] [--s-window lo,hi]
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import racelog                                                   # noqa: E402

try:
    from pit_crew import vehicle                                 # noqa: E402
except ImportError:                                              # pragma: no cover
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
    from pit_crew import vehicle                                 # noqa: E402

FINE_DT = 0.002


def _zoh(t_src, y_src, grid):
    idx = np.searchsorted(t_src, grid, 'right') - 1
    return y_src[np.clip(idx, 0, len(y_src) - 1)]


def _lag_linear(u, dt, tau):
    """Exact zero-order-hold first-order response (no slew clamp)."""
    if tau <= 1e-9:
        return u.copy()
    a = 1.0 - math.exp(-dt / tau)
    out = np.empty_like(u)
    acc = u[0]
    for i in range(len(u)):
        acc += a * (u[i] - acc)
        out[i] = acc
    return out


def _lag_slew(u, dt, tau, rate):
    """`Actuator.advance`'s law: first-order step, clamped to `rate * dt`."""
    a = 1.0 if tau <= 1e-9 else 1.0 - math.exp(-dt / tau)
    step = rate * dt
    out = np.empty_like(u)
    acc = u[0]
    for i in range(len(u)):
        d = a * (u[i] - acc)
        if d > step:
            d = step
        elif d < -step:
            d = -step
        acc += d
        out[i] = acc
    return out


def _lag_linear_fast(u, dt, tau):
    if tau <= 1e-9:
        return u.copy()
    try:
        from scipy.signal import lfilter
    except ImportError:                                          # pragma: no cover
        return _lag_linear(u, dt, tau)
    b = 1.0 - math.exp(-dt / tau)
    y = lfilter([b], [1.0, -(1.0 - b)], u - u[0]) + u[0]
    return y


def prepare(log, s=None, s_window=None):
    """Fine-grid command, row times, measured rack, and per-row masks."""
    keep = ~racelog.teleport_mask(log) & (log['lap'] >= 1) \
        & racelog.dedupe_ips_mask(log)
    if keep.sum() < 200:
        keep = racelog.dedupe_ips_mask(log)
    t = log['t'][keep]
    cmd = log['cmd_rad'][keep]
    fb = log['steer_fb_rad'][keep]
    v = log['enc_speed'][keep] if 'enc_speed' in log else np.zeros_like(t)
    # drop rows whose timestamp gap is not a sane telemetry tick
    dt = np.diff(t, prepend=t[0] - 0.05)
    good = (dt > 0.005) & (dt < 0.5)
    grid = np.arange(t[0] - 0.4, t[-1] + 1e-9, FINE_DT)
    u = _zoh(t, cmd, grid)
    out = dict(t=t, cmd=cmd, fb=fb, v=v, grid=grid, u=u, good=good)
    if s is not None:
        out['s'] = s[keep]
        if s_window is not None:
            lo, hi = s_window
            out['good'] = good & (out['s'] >= lo) & (out['s'] <= hi)
    return out


def sim_at_rows(P, td, tau, rate=None):
    """Model rack angle sampled at the log row times."""
    if rate is None:
        y = _lag_linear_fast(P['u'], FINE_DT, tau)
    else:
        y = _lag_slew(P['u'], FINE_DT, tau, rate)
    tg = P['grid'] + td
    return np.interp(P['t'], tg, y)


def fit(P, mask=None, td_max=0.22, tau_max=0.16, rate=None,
        td_step=0.002, tau_step=0.002, free_gain=True):
    """Grid (Td, tau); the static gain is least-squares at every grid point.

    Leaving the gain free matters: with it pinned at 1 the grid buys fit by
    trading lag against amplitude, and the straight-line regime (where the
    rack's measured gain is 0.93) comes out with a longer Td than it has.
    """
    m = P['good'] if mask is None else (P['good'] & mask)
    if m.sum() < 50:
        return None
    y = P['fb'][m]
    best = None
    taus = np.arange(0.0, tau_max + 1e-9, tau_step)
    tds = np.arange(0.0, td_max + 1e-9, td_step)
    for tau in taus:
        if rate is None:
            yl = _lag_linear_fast(P['u'], FINE_DT, tau)
        else:
            yl = _lag_slew(P['u'], FINE_DT, tau, rate)
        for td in tds:
            pred = np.interp(P['t'][m], P['grid'] + td, yl)
            if free_gain:
                g = float(np.dot(pred, y) / max(np.dot(pred, pred), 1e-12))
            else:
                g = 1.0
            r = float(np.sqrt(np.mean((g * pred - y) ** 2)))
            if best is None or r < best[0]:
                best = (r, float(td), float(tau), g)
    rms, td, tau, gain = best
    return dict(rms=rms, td=td, tau=tau, lag=td + tau, gain=gain,
                n=int(m.sum()), std=float(np.std(y)))


def period(log):
    d = np.diff(log['t'][racelog.dedupe_ips_mask(log)])
    d = d[(d > 0.01) & (d < 0.5)]
    return float(np.median(d)) if len(d) else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--regimes', action='store_true',
                    help='split the fit by |cmd| band')
    ap.add_argument('--slew', action='store_true',
                    help='include the 3.2 rad/s slew clamp in the model')
    ap.add_argument('--line', default=None,
                    help='raceline csv, to enable --s-window')
    ap.add_argument('--s-window', default=None, help='lo,hi arc length, m')
    a = ap.parse_args()
    rate = vehicle.STEER_RATE if a.slew else None
    sw = None
    if a.s_window:
        sw = tuple(float(x) for x in a.s_window.split(','))
    print(f'{"log":<26}{"T":>7}{"regime":>10}{"n":>7}{"Td":>7}{"tau":>7}'
          f'{"Td+tau":>8}{"gain":>7}{"rms":>8}{"std":>7}')
    for p in a.logs:
        log = racelog.load_log(p)
        T = period(log)
        s = None
        if a.line:
            sfull, pts = racelog.load_line(a.line)[:2]
            s = sfull[racelog.project(pts, log['truth_xy'])]
        P = prepare(log, s=s, s_window=sw)
        bands = [('all', None)]
        if a.regimes:
            c = np.abs(P['cmd'])
            dc = np.abs(np.gradient(P['cmd'], P['t']))
            bands += [('|c|<.10', c < 0.10),
                      ('.10-.30', (c >= 0.10) & (c < 0.30)),
                      ('>=.30', c >= 0.30),
                      ('ramp>1', dc > 1.0),
                      ('hold', dc < 0.3)]
        for name, mask in bands:
            f = fit(P, mask, rate=rate)
            if f is None:
                continue
            print(f'{os.path.basename(p):<26}{T:>7.3f}{name:>10}{f["n"]:>7}'
                  f'{f["td"]:>7.3f}{f["tau"]:>7.3f}{f["lag"]:>8.3f}'
                  f'{f["gain"]:>7.3f}{f["rms"]:>8.4f}{f["std"]:>7.3f}')


if __name__ == '__main__':
    main()
