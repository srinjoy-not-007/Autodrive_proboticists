#!/usr/bin/env python3
"""Identify the published-command -> rack response from a race log, per period.

mpc3's rollout models the rack as a pure dead time plus a first-order lag
(`Actuator`: `dead_time` = `period + dead_tick_c`, `steer_tau`).  Everything in
that chain is a constant in SECONDS while every delay in this stack is a FRAME,
so the model can only be right at one sensor period.  This fits the two
constants from a driven log:

    steer_fb(t)  ~  lag( MAX_STEER * cmd(t - Td),  tau )

by a grid search on (Td, tau), and reports the fit at the log's own measured
period.  Run it on a 0.058 s log and a 0.098 s log and the period dependence of
(Td, tau) is read straight off -- that is the input to any period-scheduled
correction inside `mpc3_core`.

Usage:
  steer_residual_id.py <race log> [<race log> ...] [--kappa-band lo,hi]
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import racelog                                                   # noqa: E402


def resample(t, y, dt):
    """Uniform resample (zero-order hold) onto a dt grid."""
    grid = np.arange(t[0], t[-1], dt)
    idx = np.searchsorted(t, grid, 'right') - 1
    return grid, y[np.clip(idx, 0, len(y) - 1)]


def lag(u, dt, tau):
    if tau <= 1e-6:
        return u.copy()
    a = dt / (tau + dt)
    out = np.empty_like(u)
    acc = u[0]
    for i, v in enumerate(u):
        acc += a * (v - acc)
        out[i] = acc
    return out


def fit(log, dt=0.005, td_max=0.30, tau_max=0.20):
    t = log['t']
    keep = ~racelog.teleport_mask(log) & (log['lap'] >= 1)
    if keep.sum() < 100:
        keep = np.ones(log['n'], bool)
    t = t[keep]
    cmd = log['cmd_rad'][keep]
    fb = log['steer_fb_rad'][keep]
    g, u = resample(t, cmd, dt)
    _, y = resample(t, fb, dt)
    n = min(len(u), len(y))
    u, y = u[:n], y[:n]
    best = None
    for tau in np.arange(0.0, tau_max + 1e-9, 0.005):
        ul = lag(u, dt, tau)
        for k in range(0, int(td_max / dt) + 1):
            pred = np.r_[np.full(k, ul[0]), ul[:n - k]]
            r = float(np.sqrt(np.mean((pred - y) ** 2)))
            if best is None or r < best[0]:
                best = (r, k * dt, tau)
    rms, td, tau = best
    # gain: least squares on the best-aligned prediction
    ul = lag(u, dt, tau)
    k = int(round(td / dt))
    pred = np.r_[np.full(k, ul[0]), ul[:n - k]]
    gain = float(np.dot(pred, y) / max(np.dot(pred, pred), 1e-9))
    return rms, td, tau, gain, float(np.std(y))


def period(log):
    d = np.diff(log['t'][racelog.dedupe_ips_mask(log)])
    d = d[(d > 0.01) & (d < 0.5)]
    return float(np.median(d)) if len(d) else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+')
    a = ap.parse_args()
    print(f'{"log":<28}{"period":>8}{"Td":>8}{"tau":>7}{"gain":>7}'
          f'{"rms":>8}{"std(fb)":>9}{"Td/T":>7}')
    for p in a.logs:
        log = racelog.load_log(p)
        T = period(log)
        rms, td, tau, gain, sd = fit(log)
        print(f'{os.path.basename(p):<28}{T:>8.3f}{td:>8.3f}{tau:>7.3f}'
              f'{gain:>7.3f}{rms:>8.4f}{sd:>9.4f}{td / T:>7.2f}')


if __name__ == '__main__':
    main()
