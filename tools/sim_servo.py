#!/usr/bin/env python3
"""Identify the simulator's throttle servo, then tune a throttle law against it.

The simulator's throttle is not a torque: it is a SPEED SERVO whose setpoint is
`SERVO_GAIN * throttle`.  That single fact changes what a throttle law should
do.  A feedforward law (`u = 0.040*v_target + 0.005`) hands the servo a setpoint
equal to the speed we want *now*, so the servo only ever accelerates as hard as
its own proportional gain times the error it happens to have -- which is small,
because the setpoint is where we already want to be.  The servo has ~6 m/s2 in
both directions, but only pays it when the setpoint is far from the current
speed.  To use that authority you must deliberately over-command the setpoint.

Two jobs, two subcommands:

  fit   -- identify  dv/dt = clip(K*(SERVO_GAIN*u(t-tau) - v), -dmax, amax)
                            - drag*v   from race logs (ground-truth x,y).
  tune  -- close the loop offline: run the fitted plant around a velocity
           profile at the real 90 ms command period, with the real encoder
           filter, comparing the shipping law against `ServoThrottle`.

Everything here is offline and reads only logs; it never touches a container.

    .venv/bin/python tools/sim_servo.py fit data/logs/race_*.csv
    .venv/bin/python tools/sim_servo.py tune /tmp/x.csv   # ay-6.0 profile
"""

import argparse
import glob
import os
import sys

import numpy as np
from scipy.signal import savgol_filter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src'))
from pit_crew.throttle2 import (ServoThrottle, SERVO_GAIN,        # noqa: E402
                                brake_limited_target)

DT = 0.09              # s, the simulator consumes one command per telemetry tick
TELEPORT_M = 0.9       # a step larger than this is a collision reset, not motion


# --------------------------------------------------------------------------
# logs -> uniformly sampled ground-truth speed
# --------------------------------------------------------------------------

def load_run(path, dt=DT):
    """Uniformly resampled (t, v_true, u, v_enc) plus teleport cut indices.

    `speed` in the log is the encoder estimate the controller saw; the truth is
    d/dt of the logged IPS x,y.  Log timestamps jitter (0.063-0.122 s), so
    differentiate the cumulative path length on a uniform grid and smooth with a
    short Savitzky-Golay window -- a raw two-point derivative of a 90 ms pose
    carries ~2 m/s2 of noise, enough to swamp the fit.
    """
    d = np.genfromtxt(path, delimiter=',', names=True)
    t = d['t'] - d['t'][0]
    keep = np.concatenate([[True], np.diff(t) > 1e-3])
    d, t = d[keep], t[keep]
    x, y = d['x'], d['y']
    step = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0.0], np.cumsum(step)])
    tg = np.arange(0.0, t[-1], dt)
    v = np.gradient(savgol_filter(np.interp(tg, t, s), 7, 2), dt)
    u = np.interp(tg, t, d['throttle'])
    enc = np.interp(tg, t, d['speed'])
    cuts = [0] + [int(tt / dt) for tt in t[1:][step > TELEPORT_M]] + [len(tg)]
    return tg, v, u, enc, cuts


def segments(paths, dt=DT, min_len=50):
    """Continuous (v, u) stretches with no teleport in them."""
    segs = []
    for p in paths:
        _, v, u, _, cuts = load_run(p, dt)
        for a, b in zip(cuts, cuts[1:]):
            a, b = a + 3, b - 3          # the savgol window straddles a cut
            if b - a >= min_len:
                segs.append((v[a:b], u[a:b]))
    return segs


# --------------------------------------------------------------------------
# the plant
# --------------------------------------------------------------------------

def plant_step(v, u_delayed, K, amax, dmax, drag, dt=DT, sub=3):
    """One command period of the servo, integrated in `sub` substeps.

    Scalars or arrays: passing arrays integrates a whole population of
    parameter sets at once, which is what makes the grid search affordable.
    """
    h = dt / sub
    for _ in range(sub):
        a = np.clip(K * (SERVO_GAIN * u_delayed - v), -dmax, amax) - drag * v
        v = np.maximum(0.0, v + a * h)
    return v


def _delayed(u, tau, dt=DT):
    """The command in force at each tick, `tau` seconds after it was issued."""
    n = len(u)
    f = np.arange(n) - tau / dt
    j = np.floor(f).astype(int)
    w = f - j
    ud = u[np.clip(j, 0, n - 1)] * (1 - w) + u[np.clip(j + 1, 0, n - 1)] * w
    ud[f < 0] = u[0]
    return ud


def simulate(u, v0, K, tau, amax, dmax, drag, dt=DT):
    """Open-loop speed for a recorded command sequence (scalar parameters)."""
    ud = _delayed(u, tau, dt)
    v = np.empty(len(u))
    v[0] = v0
    for i in range(len(u) - 1):
        v[i + 1] = plant_step(v[i], ud[i], K, amax, dmax, drag, dt)
    return v


def _sse_batch(segs, K, tau, amax, dmax, drag, dt=DT):
    """Sum of squared errors for a population of parameter sets, one tau."""
    tot = np.zeros(np.shape(K))
    for v, u in segs:
        ud = _delayed(u, tau, dt)
        vs = np.full(np.shape(K), v[0], dtype=float)
        for i in range(len(v) - 1):
            vs = plant_step(vs, ud[i], K, amax, dmax, drag, dt)
            tot = tot + (vs - v[i + 1]) ** 2
    return tot


def fit(paths, dt=DT):
    segs = segments(paths, dt)
    ns = sum(len(v) for v, _ in segs)
    print('%d segments, %d samples (%.0f s of driving)'
          % (len(segs), ns, ns * dt))

    def rmse_many(cands):
        """cands: list of (K, tau, amax, dmax, drag) -> array of RMSE."""
        out = np.empty(len(cands))
        taus = sorted({c[1] for c in cands})
        for tau in taus:
            idx = [i for i, c in enumerate(cands) if c[1] == tau]
            a = np.array([cands[i] for i in idx], dtype=float)
            sse = _sse_batch(segs, a[:, 0], tau, a[:, 2], a[:, 3], a[:, 4], dt)
            out[idx] = np.sqrt(sse / ns)
        return out

    grid = [(K, tau, amax, dmax, drag)
            for K in (1.0, 1.5, 2.0, 3.0, 4.5, 6.0, 9.0, 14.0)
            for tau in (0.0, 0.045, 0.09, 0.135, 0.18)
            for amax in (4.0, 5.0, 6.0, 7.0)
            for dmax in (4.0, 5.0, 6.0, 7.0)
            for drag in (0.0, 0.273)]
    e = rmse_many(grid)
    k = int(np.argmin(e))
    best, bp = float(e[k]), list(grid[k])
    print('  grid best  RMSE %.3f  at K=%.1f tau=%.3f amax=%.1f dmax=%.1f '
          'drag=%.3f' % (best, *bp))

    steps = [1.0, 0.045, 0.5, 0.5, 0.1]
    for _ in range(5):
        cands = []
        for j in range(5):
            for sgn in (-1, 1):
                c = list(bp)
                c[j] = max(0.0, c[j] + sgn * steps[j])
                cands.append(tuple(c))
        e = rmse_many(cands)
        k = int(np.argmin(e))
        if e[k] < best:
            best, bp = float(e[k]), list(cands[k])
        else:
            steps = [s * 0.5 for s in steps]
    K, tau, amax, dmax, drag = bp
    print('fitted:  K=%.2f 1/s  tau=%.3f s  a_max=%.2f  a_min=-%.2f  drag=%.3f'
          % (K, tau, amax, dmax, drag))
    print('         RMSE %.3f m/s' % best)
    allv = np.concatenate([v for v, _ in segs])
    allu = np.concatenate([u for _, u in segs])
    print('  baseline  v = mean                 RMSE %.3f m/s' % allv.std())
    print('  baseline  v = %.1f*u, no dynamics  RMSE %.3f m/s'
          % (SERVO_GAIN, float(np.sqrt(np.mean((SERVO_GAIN * allu - allv) ** 2)))))
    return dict(K=K, tau=tau, amax=amax, dmax=dmax, drag=drag, rmse=best)


# --------------------------------------------------------------------------
# closed loop around a velocity profile
# --------------------------------------------------------------------------

class LegacyThrottle:
    """What ships today: `vehicle.ThrottleController` with brake_gain 0.05."""

    def __init__(self, ff=0.040, idle=0.005, brake_gain=0.05,
                 deadband=0.5, brake_max=0.45, brake_min_speed=1.0):
        self.ff, self.idle = ff, idle
        self.bg, self.db = brake_gain, deadband
        self.bmax, self.bmin_v = brake_max, brake_min_speed

    def command(self, v_target, v_meas=None, a_target=0.0):
        u = self.ff * v_target + self.idle
        if v_meas is not None:
            err = v_target - v_meas
            if err < -self.db and self.bg > 0.0 and v_meas > self.bmin_v:
                u = max(-self.bmax, self.bg * err)
        return float(np.clip(u, -1.0, 1.0))


def run_lap(profile, law, plant, laps=4, dt=DT, brake_horizon_s=0.35,
            enc_alpha=0.35, delay=0.135, target='horizon', a_brake=4.0,
            look_m=8.0, use_kappa=True):
    """Drive the fitted plant round `profile` with `law`; report the lap.

    Longitudinal only: the path is fixed, so progress `s` is the integral of
    the car's own speed and the plan is looked up at `s`.

    `delay` is the publish->apply dead time (0.135 s measured), NOT the `tau`
    the fit returns: the fit aligns the speed against the THROTTLE COLUMN OF
    THE LOG, which is already the command in force at that telemetry tick, so
    its tau comes out at zero.  Kept separate on purpose -- conflating them
    would hide a real 1.5 command periods of lag.

    `target='horizon'` reproduces `mpc.control_step` exactly (minimum profile
    speed over `brake_horizon_s * v` of track); `target='kinematic'` uses
    `throttle2.brake_limited_target` over `look_m` metres instead.
    """
    v_p, a_p, L = profile['v'], profile['a'], profile['L']
    spacing = float(np.median(np.diff(profile['s'])))
    n_look = max(2, int(look_m / spacing))
    ds_ahead = np.arange(n_look) * spacing
    v = float(v_p[0])
    s, v_enc, t = 0.0, v, 0.0
    n_delay = max(0, int(round(delay / dt)))
    cmd_hist = [0.0] * (n_delay + 1)
    rec = []
    total = laps * L
    while s < total and t < 400.0:
        i0 = int((s % L) / spacing) % len(v_p)
        here = float(v_p[i0])
        if target == 'kinematic':
            idx = (i0 + np.arange(n_look)) % len(v_p)
            v_target = min(here, brake_limited_target(v_p[idx], ds_ahead, a_brake))
        else:
            n_ahead = max(2, int(max(0.2, v * brake_horizon_s) / spacing))
            idx = (i0 + np.arange(n_ahead)) % len(v_p)
            v_target = float(v_p[idx].min())
        a_target = float(a_p[i0]) if v_target >= here - 1e-3 else 0.0
        u = (law.command(v_target, v_enc, a_target, now=t,
                         kappa=(profile['kappa'][i0] if use_kappa else None))
             if isinstance(law, ServoThrottle)
             else law.command(v_target, v_enc, a_target))
        cmd_hist.append(u)
        v_prev = v
        v = plant_step(v, cmd_hist[-1 - n_delay], plant['K'], plant['amax'],
                       plant['dmax'], plant['drag'], dt)
        s += 0.5 * (v_prev + v) * dt
        v_enc += enc_alpha * (v_prev - v_enc)     # one tick old, alpha filtered
        rec.append((t, s, v, v_target, here, u))
        t += dt
    rec = np.array(rec)
    lap_t = []
    for k in range(1, laps + 1):
        j = np.searchsorted(rec[:, 1], k * L)
        j0 = np.searchsorted(rec[:, 1], (k - 1) * L)
        if j < len(rec):
            lap_t.append(rec[j, 0] - rec[j0, 0])
    steady = rec[rec[:, 1] > L]                   # drop the standing start
    err = steady[:, 2] - steady[:, 4]
    return dict(rec=rec, laps=lap_t,
                lap=float(np.median(lap_t[1:])) if len(lap_t) > 1 else float('nan'),
                rms=float(np.sqrt(np.mean(err ** 2))), mean=float(err.mean()),
                over=float(err.max()), under=float(err.min()),
                u_min=float(steady[:, 5].min()),
                frac_neg=float(np.mean(steady[:, 5] < 0.0)))


def profile_from_csv(path):
    rl = np.loadtxt(path, delimiter=',')
    s = rl[:, 0]
    L = float(s[-1] + (s[-1] - s[-2]))
    return dict(s=s, v=rl[:, 5], a=rl[:, 6], L=L,
                xy=rl[:, 1:3], kappa=rl[:, 4])


def report(name, r, profile):
    print('%-26s lap %6.3f s   speed err mean %+5.2f rms %4.2f  '
          'worst over %+5.2f under %+5.2f   u<0 %4.1f%%'
          % (name, r['lap'], r['mean'], r['rms'], r['over'], r['under'],
             100 * r['frac_neg']))


def overspeed_at(rec, profile, s_lo, s_hi, L):
    """Worst overspeed inside a given stretch of track, all laps."""
    s = rec[:, 1] % L
    m = (s >= s_lo) & (s <= s_hi) & (rec[:, 1] > L)
    if not m.any():
        return 0.0, 0.0
    spacing = float(np.median(np.diff(profile['s'])))
    i = (s[m] / spacing).astype(int) % len(profile['v'])
    d = rec[m, 2] - profile['v'][i]
    return float(d.max()), float(np.mean(d))


def tune(profile_csv, plant, laps=5, delay=0.135):
    prof = profile_from_csv(profile_csv)
    ideal = float(np.sum(np.diff(prof['s']) / prof['v'][:-1]))
    print('\nprofile %s: %.1f m, v %.2f-%.2f m/s, ideal lap %.3f s, '
          'dead time %.3f s' % (os.path.basename(profile_csv), prof['L'],
                                prof['v'].min(), prof['v'].max(), ideal, delay))
    print('%-34s %-10s %7s %6s %6s %6s %6s %7s'
          % ('law', 'target', 'lap', 'mean', 'rms', 'over', 'under', 'u<0'))

    def row(name, law, **kw):
        r = run_lap(prof, law, plant, laps=laps, delay=delay, **kw)
        print('%-34s %-10s %7.3f %+6.2f %6.2f %+6.2f %+6.2f %6.1f%%'
              % (name, kw.get('target', 'horizon'), r['lap'], r['mean'],
                 r['rms'], r['over'], r['under'], 100 * r['frac_neg']))
        return r

    def servo(**kw):
        kw.setdefault('k_ff', 1.0)
        return ServoThrottle(K=plant['K'], drag=plant['drag'], tau=delay,
                             a_auth=plant['dmax'], a_accel=plant['amax'], **kw)

    base = row('legacy ff + brake_gain 0.05', LegacyThrottle())
    row('legacy ff, brake_gain 0', LegacyThrottle(brake_gain=0.0))
    print()
    best = None
    for lead_max in (0.0, 0.15, 0.25, 0.35, 0.50, 1.50):
        r = row('servo lead_max %.2f' % lead_max, servo(lead_max=lead_max))
        # An overspeed is a collision risk at the ay the profile already
        # spends, so price it far above the lap time it buys.
        score = r['lap'] + 4.0 * max(0.0, r['over'] - 0.10)
        if best is None or score < best[0]:
            best = (score, lead_max, r)
    _, lead_max, r = best
    print('\nchosen (lap + 4 * overspeed beyond 0.10 m/s): lead_max %.2f'
          % lead_max)

    print('\nrobustness of the chosen law:')
    for d in (0.09, 0.135, 0.18, 0.27):
        rr = run_lap(prof, servo(lead_max=lead_max), plant, laps=laps, delay=d)
        print('  plant dead time %.3f (law assumes %.3f): lap %.3f  '
              'worst overspeed %+.2f' % (d, delay, rr['lap'], rr['over']))
    for am, dm in ((4.0, 4.0), (plant['amax'], plant['dmax']), (6.0, 6.0)):
        pl = dict(plant, amax=am, dmax=dm)
        rr = run_lap(prof, servo(lead_max=lead_max), pl, laps=laps, delay=delay)
        print('  plant authority +%.1f/-%.1f: lap %.3f  worst overspeed %+.2f'
              % (am, dm, rr['lap'], rr['over']))

    print('\nthe brake point is the caller\'s, not the law\'s: same servo law, '
          'kinematic target')
    for a_brake in (2.5, 4.0):
        row('servo lead_max %.2f, a_brake %.1f' % (lead_max, a_brake),
            servo(lead_max=lead_max), target='kinematic', a_brake=a_brake)
    for bh in (0.5, 0.9):
        row('servo lead_max %.2f, horizon %.2f s' % (lead_max, bh),
            servo(lead_max=lead_max), brake_horizon_s=bh)

    print('\nworst overspeed vs the plan, by stretch (the hairpin is s 10-14):')
    for name, rr in (('legacy', base), ('servo', r)):
        line = '  %-16s' % name
        for lo, hi in ((0, 6), (6, 10), (10, 14), (14, 20), (20, 30)):
            o, m = overspeed_at(rr['rec'], prof, lo, hi, prof['L'])
            line += '  s%2d-%2d %+5.2f' % (lo, hi, o)
        print(line)
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=('fit', 'tune', 'all'))
    ap.add_argument('paths', nargs='*')
    ap.add_argument('--profile', default=None)
    ap.add_argument('--laps', type=int, default=5)
    ap.add_argument('--delay', type=float, default=0.135,
                    help='publish->apply dead time, s (measured 0.135)')
    a = ap.parse_args()
    plant = dict(K=18.0, tau=0.0, amax=5.0, dmax=4.5, drag=0.0)
    if a.mode in ('fit', 'all'):
        paths = []
        for p in (a.paths or ['data/logs/race_*.csv']):
            paths += sorted(glob.glob(p))
        plant.update(fit(paths))
    if a.mode in ('tune', 'all'):
        prof = a.profile or (a.paths[0] if a.mode == 'tune' else None)
        if not prof:
            ap.error('tune needs a velocity-profile csv')
        tune(prof, plant, laps=a.laps, delay=a.delay)


if __name__ == '__main__':
    main()
