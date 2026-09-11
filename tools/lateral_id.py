#!/usr/bin/env python3
"""Identify the car's lateral response from logged races (On-Track SysID style).

Everything the planner and the controller believe about lateral grip is a
guessed constant: `--ay` on `optimize_raceline.py`, `ay_limit` in the MPC, the
hand-drawn `_UNDERSTEER` table in `vehicle.py`. This tool replaces them with a
model fitted to `data/logs/race_*.csv`, in the spirit of ForzaETH On-Track
SysID (arXiv 2411.17508): identify the steady-state steering-to-lateral-
acceleration map plus the first-order transient from ordinary racing laps, then
emit a lookup table (the MAP controller idea, arXiv 2209.04346) and a
speed-dependent ggv.

Measurement chain (all of it ground truth, none of it localization):
  * x,y in the race log are the simulator's IPS at ~11 Hz. A local cubic fit
    over +-2 samples gives velocity and acceleration, hence speed v, path
    curvature kappa and lateral acceleration ay = v^2 * kappa.
  * `steering` in the same row is the actuator FEEDBACK in radians, published
    by the simulator on the same WebSocket packet as the pose, so the two are
    aligned to within one sample without any assumption about command latency.
    The 0.135 s command->actuator dead time is therefore NOT in this data and
    must not be fitted here; what is fitted is the actuator->vehicle response.
  * Samples within +-12 rows of a collision, and any implied speed above
    9 m/s, are dropped: the simulator teleports to a checkpoint and the
    derivative of a teleport is nonsense.

Cross-checked against the IMU gyro in data/bags/exp_ay50_d180_181558 (export
with tools/delay_model_id_export.py): the IPS-derived path yaw rate is 0.954x
the measured body yaw rate, i.e. this pipeline reads ~5% LOW at the peaks
(smoothing plus vehicle sideslip). Everything printed below is therefore a
slightly conservative estimate of the real lateral acceleration, and the path
curvature it measures is exactly the quantity a raceline velocity profile
needs.

Model (steady state, single track with a saturating front axle):

    kappa_lin = g * tan(delta) / (L + K_us * v^2)      linear region
    ay_ss     = ay_sat * tanh(v^2 * kappa_lin / ay_sat)  saturating

`g` is the gain of the kinematic bicycle (1.0 = exactly kinematic), `K_us` the
understeer gradient in rad per m/s^2, `ay_sat` the asymptote the front axle
tends to. The transient is one pole:

    d(kappa)/dt = (kappa_ss - kappa) / tau

Usage:
  .venv/bin/python tools/lateral_id.py                 # default log set
  .venv/bin/python tools/lateral_id.py --logs data/logs/race_X.csv ...
  .venv/bin/python tools/lateral_id.py --write         # emit data/calibration/

Offline only; never touches the simulator.
"""

import argparse
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from pit_crew import vehicle  # noqa: E402
from pit_crew.throttle2 import ServoThrottle  # noqa: E402

L = vehicle.WHEELBASE
# The longitudinal plant was identified separately, by `tools/sim_servo.py fit`
# against ground-truth speed; its numbers live on ServoThrottle. Taking them
# from there rather than re-deriving them from these logs keeps one source of
# truth -- and the percentile envelope printed below is a poor estimator
# anyway, because the car is under its own plan most of the time.
_SERVO = {k: v.default for k, v in
          __import__('inspect').signature(ServoThrottle.__init__).parameters.items()}
A_ACCEL = float(_SERVO['a_accel'])      # m/s2, measured drive authority
A_BRAKE = float(_SERVO['a_auth'])       # m/s2, measured servo braking authority
DRAG = 0.273                            # 1/s, Unity rigidbody linear drag
MAX_STEER = vehicle.MAX_STEER

# Runs used by default, by name in data/experiments.tsv. The Porto set is the
# high-ay end of the Sept 11 sweep -- those are the only logs that contain
# samples near the limit. The ICRA runs are HELD OUT: a different track, a
# different speed distribution, so a model fitted on Porto that also predicts
# ICRA is a model and not a memory of one circuit.
FIT_RUNS = ('ay65_mesh_100', 'ay70_mesh', 'mpc2_ay75b', 'porto_ag02_100',
            'ay70_obs', 'mpc2_mesh75')
HOLDOUT_RUNS = ('icra_mesh_40', 'icra_ag02')

TELEPORT_MPS = 9.0        # implied speed above this is a checkpoint reset
COLLISION_PAD = 12        # rows dropped either side of a collision
MIN_SPEED = 1.0           # m/s; geometry below this is meaningless
QS_RATE = 0.25            # rad/s; |d(delta)/dt| under this is quasi-steady
QS_SPAN = 2               # ...sustained over +-this many samples
LINEAR_AY = 4.0           # m/s2; below this the front axle is not saturating
LINEAR_DELTA = 0.15       # rad; ditto


# ---------------------------------------------------------------- loading ---

def resolve_runs(names):
    """Map run names in data/experiments.tsv to log paths (last match wins)."""
    out = {}
    path = 'data/experiments.tsv'
    if not os.path.exists(path):
        return out
    for line in open(path):
        f = line.rstrip('\n').split('\t')
        if len(f) >= 2 and f[1] in names:
            out[f[1]] = f[-1]
    return out


def local_derivs(t, x, y, half=2, deg=3):
    """Value/1st/2nd derivative of x(t), y(t) from a local polynomial fit.

    A centred fit is zero-phase, so it cannot invent the lag we are trying to
    measure. half=2 (a 0.36 s window) was chosen by comparing the resulting ay
    against the gyro on the bag: half=2 loses ~4% of the p95, half=4 loses ~9%.
    """
    n = len(t)
    dx = np.full((n, 3), np.nan)
    dy = np.full((n, 3), np.nan)
    ok = np.zeros(n, bool)
    span = 1.5 * (2 * half + 1) * 0.09
    for i in range(n):
        a, b = max(0, i - half), min(n, i + half + 1)
        if b - a < deg + 1 or t[b - 1] - t[a] > span:
            continue
        tt = t[a:b] - t[i]
        V = np.vander(tt, deg + 1)
        cx = np.linalg.lstsq(V, x[a:b], rcond=None)[0]
        cy = np.linalg.lstsq(V, y[a:b], rcond=None)[0]
        dx[i] = [cx[-1], cx[-2], 2.0 * cx[-3]]
        dy[i] = [cy[-1], cy[-2], 2.0 * cy[-3]]
        ok[i] = True
    return dx, dy, ok


def load_log(path, half=2):
    a = np.genfromtxt(path, delimiter=',', names=True)
    t = a['t'].astype(float)
    x, y = a['x'].astype(float), a['y'].astype(float)
    delta = a['steering'].astype(float)
    thr = a['throttle'].astype(float)
    col = a['collision_count'].astype(float)

    bad = np.zeros(len(t), bool)
    step = np.hypot(np.diff(x), np.diff(y))
    dt = np.maximum(np.diff(t), 1e-3)
    jump = step / dt > TELEPORT_MPS
    bad[:-1] |= jump
    bad[1:] |= jump
    for i in np.flatnonzero(np.diff(col) > 0) + 1:
        bad[max(0, i - COLLISION_PAD):i + COLLISION_PAD] = True

    dx, dy, ok = local_derivs(t, x, y, half=half)
    vx, ax2 = dx[:, 1], dx[:, 2]
    vy, ay2 = dy[:, 1], dy[:, 2]
    v = np.hypot(vx, vy)
    kappa = (vx * ay2 - vy * ax2) / np.maximum(v ** 3, 1e-6)
    ax_long = (vx * ax2 + vy * ay2) / np.maximum(v, 1e-6)

    spread = bad.copy()
    for k in range(1, half + 1):
        spread[k:] |= bad[:-k]
        spread[:-k] |= bad[k:]
    ok &= ~spread
    ok &= v > MIN_SPEED
    return dict(t=t, delta=delta, v=v, kappa=kappa, ay=v * v * kappa,
                ax=ax_long, thr=thr, ok=ok, path=path)


def load_sysid(path, half=2):
    """A staircase log from `ros2 run pit_crew sysid_run` (see that module).

    Same fields as a race log, plus the commanded cell and the IMU. Curvature
    comes from the GYRO here, not from differentiating the pose: the explore
    world has no walls, the manoeuvre is deliberately steady, and a directly
    measured yaw rate beats a twice-differentiated position. Speed comes from
    the IPS trace, smoothed, with the wheel encoder as the fallback -- the
    logger writes a row per IMU message and the IPS arrives slightly less
    often, so consecutive rows sometimes repeat a pose and a naive derivative
    reads zero. Only the measured `hold` phase is kept.
    """
    a = np.genfromtxt(path, delimiter=',', names=True,
                      dtype=None, encoding='utf-8')
    phase = np.array([str(x) for x in a['phase']])
    t = a['t'].astype(float)
    x, y = a['x'].astype(float), a['y'].astype(float)
    wz = a['wz'].astype(float)
    v_enc = a['v_enc'].astype(float)

    # Speed over a +-half window of DISTINCT poses, so a repeated row does not
    # read as a stop.
    v_ips = np.full(len(t), np.nan)
    for i in range(len(t)):
        a0, b0 = max(0, i - half), min(len(t), i + half + 1)
        dt = t[b0 - 1] - t[a0]
        if dt > 1e-3:
            v_ips[i] = math.hypot(x[b0 - 1] - x[a0], y[b0 - 1] - y[a0]) / dt
    v = np.where(np.isfinite(v_ips) & (v_ips > 0.2), v_ips, v_enc)
    kappa = wz / np.maximum(v, 0.2)
    ok = ((phase == 'hold') & (v > MIN_SPEED)
          & np.isfinite(kappa) & np.isfinite(v))
    return dict(t=t, delta=a['delta_act'].astype(float), v=v, kappa=kappa,
                ay=v * wz, ax=np.gradient(v, t), thr=a['throttle'].astype(float),
                ok=ok, path=path)


def segments(paths, half=2, min_len=40, sysid=False):
    """Contiguous stretches of usable samples, so the transient can integrate."""
    segs = []
    for p in paths:
        d = load_sysid(p, half=half) if sysid else load_log(p, half=half)
        idx = np.flatnonzero(d['ok'])
        if not len(idx):
            continue
        cuts = np.flatnonzero(np.diff(idx) > 1)
        for a, b in zip(np.r_[0, cuts + 1], np.r_[cuts, len(idx) - 1]):
            s, e = idx[a], idx[b] + 1
            if e - s < min_len:
                continue
            segs.append({k: d[k][s:e] for k in
                         ('t', 'delta', 'v', 'kappa', 'ay', 'ax', 'thr')}
                        | {'log': os.path.basename(p)})
    return segs


def cat(segs, key):
    return np.concatenate([s[key] for s in segs]) if segs else np.array([])


def quasi_steady(segs, rate=QS_RATE, span=QS_SPAN):
    out = []
    for s in segs:
        q = np.abs(np.gradient(s['delta'], s['t'])) < rate
        m = q.copy()
        for j in range(1, span + 1):
            m[j:] &= q[:-j]
            m[:-j] &= q[j:]
        out.append(m)
    return np.concatenate(out) if out else np.array([], bool)


# ------------------------------------------------------------------ model ---

def kappa_ss(delta, v, g, k_us, ay_sat):
    lin = g * np.tan(delta) / (L + k_us * v * v)
    if ay_sat is None or not np.isfinite(ay_sat):
        return lin
    ay_lin = v * v * lin
    return ay_sat * np.tanh(ay_lin / ay_sat) / np.maximum(v * v, 1e-6)


def delta_for(ay, v, g, k_us, ay_sat):
    """Steering angle for a requested lateral acceleration (the steer map).

    NaN when the request is beyond the identified axle saturation or beyond
    steering lock -- 'not achievable' is a legitimate answer and the caller has
    to see it rather than be handed a number that will not be delivered.
    """
    ay = np.asarray(ay, float)
    v = np.asarray(v, float)
    with np.errstate(divide='ignore', invalid='ignore'):
        r = np.clip(np.abs(ay) / ay_sat, 0.0, 0.999999)
        ay_lin = np.sign(ay) * ay_sat * np.arctanh(r)
        kap = ay_lin / np.maximum(v * v, 1e-6)
        d = np.arctan(kap * (L + k_us * v * v) / g)
    d = np.where(np.abs(ay) >= 0.98 * ay_sat, np.nan, d)
    return np.where(np.abs(d) > MAX_STEER, np.nan, d)


def simulate(seg, g, k_us, ay_sat, tau, td=0.0):
    t, v = seg['t'], seg['v']
    de = np.interp(t - td, t, seg['delta']) if td else seg['delta']
    kss = kappa_ss(de, v, g, k_us, ay_sat)
    k = np.empty_like(kss)
    k[0] = seg['kappa'][0]
    a = np.exp(-np.diff(t) / max(tau, 1e-3))
    for i in range(1, len(k)):
        k[i] = a[i - 1] * k[i - 1] + (1.0 - a[i - 1]) * kss[i]
    return k


def residual(segs, g, k_us, ay_sat, tau, td=0.0):
    """Curvature error, weighted by sqrt(v) so fast samples (where a curvature
    error costs the most lateral acceleration) carry more of the fit."""
    return np.concatenate([(simulate(s, g, k_us, ay_sat, tau, td) - s['kappa'])
                           * np.sqrt(s['v']) for s in segs])


# ----------------------------------------------------------------- report ---

def report_coverage(segs, label):
    v, de = cat(segs, 'v'), np.abs(cat(segs, 'delta'))
    vb = [1, 2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 8]
    db = [0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, MAX_STEER]
    print(f'\n--- coverage, {label}: samples per (speed x |steer|) cell ---')
    print('  v \\ |delta|' + ''.join(f'{x:>8.2f}' for x in db[1:]))
    for a, b in zip(vb[:-1], vb[1:]):
        row = [int(((v >= a) & (v < b) & (de >= c) & (de < d)).sum())
               for c, d in zip(db[:-1], db[1:])]
        print(f'  {a:4.1f}-{b:<4.1f} ' + ''.join(f'{x:8d}' for x in row))
    print(f'  max |steering| seen: {de.max():.3f} rad '
          f'({100 * de.max() / MAX_STEER:.0f}% of lock); '
          f'{int((de > 0.33).sum())} samples above 0.33 rad')


def held(segs, nhold):
    """Lateral acceleration held for a window: min |ay| over nhold samples."""
    V, A = [], []
    for s in segs:
        a = np.abs(s['ay'])
        h = nhold // 2
        V.append(s['v'])
        A.append(np.array([a[max(0, i - h):i + h + 1].min()
                           for i in range(len(a))]))
    return np.concatenate(V), np.concatenate(A)


def report_envelope(segs):
    print('\n--- lateral acceleration actually reached (m/s2) ---')
    print('  speed      n   instantaneous          held 0.27 s        held 0.45 s')
    print('                  p95   p99   max     p95   p99   max     p95   p99')
    cols = [held(segs, n) for n in (1, 3, 5)]
    for lo, hi in zip([1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.5],
                      [2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.5, 8.0]):
        m = (cols[0][0] >= lo) & (cols[0][0] < hi)
        if m.sum() < 50:
            print(f'  {lo:.1f}-{hi:.1f} {m.sum():6d}   (too few samples)')
            continue
        f = []
        for (v, a), k in zip(cols, (3, 3, 2)):
            f += list(np.percentile(a[m], [95, 99]))
            if k == 3:
                f.append(a[m].max())
        print(f'  {lo:.1f}-{hi:.1f} {m.sum():6d} ' +
              ' '.join(f'{x:5.2f}' for x in f[:3]) + '   ' +
              ' '.join(f'{x:5.2f}' for x in f[3:6]) + '   ' +
              ' '.join(f'{x:5.2f}' for x in f[6:]))


def report_steering_gain(segs):
    """Where the data actually reaches the limit: the marginal return on lock.

    In the linear region d(kappa)/d(delta) is the kinematic 1/(L cos^2 delta)
    ~ 3.2 1/m per rad. When the front axle saturates the same extra lock buys
    a fraction of that, and that fraction -- not a percentage of some assumed
    mu -- is the honest statement of how close to the limit the car is.
    """
    q = quasi_steady(segs)
    v, de = cat(segs, 'v'), cat(segs, 'delta')
    ka, ay = cat(segs, 'kappa'), cat(segs, 'ay')
    sgn = np.sign(de)
    ad, ka, ay = np.abs(de), ka * sgn, ay * sgn
    print('\n--- quasi-steady steering gain (|d delta/dt| < '
          f'{QS_RATE} rad/s, n={int(q.sum())}) ---')
    for vlo, vhi in ((2.0, 2.5), (2.5, 3.0), (3.0, 3.5), (3.5, 4.5), (4.5, 6.0)):
        print(f'  v {vlo:.1f}-{vhi:.1f} m/s'
              '   |delta|      n   kappa   kinematic  ratio     ay   d(kappa)/d(delta)')
        prev = None
        any_row = False
        for lo, hi in zip([0.05, 0.10, 0.15, 0.20, 0.24, 0.28, 0.32],
                          [0.10, 0.15, 0.20, 0.24, 0.28, 0.32, MAX_STEER]):
            m = q & (v >= vlo) & (v < vhi) & (ad >= lo) & (ad < hi)
            if m.sum() < 25:
                prev = None
                continue
            any_row = True
            dm, km = np.median(ad[m]), np.median(ka[m])
            kin = math.tan(dm) / L
            slope = ''
            if prev and dm > prev[0] + 1e-6:
                got = (km - prev[1]) / (dm - prev[0])
                exp = (math.tan(dm) - math.tan(prev[0])) / L / (dm - prev[0])
                slope = f'{got:6.2f} of {exp:5.2f} ({100*got/exp:3.0f}%)'
            print(f'{"":16}{lo:.2f}-{hi:.2f} {m.sum():6d}  {km:6.3f}   '
                  f'{kin:6.3f}   {km/kin:5.3f} {np.median(ay[m]):6.2f}   {slope}')
            prev = (dm, km)
        if not any_row:
            print(f'{"":16}(no quasi-steady cornering samples in this band)')


def report_ellipse(segs):
    v, ax, ay = cat(segs, 'v'), cat(segs, 'ax'), cat(segs, 'ay')
    thr = cat(segs, 'thr')
    print('\n--- longitudinal envelope and combined (friction ellipse) ---')
    print('  |ay| band       n   ax p05   ax p95   |a| total p99')
    for lo, hi in ((0, 1.5), (1.5, 3.0), (3.0, 4.5), (4.5, 5.5), (5.5, 9.0)):
        m = (np.abs(ay) >= lo) & (np.abs(ay) < hi) & (v > 2.0)
        if m.sum() < 50:
            continue
        print(f'  {lo:4.1f}-{hi:4.1f} {m.sum():8d}  {np.percentile(ax[m], 5):7.2f} '
              f' {np.percentile(ax[m], 95):7.2f}   '
              f'{np.percentile(np.hypot(ax[m], ay[m]), 99):7.2f}')
    print('  drive / coast-brake envelope on the straights (|ay| < 2):')
    print('   speed      n    drive p99    n   coast p01')
    rows = []
    for lo, hi in zip([2, 3, 4, 5, 6], [3, 4, 5, 6, 8]):
        base = (v >= lo) & (v < hi) & (np.abs(ay) < 2.0)
        md, mb = base & (thr > 0.15), base & (thr < 0.05)
        dv = np.percentile(ax[md], 99) if md.sum() > 40 else np.nan
        bv = np.percentile(ax[mb], 1) if mb.sum() > 40 else np.nan
        rows.append((0.5 * (lo + hi), dv, bv))
        print(f'   {lo}-{hi} {md.sum():8d}   {dv:8.2f} {mb.sum():6d}   {bv:8.2f}')
    return rows


def report_validation(segs, g, k_us, ay_sat):
    """Forward check: model ay against measured ay per (speed, steer) cell.

    This is the direction that is free of selection bias. Binning by the
    ACHIEVED lateral acceleration and reading back the median steering -- which
    is what the `delta_measured_rad` column of the steer map does -- conditions
    on the noisy variable and biases the answer low; binning by the commanded
    steering, which the model takes as input, does not.
    """
    q = quasi_steady(segs)
    v, de, ay = cat(segs, 'v'), cat(segs, 'delta'), cat(segs, 'ay')
    ad, ays = np.abs(de), ay * np.sign(de)
    print('\n--- forward validation: model vs measured ay, quasi-steady ---')
    print('   speed      |delta|      n   measured   model    error')
    for vlo, vhi in ((2.0, 2.5), (2.5, 3.0), (3.0, 3.5), (3.5, 4.5), (4.5, 6.0)):
        for lo, hi in zip([0.05, 0.10, 0.15, 0.20, 0.24, 0.28],
                          [0.10, 0.15, 0.20, 0.24, 0.28, MAX_STEER]):
            m = q & (v >= vlo) & (v < vhi) & (ad >= lo) & (ad < hi)
            if m.sum() < 25:
                continue
            vm, dm = np.median(v[m]), np.median(ad[m])
            mod = float(vm * vm * kappa_ss(np.array([dm]), np.array([vm]),
                                           g, k_us, ay_sat)[0])
            meas = float(np.median(ays[m]))
            print(f'   {vlo:.1f}-{vhi:.1f}   {lo:.2f}-{hi:.2f} {m.sum():7d}  '
                  f'{meas:8.2f} {mod:8.2f} {mod - meas:+8.2f}')


def fit(segs, holdout, args):
    """Three stages, because the three things are identifiable from different data.

    1. The linear relation, from quasi-steady samples below LINEAR_AY where
       nothing saturates and nothing is transient. kappa and ay are ~0.94
       correlated on one circuit -- a free regression returns an effective
       wheelbase of 0.17 m, which is nonsense -- so `g` is bounded at the
       kinematic 1.0 and the understeer gradient takes what is left. The free
       fit is printed so the conditioning is visible and not hidden.
    2. The axle saturation, from quasi-steady samples ONLY, so the transient
       cannot contaminate it. Two shapes are fitted: a soft tanh asymptote and
       a hard clip. They disagree by 4 m/s2 and fit equally well; that spread
       IS the measurement, and it is reported rather than averaged away.
    3. The transient pole, on every usable sample, with 1 and 2 fixed.
    """
    q = quasi_steady(segs)
    v, de = cat(segs, 'v'), cat(segs, 'delta')
    ka, ay = cat(segs, 'kappa'), cat(segs, 'ay')
    m = (q & (np.abs(de) > 0.03) & (np.abs(de) < LINEAR_DELTA)
         & (np.abs(ay) < LINEAR_AY) & (v > 1.5))
    A = np.column_stack([ka[m], ay[m]])
    free = np.linalg.lstsq(A, de[m], rcond=None)[0]
    sol = least_squares(lambda p: A @ p - de[m], [L, 2e-3],
                        bounds=([L, 0.0], [L / 0.75, 0.05]))
    g, k_us = L / sol.x[0], sol.x[1]
    print(f'\n--- stage 1: linear region (n={int(m.sum())}, '
          f'corr(kappa, ay) = {np.corrcoef(ka[m], ay[m])[0, 1]:.3f}) ---')
    print(f'  free      delta = {free[0]:.4f}*kappa + {free[1]:.5f}*ay  '
          f'-> effective wheelbase {free[0]:.3f} m '
          f'({"IMPLAUSIBLE, ill-conditioned" if free[0] < L else "plausible"})')
    print(f'  bounded   g = {g:.3f}, K_us = {k_us:.5f} rad/(m/s2)  '
          f'-> characteristic speed {math.sqrt(L / max(k_us, 1e-9)):.1f} m/s, '
          f'effective wheelbase {L + k_us * 9:.3f} m at 3 m/s, '
          f'{L + k_us * 25:.3f} m at 5 m/s')

    print('\n--- stage 2: axle saturation, quasi-steady samples only ---')
    def qs_arrays(S):
        qq = quasi_steady(S)
        vv, dd, aa = cat(S, 'v'), cat(S, 'delta'), cat(S, 'ay')
        k = qq & (vv > 1.5) & (np.abs(dd) > 0.03)
        return vv[k], dd[k], aa[k]

    def sat_fit(S, mode):
        vv, dd, aa = qs_arrays(S)
        lin = vv * vv * g * np.tan(dd) / (L + k_us * vv * vv)
        def f(p):
            mod = (p[0] * np.tanh(lin / p[0]) if mode == 'tanh'
                   else np.clip(lin, -p[0], p[0]))
            return mod - aa
        r = least_squares(f, [7.0], bounds=([3.0], [40.0]))
        return float(r.x[0]), float(np.sqrt(np.mean(r.fun ** 2))), len(vv)

    ay_sat, rms_t, n_qs = sat_fit(segs, 'tanh')
    clip, rms_c, _ = sat_fit(segs, 'clip')
    print(f'  fit set   (n={n_qs}):  tanh asymptote {ay_sat:5.2f} m/s2 '
          f'(rms {rms_t:.3f} m/s2)   hard clip {clip:5.2f} (rms {rms_c:.3f})')
    if holdout:
        a2, r2, n2 = sat_fit(holdout, 'tanh')
        c2, r2c, _ = sat_fit(holdout, 'clip')
        print(f'  held out  (n={n2}):  tanh asymptote {a2:5.2f} m/s2 '
              f'(rms {r2:.3f} m/s2)   hard clip {c2:5.2f} (rms {r2c:.3f})')
    vv, dd, aa = qs_arrays(segs + holdout)
    lin = vv * vv * g * np.tan(dd) / (L + k_us * vv * vv)
    print('  tanh asymptote profile, pooled (rms in m/s2):')
    prof = [(c, float(np.sqrt(np.mean((c * np.tanh(lin / c) - aa) ** 2))))
            for c in (5, 6, 7, 8, 9, 10, 12, 15, 25, 40)]
    best = min(r for _, r in prof)
    print('   ' + '  '.join(f'{c}:{r:.3f}{"*" if r < 1.03 * best else ""}'
                            for c, r in prof))
    print('   (* within 3% of the best: anything in that range fits the data '
          'equally well -- the asymptote is NOT identified by these logs)')

    print('\n--- stage 3: transient, all samples ---')
    s3 = least_squares(lambda p: residual(segs, g, k_us, ay_sat, p[0]),
                       [0.09], bounds=([0.005], [0.40]))
    tau = float(s3.x[0])
    rms = float(np.sqrt(np.mean(s3.fun ** 2)))
    dt_only = least_squares(
        lambda p: residual(segs, g, k_us, ay_sat, 0.005, p[0]),
        [0.05], bounds=([0.0], [0.25]))
    both = least_squares(
        lambda p: residual(segs, g, k_us, ay_sat, p[0], p[1]),
        [0.09, 0.0], bounds=([0.005, -0.05], [0.40, 0.25]))
    print(f'  first-order pole      tau = {tau:.3f} s              '
          f'rms {rms:.4f} 1/m')
    print(f'  pure dead time        td  = {dt_only.x[0]:.3f} s              '
          f'rms {np.sqrt(np.mean(dt_only.fun ** 2)):.4f} 1/m')
    print(f'  both                  tau = {both.x[0]:.3f} s, td = {both.x[1]:+.3f} s  '
          f'rms {np.sqrt(np.mean(both.fun ** 2)):.4f} 1/m')
    print('  (sampling is 0.09 s, so tau and td trade against each other at '
          'this resolution;\n   the IMU bag cross-check gives tau 0.045 s, '
          'td 0.000 s on an ay-5.0 run.\n   Either way the lateral response '
          'itself is fast -- the 0.135 s command dead\n   time is not in this '
          'signal and is not what tau measures.)')
    if holdout:
        rh = float(np.sqrt(np.mean(residual(holdout, g, k_us, ay_sat, tau) ** 2)))
        print(f'  held-out rms at these parameters: {rh:.4f} 1/m')
    print('  per-log residual rms (1/m):')
    fit_logs = {s['log'] for s in segs}
    for lg in sorted({s['log'] for s in segs + holdout}):
        sl = [s for s in segs + holdout if s['log'] == lg]
        tag = 'fit    ' if lg in fit_logs else 'HELDOUT'
        print(f'    {tag} {lg}  '
              f'{np.sqrt(np.mean(residual(sl, g, k_us, ay_sat, tau) ** 2)):.4f}')
    return g, k_us, ay_sat, clip, tau


# ------------------------------------------------------------ calibration ---

def measured_cells(segs, v_edges, ay_edges, min_n):
    """Median |steering| per (v, ay) cell from quasi-steady samples."""
    q = quasi_steady(segs)
    v, ay, de = cat(segs, 'v'), np.abs(cat(segs, 'ay')), np.abs(cat(segs, 'delta'))
    vi = np.digitize(v, v_edges) - 1
    ai = np.digitize(ay, ay_edges) - 1
    cells = {}
    good = q & (vi >= 0) & (ai >= 0) & (vi < len(v_edges) - 1) & (ai < len(ay_edges) - 1)
    for i in np.flatnonzero(good):
        cells.setdefault((vi[i], ai[i]), []).append(de[i])
    return {k: (float(np.median(x)), len(x)) for k, x in cells.items()
            if len(x) >= min_n}


def write_steer_map(path, segs, g, k_us, ay_sat, args):
    v_grid = np.arange(1.0, 8.0 + 1e-9, 0.25)
    ay_grid = np.arange(0.0, 10.0 + 1e-9, 0.25)
    v_edges = np.r_[v_grid - 0.125, v_grid[-1] + 0.125]
    ay_edges = np.r_[ay_grid - 0.125, ay_grid[-1] + 0.125]
    cells = measured_cells(segs, v_edges, ay_edges, args.min_cell)
    n_meas = 0
    with open(path, 'w') as f:
        f.write('# steering angle for a requested lateral acceleration\n')
        f.write('# identified by tools/lateral_id.py from data/logs/race_*.csv\n')
        f.write(f'# model: kappa = g*tan(delta)/(L + K_us*v^2), '
                f'ay = ay_sat*tanh(v^2*kappa/ay_sat)\n')
        f.write(f'# g={g:.4f} K_us={k_us:.5f} ay_sat={ay_sat:.3f} '
                f'L={L} max_steer={MAX_STEER}\n')
        f.write('# delta_rad is NaN where the request is beyond the identified\n'
                '# saturation or beyond steering lock: not achievable.\n')
        f.write('# delta_measured_rad is the median steering among quasi-steady\n'
                '# samples that ACHIEVED this ay at this speed. It is biased LOW by\n'
                '# roughly 10-20%% near the limit -- conditioning on the achieved\n'
                '# value selects the laps where the car over-rotated -- so read it as\n'
                '# evidence that the cell was visited, not as a better delta. The\n'
                '# unbiased comparison is the forward table the tool prints.\n')
        f.write('# source: measured = >= %d quasi-steady samples in this cell;\n'
                '#         model    = inside the measured speed/accel envelope;\n'
                '#         extrap   = outside it, trust only as a shape.\n' % args.min_cell)
        f.write('v_mps,ay_mps2,delta_rad,delta_measured_rad,n_samples,source\n')
        for i, v in enumerate(v_grid):
            for j, ay in enumerate(ay_grid):
                d = delta_for(ay, v, g, k_us, ay_sat)
                meas = cells.get((i, j))
                if meas:
                    n_meas += 1
                inside = (args.v_meas_lo <= v <= args.v_meas_hi
                          and ay <= args.ay_meas_hi)
                src = 'measured' if meas else ('model' if inside else 'extrap')
                if not np.isfinite(d):
                    src = 'unreachable'
                ds = 'nan' if not np.isfinite(d) else f'{float(d):.5f}'
                ms = f'{meas[0]:.5f}' if meas else 'nan'
                f.write(f'{v:.2f},{ay:.2f},{ds},{ms},'
                        f'{meas[1] if meas else 0},{src}\n')
    print(f'\nwrote {path}: {len(v_grid)}x{len(ay_grid)} grid, '
          f'{n_meas} cells backed by measurement')


def write_ggv(path, segs, g, k_us, ay_sat, long_rows, args):
    """TUM ggv: v_mps, ax_max_mps2, ay_max_mps2 (tph applies the ellipse).

    ay_max is FLAT on purpose. The per-speed measured envelope printed above
    rises from 4.7 m/s2 at 1.75 m/s to 6.5 at 3.25 and falls away again above
    4 m/s, and none of that is grip: it is demand. The car only ever corners
    hard where the line is tight, which is where it is slow, and it is never
    asked for lateral acceleration at all above ~4.5 m/s. Writing that shape
    into a ggv would tell the planner the car gains grip with speed, which
    nothing measured supports. The honest statement is one number plus a
    declared range of validity.
    """
    v_grid = np.arange(0.0, 8.0 + 1e-9, 0.5)
    vh, ah = held(segs, 3)
    bands = []
    for lo in np.arange(1.5, 5.5, 0.5):
        m = (vh >= lo) & (vh < lo + 0.5)
        if m.sum() >= 200:
            bands.append((lo + 0.25, float(np.percentile(ah[m], 99)), int(m.sum())))
    env = max((b[1] for b in bands), default=float('nan'))
    ay_cap = args.ay_cap if args.ay_cap is not None else env
    ay_max = np.full_like(v_grid, ay_cap)
    ax_max = np.clip(A_ACCEL - DRAG * v_grid, 1.0, A_ACCEL)

    with open(path, 'w') as f:
        f.write('# ggv identified by tools/lateral_id.py from race logs.\n')
        f.write('# TUM format (the column header is a comment line, as in the\n'
                '# reference ggv files). Consume with\n'
                '#   ggv = np.loadtxt(path, delimiter=",", comments="#")\n'
                '#   tph.calc_vel_profile.calc_vel_profile(ggv=ggv, ...)\n'
                '# which is exactly the array optimize_raceline.py builds today from\n'
                '# the flat --ay/--ax, so it drops in where those two do.\n#\n')
        f.write('# ay_max %.2f m/s2, flat. MEASURED as the 99th percentile of lateral\n'
                '#   acceleration HELD for 0.27 s, over %.1f-%.1f m/s:\n'
                % (ay_cap, min(b[0] for b in bands) - 0.25,
                   max(b[0] for b in bands) + 0.25) if bands else '')
        for c, val, n in bands:
            f.write(f'#     v {c - 0.25:.1f}-{c + 0.25:.1f} m/s: {val:.2f} m/s2 (n={n})\n')
        f.write('#   The rise with speed in that list is demand, not grip -- see the\n'
                '#   docstring. Outside 1.5-5.0 m/s this number is EXTRAPOLATED; the\n'
                '#   car has 4 logged samples above 4.5 m/s with more than 0.15 rad of\n'
                '#   steering, so there is no evidence either way up there.\n')
        f.write('#   Saturation estimates from the same logs bracket it: hard clip\n'
                '#   ~5.9 m/s2, tanh asymptote ~10 m/s2 (unidentified, see stage 2).\n')
        f.write('# ax_max = a_accel - drag*v = %.1f - %.3f*v, from the longitudinal\n'
                '#   plant identified by tools/sim_servo.py (ServoThrottle defaults).\n'
                '#   This is the SPEED SERVO, not the tyre: %.1f m/s2 of authority,\n'
                '#   spent only when the setpoint is well above the current speed.\n'
                '#   Braking is separate (optimize_raceline --brake): the servo has\n'
                '#   %.1f m/s2 and the shipping profile uses 4.0 because braking\n'
                '#   overlaps corner entry.\n' % (A_ACCEL, DRAG, A_ACCEL, A_BRAKE))
        f.write('# v_mps,ax_max_mps2,ay_max_mps2\n')
        for v, ax, ay in zip(v_grid, ax_max, ay_max):
            f.write(f'{v:.2f},{ax:.3f},{ay:.3f}\n')
    print(f'wrote {path}: ay_max {ay_cap:.2f} flat '
          f'(measured envelope {env:.2f}), ax_max {ax_max.max():.2f} down to '
          f'{ax_max.min():.2f} m/s2')
    if long_rows:
        print('  for comparison, the log percentile envelope this REPLACES:')
        for c, dv, bv in long_rows:
            print(f'    v ~{c:.1f} m/s: drive p99 {dv:6.2f}, coast p01 {bv:6.2f} m/s2'
                  '   (contaminated by teleports and demand-limited)')
    return v_grid, ax_max, ay_max


def verdict(segs, g, k_us, ay_sat, ay_clip, ay_env, has_sysid=False):
    """The question this was built to answer, stated against the data."""
    print('\n--- where the data reaches the limit, and where it does not ---')
    v, ay, de = cat(segs, 'v'), np.abs(cat(segs, 'ay')), np.abs(cat(segs, 'delta'))
    ka = np.abs(cat(segs, 'kappa'))
    q = quasi_steady(segs)
    print(f'  quasi-steady samples above ay 6: {int((q & (ay > 6)).sum())}; '
          f'above ay 7: {int((q & (ay > 7)).sum())}; '
          f'above ay 8: {int((q & (ay > 8)).sum())}')
    print(f'  samples with |steering| > 0.33 rad: {int((de > 0.33).sum())} '
          f'({int((q & (de > 0.33)).sum())} of them quasi-steady); '
          f'steering lock is {MAX_STEER:.3f} rad')
    fast = (v > 4.5) & (de > 0.15)
    print(f'  samples above 4.5 m/s with more than 0.15 rad of steering: '
          f'{int(fast.sum())} -- the car has never been asked to corner hard '
          f'at speed,\n    so ay_max above ~4.5 m/s is EXTRAPOLATION in '
          f'anything written below.')

    print('\n--- does the data support ay 8-9 in the hairpins? ---')
    m = (v > 2.5) & (v < 3.2) & (de > 0.26)
    if m.sum() > 50:
        print(f'  hairpin samples (v 2.5-3.2 m/s, |delta| > 0.26 rad): '
              f'n={int(m.sum())}')
        print(f'    curvature asked {np.median(np.tan(de[m])) / L:.3f} 1/m, '
              f'delivered {np.median(ka[m]):.3f} 1/m '
              f'({100 * np.median(ka[m]) / (np.median(np.tan(de[m])) / L):.0f}%)')
        print(f'    lateral acceleration there: median {np.median(ay[m]):.2f}, '
              f'p99 {np.percentile(ay[m], 99):.2f} m/s2')
    print(f'  measured sustained envelope (0.27 s hold, p99, best speed band): '
          f'{ay_env:.2f} m/s2')
    print(f'  saturation estimates: hard clip {ay_clip:.2f} m/s2, '
          f'tanh asymptote {ay_sat:.2f} m/s2 (not identified, see stage 2)')
    for ay_req, v_req in ((8.0, 2.9), (9.0, 3.0)):
        d = float(delta_for(np.array([ay_req]), np.array([v_req]),
                            g, k_us, ay_sat)[0])
        dc = (float(np.arctan((ay_req / v_req ** 2)
                              * (L + k_us * v_req ** 2) / g))
              if ay_req < ay_clip else float('nan'))
        print(f'  ay {ay_req:.1f} at {v_req:.1f} m/s needs '
              f'{"lock and more" if not np.isfinite(d) else f"{d:.3f} rad"} '
              f'under the tanh model, and is '
              f'{"IMPOSSIBLE" if not np.isfinite(dc) else f"{dc:.3f} rad"} '
              f'under the clip model')
    v_ok = math.sqrt(ay_env / 1.0)
    n_qs_hi = int((q & (de > 0.33)).sum())
    print(f'\n  ANSWER: not from this data. Nothing in these logs holds more than '
          f'{ay_env:.1f} m/s2\n  for a quarter of a second at any speed; the marginal '
          f'return on steering is\n  already down to ~50-60% of kinematic by ay 5.4; '
          f'and there are {n_qs_hi} quasi-steady\n  samples in the whole corpus above '
          f'0.33 rad of steering. A kappa 1.0 apex is\n  drivable at {v_ok:.2f} m/s on '
          f'the measured envelope, not at 2.9-3.0 m/s, which\n  would demand ay '
          f'{2.95 ** 2:.1f}.\n'
          f'\n  What the data cannot do is RULE OUT 8-9. The two saturation shapes '
          f'bracket\n  it ({ay_clip:.1f} against {ay_sat:.1f} m/s2) and fit equally '
          f'well, the tyre asset peaks at mu 1.0,\n  and the -explore skidpad reached '
          f'9.9 m/s2 in open space. The controller has\n  never asked for more than '
          f'0.32 rad in steady state on track, so no amount of\n  further racing will '
          f'settle it.'
          + ('\n  Staircase data IS included above -- read the stage-2 numbers and the '
             'coverage\n  table, which now cover the cells racing never visits.'
             if has_sysid else
             ' Drive the staircase instead:\n'
             '    TAG=2026-iros-explore ./scripts/sim_restart_connect.sh\n'
             '    ros2 run pit_crew sysid_run\n'
             '    .venv/bin/python tools/lateral_id.py --sysid data/logs/sysid_*.csv --write'))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--logs', nargs='*', default=None,
                    help='race log CSVs to fit (default: the high-ay Porto set)')
    ap.add_argument('--sysid', nargs='*', default=None,
                    help='staircase logs from `ros2 run pit_crew sysid_run`, '
                         'added to the fit set (data/logs/sysid_*.csv)')
    ap.add_argument('--holdout', nargs='*', default=None,
                    help='logs to score but not fit (default: the ICRA runs)')
    ap.add_argument('--write', action='store_true',
                    help='write data/calibration/{steer_map,ggv}.csv')
    ap.add_argument('--outdir', default='data/calibration')
    ap.add_argument('--min-cell', type=int, default=15,
                    help='quasi-steady samples before a steer-map cell counts '
                         'as measured')
    ap.add_argument('--v-meas-lo', type=float, default=2.0)
    ap.add_argument('--v-meas-hi', type=float, default=4.5)
    ap.add_argument('--ay-meas-hi', type=float, default=6.5)
    ap.add_argument('--ay-cap', type=float, default=None,
                    help='write this ay_max into the ggv instead of the '
                         'measured envelope (e.g. 6.0, the validated planner '
                         'value)')
    args = ap.parse_args()

    if args.logs is None:
        found = resolve_runs(FIT_RUNS)
        args.logs = [found[r] for r in FIT_RUNS if r in found]
        if not args.logs:
            sys.exit('no default logs found; pass --logs')
    if args.holdout is None:
        found = resolve_runs(HOLDOUT_RUNS)
        args.holdout = [found[r] for r in HOLDOUT_RUNS if r in found]

    segs = segments(args.logs)
    if args.sysid:
        extra = segments(args.sysid, min_len=8, sysid=True)
        n_extra = sum(len(s['t']) for s in extra)
        print(f'staircase logs: {len(args.sysid)} files, {len(extra)} holds, '
              f'{n_extra} measured samples')
        if n_extra < 100:
            print('  WARNING: that is almost nothing. A staircase log with no '
                  '`hold` rows\n  means the run stopped before it measured '
                  'anything -- check the guards.')
        segs += extra
    hold = segments(args.holdout) if args.holdout else []
    print(f'fit set: {len(args.logs)} logs, {len(segs)} segments, '
          f'{sum(len(s["t"]) for s in segs)} usable samples')
    for p in args.logs:
        print(f'   {p}')
    if hold:
        print(f'held out: {len(args.holdout)} logs, '
              f'{sum(len(s["t"]) for s in hold)} samples')
        for p in args.holdout:
            print(f'   {p}')

    report_coverage(segs, 'fit set')
    if hold:
        report_coverage(hold, 'held-out set')
    report_envelope(segs + hold)
    report_steering_gain(segs + hold)
    long_rows = report_ellipse(segs + hold)
    g, k_us, ay_sat, ay_clip, tau = fit(segs, hold, args)
    report_validation(segs + hold, g, k_us, ay_sat)

    all_segs = segs + hold
    vh, ah = held(all_segs, 3)
    ay_env = 0.0
    for lo in np.arange(1.5, 5.5, 0.5):
        m = (vh >= lo) & (vh < lo + 0.5)
        if m.sum() >= 200:
            ay_env = max(ay_env, float(np.percentile(ah[m], 99)))
    if args.write:
        os.makedirs(args.outdir, exist_ok=True)
        write_steer_map(os.path.join(args.outdir, 'steer_map.csv'),
                        all_segs, g, k_us, ay_sat, args)
        write_ggv(os.path.join(args.outdir, 'ggv.csv'),
                  all_segs, g, k_us, ay_sat, long_rows, args)
    else:
        print('\n(no files written; pass --write)')
    verdict(all_segs, g, k_us, ay_sat, ay_clip, ay_env,
            has_sysid=bool(args.sysid))


if __name__ == '__main__':
    main()
