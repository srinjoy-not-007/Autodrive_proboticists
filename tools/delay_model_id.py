#!/usr/bin/env python3
"""Identify the dead-time prediction model the MPC should use.

The shipping controller (`src/pit_crew/mpc.py`) predicts the pose forward by
`cmd_delay_s` with a kinematic bicycle driven by the *commanded* steering
history. Live, that prediction lands ~0.11 m OUTSIDE the car's true path in the
hairpins and agrees with it on the straights: the real car rotates MORE than
the model says it does.

This tool replays a recorded bag OFFLINE and scores candidate prediction models
against IPS ground truth, so the question is answered from data instead of from
lap times. For every sample it starts at the TRUE pose, predicts H seconds
ahead, and compares against the TRUE pose at t+H.

    M0  constant heading (straight-line dead reckoning)
    M1  gyro-held yaw rate
    M2  bicycle, ACTUAL steering feedback held constant over H
    M3  bicycle, steering feedback shifted by the 0.135 s command dead time
        (i.e. what the actuator will do given commands already in flight,
        approximated by time-shifting the feedback trace)
    M4  M3 with the scene-measured wheelbase 0.3312 m
    M5  M3 with a fitted yaw-rate gain (fitted on the hairpin bins)
    M6  M3 with vehicle.understeer_ratio() scaling
    M7  gyro-held yaw rate + the change in commanded curvature (the 'gyro'
        delay model in mpc.py, which made things worse live)
    M8  the SHIPPING composite: a gyro-held leg over the scan->now latency,
        then the bicycle driven by the steering the MPC believes is acting --
        the command published `cmd_delay` (0.18 s) earlier, which reaches the
        actuator DEAD_TIME later, i.e. the feedback trace at t - 0.045 s.

Sign convention (stated once, used everywhere):
    error = predicted_pose(t+H) - true_pose(t+H), rotated into the TRUE
    heading frame at t+H. +lateral = prediction is to the LEFT of the true car;
    +longitudinal = prediction is AHEAD of the true car. In a right-hand corner
    (kappa < 0) +lateral therefore means the prediction is on the OUTSIDE.

Input: CSVs exported from the bag inside the devkit container (rclpy lives only
there). Write EXPORT_SRC below to /tmp in the container, `docker cp` it, run it,
`docker cp` the output directory back, then point this tool at it:

    docker cp export_bag.py autodrive_roboracer_api:/tmp/export_bag.py
    docker exec autodrive_roboracer_api bash -c \
      'source /opt/ros/humble/setup.bash && \
       python3 /tmp/export_bag.py /home/data/bags/<bag> /tmp/bagcsv'
    docker cp autodrive_roboracer_api:/tmp/bagcsv/. ./bagcsv/
    .venv/bin/python tools/delay_model_id.py ./bagcsv

Offline use only; it never touches the simulator.
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from pit_crew import vehicle  # noqa: E402

EXPORT_SRC = 'tools/delay_model_id_export.py'   # container-side exporter

DEAD_TIME = 0.135      # s, measured command -> actuator-feedback dead time
HORIZONS = (0.09, 0.18, 0.27)
KAPPA_HAIRPIN = 0.6    # |kappa| above this is a hairpin bin
MIN_SPEED = 1.0        # m/s, below this the geometry is meaningless


def load_csv(d, name):
    p = os.path.join(d, name + '.csv')
    if not os.path.exists(p):
        sys.exit(f'missing {p} (export the bag first, see the docstring)')
    rows = np.genfromtxt(p, delimiter=',', names=True)
    return np.atleast_1d(rows)


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def yaw_from_quat(x, y, z, w):
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def interp_at(t_src, v_src, t_q):
    return np.interp(t_q, t_src, v_src)


def interp_angle(t_src, a_src, t_q):
    """Interpolate an angle without wrapping artefacts."""
    return np.arctan2(np.interp(t_q, t_src, np.sin(a_src)),
                      np.interp(t_q, t_src, np.cos(a_src)))


def encoder_speed(t, pos):
    """Wheel speed from the cumulative wheel angle, glitch-filtered.

    The cumulative angle steps backwards on ~7% of samples (see vehicle.py);
    reject implausible derivatives and hold the previous reference so the next
    sample differentiates cleanly across the gap.
    """
    v = np.zeros_like(t)
    last_t, last_p, last_v = t[0], pos[0], 0.0
    for i in range(1, len(t)):
        dt = t[i] - last_t
        if dt <= 1e-4:
            v[i] = last_v
            continue
        cand = vehicle.WHEEL_RADIUS * (pos[i] - last_p) / dt
        if -vehicle.REVERSE_MAX <= cand <= vehicle.SPEED_MAX:
            last_t, last_p, last_v = t[i], pos[i], cand
        v[i] = last_v
    v[0] = v[1] if len(v) > 1 else 0.0
    return v


def load_run(d):
    ips = load_csv(d, 'ips')
    imu = load_csv(d, 'imu')
    enc_l = load_csv(d, 'left_encoder')
    enc_r = load_csv(d, 'right_encoder')
    steer = load_csv(d, 'steering')

    t = ips['t'].astype(float)
    t0 = t[0]
    t = t - t0
    x, y = ips['x'].astype(float), ips['y'].astype(float)

    ti = imu['t'].astype(float) - t0
    yaw_imu = yaw_from_quat(imu['qx'], imu['qy'], imu['qz'], imu['qw'])
    yaw = interp_angle(ti, yaw_imu, t)
    gyro = interp_at(ti, imu['wz'].astype(float), t)

    ts = steer['t'].astype(float) - t0
    delta = interp_at(ts, steer['v'].astype(float), t)

    vl = encoder_speed(enc_l['t'].astype(float) - t0, enc_l['pos'].astype(float))
    vr = encoder_speed(enc_r['t'].astype(float) - t0, enc_r['pos'].astype(float))
    v_enc = 0.5 * (interp_at(enc_l['t'].astype(float) - t0, vl, t)
                   + interp_at(enc_r['t'].astype(float) - t0, vr, t))

    # IPS speed: central difference of ground-truth position.
    dx = np.gradient(x, t)
    dy = np.gradient(y, t)
    v_ips = np.hypot(dx, dy)
    course = np.arctan2(dy, dx)       # direction of travel from ground truth

    return dict(t=t, x=x, y=y, yaw=yaw, gyro=gyro, delta=delta,
                v_enc=v_enc, v_ips=v_ips, course=course)


def heading_offset(run):
    """IMU yaw minus the IPS velocity direction, while genuinely driving."""
    m = (run['v_ips'] > 2.0) & (np.abs(run['gyro']) < 0.3)
    if m.sum() < 20:
        m = run['v_ips'] > 1.0
    d = wrap(run['yaw'][m] - run['course'][m])
    return float(np.arctan2(np.sin(d).mean(), np.cos(d).mean())), float(
        np.degrees(np.std(wrap(d - np.arctan2(np.sin(d).mean(),
                                              np.cos(d).mean())))))


def load_ref(ref_csv):
    ref = np.genfromtxt(ref_csv, delimiter=',', comments='#')
    x, y = ref[:, 1], ref[:, 2]
    # Tangent from the geometry itself: the file's psi_rad is in the TUM
    # convention (zero along +y, heading = psi + pi/2), and guessing wrong
    # silently turns cross-track into along-track.
    th = np.arctan2(np.gradient(np.r_[y, y[:1]])[:-1],
                    np.gradient(np.r_[x, x[:1]])[:-1])
    return dict(s=ref[:, 0], x=x, y=y, psi=ref[:, 3], theta=th,
                kappa=ref[:, 4], v=ref[:, 5])


def ref_kappa(run, ref):
    d = np.hypot(run['x'][:, None] - ref['x'][None, :],
                 run['y'][:, None] - ref['y'][None, :])
    j = np.argmin(d, axis=1)
    return ref['kappa'][j], ref['s'][j], d[np.arange(len(j)), j]


def cross_track(ref, x, y):
    """Signed distance from the reference line, + = LEFT of the reference."""
    x = np.atleast_1d(x)
    y = np.atleast_1d(y)
    d = np.hypot(x[:, None] - ref['x'][None, :], y[:, None] - ref['y'][None, :])
    j = np.argmin(d, axis=1)
    dx = x - ref['x'][j]
    dy = y - ref['y'][j]
    th = ref['theta'][j]
    return -np.sin(th) * dx + np.cos(th) * dy, j


# --------------------------------------------------------------------------
# Prediction models.  Each returns (x, y, yaw) at t+H starting from the true
# pose at t, integrating with sub-steps of ~0.01 s.
# --------------------------------------------------------------------------

def integrate(run, i, H, yaw_rate_fn, v, sub=0.01):
    n = max(1, int(round(H / sub)))
    dt = H / n
    x, y, yaw = run['x'][i], run['y'][i], run['yaw_c'][i]
    for k in range(n):
        r = yaw_rate_fn(k * dt)
        yaw_mid = yaw + 0.5 * r * dt
        x += v * math.cos(yaw_mid) * dt
        y += v * math.sin(yaw_mid) * dt
        yaw = yaw + r * dt
    return x, y, yaw


MPC_CMD_DELAY = 0.18      # mpc.py cmd_delay_s
SCAN_LATENCY = 0.09       # s, one sensor period: pose stamp -> control step


def t_of(run, i):
    return run['t'][i]


def delta_of(run, ti):
    return float(np.interp(ti, run['t'], run['delta']))


def make_models(run, wheelbase_alt=0.3312, gain=1.0):
    t = run['t']
    delta = run['delta']

    def delta_at(ti):
        return float(np.interp(ti, t, delta))

    models = {}

    models['M0 const heading'] = lambda i, H, v: (lambda tau: 0.0)
    models['M1 gyro held'] = lambda i, H, v: (lambda tau: run['gyro'][i])
    models['M2 bicycle delta held'] = lambda i, H, v: (
        lambda tau, d=delta[i]: v * math.tan(d) / vehicle.WHEELBASE)

    def m3(i, H, v):
        ti = t[i]
        return lambda tau: v * math.tan(
            delta_at(ti + tau + DEAD_TIME)) / vehicle.WHEELBASE

    def m4(i, H, v):
        ti = t[i]
        return lambda tau: v * math.tan(
            delta_at(ti + tau + DEAD_TIME)) / wheelbase_alt

    def m5(i, H, v):
        ti = t[i]
        return lambda tau: gain * v * math.tan(
            delta_at(ti + tau + DEAD_TIME)) / vehicle.WHEELBASE

    def m6(i, H, v):
        ti = t[i]
        return lambda tau: v * vehicle.achieved_curvature(
            delta_at(ti + tau + DEAD_TIME))

    def m7(i, H, v):
        ti = t[i]
        k0 = math.tan(delta_at(ti + DEAD_TIME)) / vehicle.WHEELBASE
        return lambda tau: run['gyro'][i] + v * (
            math.tan(delta_at(ti + tau + DEAD_TIME)) / vehicle.WHEELBASE - k0)

    models['M3 bicycle delta shifted'] = m3
    models['M4 M3 + L=0.3312'] = m4
    models['M5 M3 + fitted gain'] = m5
    models['M6 M3 + understeer'] = m6
    def m8(i, H, v):
        """What mpc.py actually integrates.

        `_cmd_hist` holds the rate-limited COMMAND in radians and is indexed at
        t - cmd_delay. A command reaches the actuator DEAD_TIME later, so the
        angle mpc believes is acting at time tau is the FEEDBACK trace at
        tau - (MPC_CMD_DELAY - DEAD_TIME) = tau - 0.045 s. The first
        SCAN_LATENCY of the window is predicted_state()'s gyro-held leg.
        """
        ti = t[i]
        lag = MPC_CMD_DELAY - DEAD_TIME
        g = run['gyro'][i]
        return lambda tau: (g if tau < SCAN_LATENCY else
                            v * math.tan(delta_at(ti + tau - lag))
                            / vehicle.WHEELBASE)

    models['M7 gyro + dcurv'] = m7
    models['M8 mpc composite'] = m8
    return models


def evaluate(run, models, H, speed_key, idx):
    """Return dict model -> (e_lat, e_lon) arrays over the samples in `idx`."""
    t = run['t']
    xt = np.interp(t[idx] + H, t, run['x'])
    yt = np.interp(t[idx] + H, t, run['y'])
    yawt = interp_angle(t, run['yaw_c'], t[idx] + H)
    out = {}
    for name, mk in models.items():
        ex = np.empty(len(idx))
        ey = np.empty(len(idx))
        for n, i in enumerate(idx):
            v = run[speed_key][i]
            px, py, _ = integrate(run, i, H, mk(i, H, v), v)
            ex[n], ey[n] = px - xt[n], py - yt[n]
        c, s = np.cos(yawt), np.sin(yawt)
        lon = ex * c + ey * s
        lat = -ex * s + ey * c
        out[name] = (lat, lon)
    return out


def fit_gain(run, H, speed_key, idx, lo=0.8, hi=2.0):
    """Yaw gain on M3 that minimises lateral RMS over `idx`."""
    best, bg = None, 1.0
    for g in np.arange(lo, hi + 1e-9, 0.01):
        models = {'m': make_models(run, gain=g)['M5 M3 + fitted gain']}
        lat, _ = evaluate(run, models, H, speed_key, idx)['m']
        r = float(np.sqrt(np.mean(lat ** 2)))
        if best is None or r < best:
            best, bg = r, float(g)
    return bg, best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv_dir', help='directory of CSVs exported from the bag')
    ap.add_argument('--ref', default='data/racelines/track_v2_safe.csv')
    ap.add_argument('--stride', type=int, default=1,
                    help='sample stride (speed vs coverage)')
    ap.add_argument('--fit-stride', type=int, default=2,
                    help='stride used for the gain fit')
    args = ap.parse_args()

    run = load_run(args.csv_dir)
    off, spread = heading_offset(run)
    print(f'samples {len(run["t"])}, duration {run["t"][-1]:.1f} s, '
          f'mean dt {np.mean(np.diff(run["t"]))*1000:.1f} ms')
    print(f'IMU yaw - IPS course offset: {off:+.4f} rad '
          f'({math.degrees(off):+.2f} deg), scatter {spread:.2f} deg')
    # Use the IMU yaw corrected onto the ground-truth travel direction.
    run['yaw_c'] = wrap(run['yaw'] - off)

    ref = load_ref(args.ref)
    kap, s_ref, d_ref = ref_kappa(run, ref)
    run['kappa'] = kap

    ok = ((run['v_ips'] > MIN_SPEED) & (d_ref < 1.0)
          & np.isfinite(run['v_enc']))
    # Drop samples whose t+H window runs off the end of the bag, and teleports.
    step = np.hypot(np.diff(run['x']), np.diff(run['y']))
    dt = np.diff(run['t'])
    # A jump is an implied speed no car can do -- NOT a fixed distance, which
    # would flag every fast straight (0.28 m/sample at 3 m/s, 0.55 m at 6).
    bad = (step / np.maximum(dt, 1e-3) > vehicle.SPEED_MAX) | (dt > 0.15)
    tele = np.zeros(len(run['t']), bool)
    tele[1:] = bad
    tele[:-1] |= bad
    # A prediction window that spans a teleport/gap is meaningless: mask the
    # whole window, not just its endpoints.
    span = int(math.ceil((max(HORIZONS) + DEAD_TIME)
                         / max(np.median(dt), 1e-3))) + 1
    blocked = tele.copy()
    for k in range(1, span + 1):
        blocked[:-k] |= tele[k:]
    ok &= ~blocked
    print(f'masked {blocked.sum()} samples around teleports / sensor gaps '
          f'({int(np.sum(step / np.maximum(dt, 1e-3) > vehicle.SPEED_MAX))} '
          f'position jumps, {int(np.sum(dt > 0.15))} gaps)')
    horizon_room = run['t'] < run['t'][-1] - max(HORIZONS) - DEAD_TIME - 0.05
    ok &= horizon_room

    hair = ok & (np.abs(kap) > KAPPA_HAIRPIN)
    straight = ok & (np.abs(kap) < 0.15)
    print(f'usable {ok.sum()}  hairpin {hair.sum()}  straight {straight.sum()}')

    # Speed sanity + the core diagnostic: measured yaw rate vs the bicycle model.
    m = ok & (np.abs(kap) > KAPPA_HAIRPIN)
    for label, key in (('encoder', 'v_enc'), ('IPS', 'v_ips')):
        print(f'speed[{label}] mean {run[key][ok].mean():.3f} m/s, '
              f'hairpin mean {run[key][m].mean():.3f} m/s, '
              f'bias vs IPS {np.mean(run[key][ok] - run["v_ips"][ok]):+.3f}')

    print('\n--- measured yaw rate vs kinematic bicycle ---')
    for label, mask in (('hairpin', hair), ('straight', straight), ('all', ok)):
        for shift, tag in ((0.0, 'delta held'), (DEAD_TIME, 'delta shifted')):
            d = np.interp(run['t'] + shift, run['t'], run['delta'])
            pred = run['v_ips'] * np.tan(d) / vehicle.WHEELBASE
            g = run['gyro']
            sel = mask & (np.abs(pred) > 0.2)
            if sel.sum() < 10:
                continue
            ratio = np.median(g[sel] / pred[sel])
            print(f'{label:9s} {tag:14s} n={sel.sum():4d} '
                  f'median gyro/model {ratio:.3f}  '
                  f'(gyro {np.abs(g[sel]).mean():.3f}, '
                  f'model {np.abs(pred[sel]).mean():.3f} rad/s)')
    # Is the gyro the same as the ground-truth yaw rate?
    dyaw = np.gradient(np.unwrap(run['yaw_c']), run['t'])
    sel = ok & (np.abs(dyaw) > 0.2)
    print(f'gyro vs d(yaw)/dt: median ratio '
          f'{np.median(run["gyro"][sel] / dyaw[sel]):.3f}, '
          f'lag-corr peak at '
          f'{best_lag(run["gyro"][ok], dyaw[ok], np.mean(np.diff(run["t"]))):+.3f} s')
    # Does the IMU heading lag the true course?
    print(f'heading lag (IMU yaw vs IPS course, after offset): '
          f'{best_lag(wrap(run["yaw_c"][ok]), wrap(run["course"][ok]), np.mean(np.diff(run["t"]))):+.3f} s')

    print('\n--- steering staleness (hairpins) ---')
    lag = MPC_CMD_DELAY - DEAD_TIME
    d_now = run['delta']
    d_mpc = np.interp(run['t'] - lag, run['t'], run['delta'])
    d_shift = np.interp(run['t'] + DEAD_TIME, run['t'], run['delta'])
    for label, mask in (('hairpin', hair), ('straight', straight)):
        a = np.abs(d_now[mask]).mean()
        b = np.abs(d_mpc[mask]).mean()
        c = np.abs(d_shift[mask]).mean()
        print(f'{label:9s} |delta| now {a:.4f}  mpc-believed(t-0.045) {b:.4f} '
              f'({100*(b-a)/max(a,1e-6):+.1f}%)  actuator-in-flight(t+0.135) '
              f'{c:.4f} ({100*(c-a)/max(a,1e-6):+.1f}%)')
        k_now = np.tan(np.abs(d_now[mask])) / vehicle.WHEELBASE
        k_mpc = np.tan(np.abs(d_mpc[mask])) / vehicle.WHEELBASE
        print(f'{"":9s} mean |kappa| model now {k_now.mean():.3f} vs '
              f'mpc-believed {k_mpc.mean():.3f} 1/m; gyro-implied '
              f'{np.mean(np.abs(run["gyro"][mask]) / np.maximum(run["v_ips"][mask], 0.5)):.3f}')

    print('\n--- where the live 0.11 m goes (M8, encoder speed) ---')
    print('cross-track sign: + = LEFT of the reference line')
    models8 = {'M8': make_models(run)['M8 mpc composite']}
    for H in HORIZONS:
        ih = np.flatnonzero(hair)[::args.fit_stride]
        e_now, _ = cross_track(ref, run['x'][ih], run['y'][ih])
        xf = np.interp(run['t'][ih] + H, run['t'], run['x'])
        yf = np.interp(run['t'][ih] + H, run['t'], run['y'])
        e_fut, _ = cross_track(ref, xf, yf)
        px = np.empty(len(ih)); py = np.empty(len(ih))
        for n, i in enumerate(ih):
            v = run['v_enc'][i]
            px[n], py[n], _ = integrate(run, i, H, models8['M8'](i, H, v), v)
        e_pred, _ = cross_track(ref, px, py)
        sg = -np.sign(run['kappa'][ih])        # + = outside of the corner
        print(f'H={H:.2f}  true e(t) {np.mean(e_now*sg):+.3f}  '
              f'true e(t+H) {np.mean(e_fut*sg):+.3f}  '
              f'predicted e(t+H) {np.mean(e_pred*sg):+.3f}  '
              f'(outside-positive, hairpins, n={len(ih)})   '
              f'pred(t+H)-true(t) = {np.mean((e_pred-e_now)*sg):+.3f} m, '
              f'pred(t+H)-true(t+H) = {np.mean((e_pred-e_fut)*sg):+.3f} m')

    print('\n--- effective steering-source lag sweep (M8 form) ---')
    print('lag = how far BACK in the feedback trace the model reads the angle')
    print('acting at time tau.  mpc.py implies cmd_delay - dead_time = '
          f'{MPC_CMD_DELAY - DEAD_TIME:.3f} s.')
    ih = np.flatnonzero(hair)[::args.fit_stride]
    ia = np.flatnonzero(ok)[::args.stride]
    for H in (0.18, 0.27):
        line = []
        for lag in (-0.09, -0.045, 0.0, 0.045, 0.09, 0.135, 0.18):
            def mk(i, Hh, v, lag=lag):
                ti, g = t_of(run, i), run['gyro'][i]
                return lambda tau: (g if tau < SCAN_LATENCY else
                                    v * math.tan(delta_of(run, ti + tau - lag))
                                    / vehicle.WHEELBASE)
            r = evaluate(run, {'m': mk}, H, 'v_enc', ih)['m'][0]
            ra = evaluate(run, {'m': mk}, H, 'v_enc', ia)['m'][0]
            line.append(f'  lag {lag:+.3f}: hairpin RMS {np.sqrt(np.mean(r**2)):.3f} '
                        f'bias {r.mean():+.3f} | all RMS {np.sqrt(np.mean(ra**2)):.3f}')
        print(f'H={H:.2f}')
        print('\n'.join(line))

    idx_all = np.flatnonzero(ok)[::args.stride]
    idx_h = np.flatnonzero(hair)[::args.stride]
    idx_s = np.flatnonzero(straight)[::args.stride]

    for speed_key, sname in (('v_enc', 'encoder speed'), ('v_ips', 'IPS speed')):
        print(f'\n=== {sname} ===')
        for H in HORIZONS:
            g, gr = fit_gain(run, H, speed_key,
                             np.flatnonzero(hair)[::args.fit_stride])
            models = make_models(run, gain=g)
            res = {b: evaluate(run, models, H, speed_key, ix)
                   for b, ix in (('all', idx_all), ('hairpin', idx_h),
                                 ('straight', idx_s))}
            print(f'\nH = {H:.2f} s   (fitted M5 yaw gain = {g:.2f}, '
                  f'hairpin lat RMS {gr:.3f} m)')
            hdr = (f'{"model":24s}' + ''.join(
                f'{b+" RMS":>10s}{b+" mean":>11s}'
                for b in ('all', 'hairpin', 'straight'))
                + f'{"hp outside":>11s}{"hp lon":>8s}{"hp p50":>9s}{"hp p95":>9s}')
            print(hdr)
            out_sign = -np.sign(run['kappa'][idx_h])
            for name in models:
                cells = []
                for b in ('all', 'hairpin', 'straight'):
                    lat, _ = res[b][name]
                    cells.append(f'{np.sqrt(np.mean(lat**2)):10.3f}'
                                 f'{lat.mean():11.3f}')
                    if b == 'hairpin':
                        p95 = np.percentile(np.abs(lat), 95)
                        p50 = np.median(np.abs(lat))
                lat_h, lon_h = res['hairpin'][name]
                print(f'{name:24s}' + ''.join(cells)
                      + f'{np.mean(lat_h * out_sign):11.3f}'
                      + f'{lon_h.mean():8.3f}' + f'{p50:9.3f}{p95:9.3f}')


def best_lag(a, b, dt, max_lag=8):
    """Lag (s) at which `a` best matches `b`; positive = a lags b."""
    a = a - a.mean()
    b = b - b.mean()
    best, bl = -1e9, 0
    for k in range(-max_lag, max_lag + 1):
        if k >= 0:
            x, y = a[k:], b[:len(b) - k] if k else b
        else:
            x, y = a[:len(a) + k], b[-k:]
        n = min(len(x), len(y))
        if n < 20:
            continue
        c = float(np.dot(x[:n], y[:n]) / (np.linalg.norm(x[:n])
                                          * np.linalg.norm(y[:n]) + 1e-12))
        if c > best:
            best, bl = c, k
    return bl * dt


if __name__ == '__main__':
    main()
