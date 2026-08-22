"""Measure the car's real lateral grip from an ordinary race log.

`skidpad.py` tries to measure grip by holding a steering angle and ramping
speed, but it needs open space neither competition track has — on Porto the
slide widens the circle until the car grinds the walls and corrupts the result.

This gets the same curve out of a normal racing lap instead. Yaw rate comes
from differentiating the logged IPS trace, so for every sample we can compare
the curvature the car *achieved* against the curvature the steering angle
*asked for* under the kinematic bicycle model, kappa = tan(delta)/L. While the
front tyres are inside their grip the ratio is 1.0; as they saturate it falls
away, and the lateral acceleration at which it stops rising is the real limit.

Samples near a collision are dropped: the simulator teleports the car back to
its last checkpoint, which puts a metres-per-sample step into the derivative.

Usage:
  .venv/bin/python tools/grip_curve.py data/logs/race_<ts>.csv
"""

import csv
import sys

import numpy as np

sys.path.insert(0, 'src')
WHEELBASE = 0.324          # kept literal so this runs on the host, outside ROS
MAX_STEER = 0.5236
TELEPORT_MPS = 8.0         # faster than the car can go: implies a reset
STEER_BINS = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, MAX_STEER]


def load(path):
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r['t']) for r in rows])
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    steer = np.array([float(r['steering']) for r in rows])
    col = np.array([int(r['collision_count']) for r in rows])
    return t, xy, steer, col


def kinematics(t, xy, steer, col):
    """Per-sample achieved curvature, kinematic curvature and lateral accel."""
    dt = np.maximum(np.diff(t), 1e-3)
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    v = step / dt

    drop = np.zeros(len(t), bool)
    for i in np.flatnonzero(np.diff(col) > 0) + 1:
        drop[max(0, i - 8):i + 8] = True          # collision reset teleports
    jump = step > TELEPORT_MPS * dt
    drop[:-1] |= jump
    drop[1:] |= jump

    head = np.arctan2(np.diff(xy[:, 1]), np.diff(xy[:, 0]))
    dhead = (np.diff(head) + np.pi) % (2 * np.pi) - np.pi
    yaw = dhead / np.maximum(np.diff(0.5 * (t[:-1] + t[1:])), 1e-3)

    v_mid = 0.5 * (v[:-1] + v[1:])
    steer_mid = steer[1:-1]
    usable = (~drop[1:-1]) & (v_mid > 1.0)

    kappa_act = np.abs(yaw) / np.maximum(v_mid, 0.1)
    kappa_kin = np.abs(np.tan(steer_mid)) / WHEELBASE
    return {
        'v': v_mid, 'steer': np.abs(steer_mid), 'ok': usable,
        'kappa_act': kappa_act, 'kappa_kin': kappa_kin,
        'ay': v_mid ** 2 * kappa_act,
        'ratio': kappa_act / np.maximum(kappa_kin, 1e-3),
    }


def main(path):
    d = kinematics(*load(path))
    ok = d['ok']
    print(f'{ok.sum()} usable samples, {d["v"][ok].min():.1f}-{d["v"][ok].max():.1f} m/s\n')

    print(f'{"|steer| rad":>13} {"n":>5} {"kappa asked":>12} {"achieved":>9} '
          f'{"ratio":>6} {"ay m/s2":>8}')
    limit = None
    for lo, hi in zip(STEER_BINS[:-1], STEER_BINS[1:]):
        m = ok & (d['steer'] >= lo) & (d['steer'] < hi)
        if m.sum() < 15:
            continue
        ratio = float(np.median(d['ratio'][m]))
        ay = float(np.median(d['ay'][m]))
        print(f'{lo:5.2f}-{hi:5.2f} {m.sum():7d} {np.median(d["kappa_kin"][m]):12.2f} '
              f'{np.median(d["kappa_act"][m]):9.2f} {ratio:6.2f} {ay:8.2f}')
        if limit is None and ratio < 0.85:
            limit = (lo, ay)

    print(f'\nachieved ay: p50 {np.percentile(d["ay"][ok], 50):.2f}  '
          f'p90 {np.percentile(d["ay"][ok], 90):.2f}  '
          f'p99 {np.percentile(d["ay"][ok], 99):.2f} m/s2')
    if limit:
        print(f'front tyres leave their grip at about {limit[0]:.2f} rad of steering, '
              f'where the car is pulling {limit[1]:.2f} m/s2')
    cornering = ok & (d['steer'] > 0.15)
    if cornering.sum() > 30:
        print(f'sustained cornering limit (median ay above 0.15 rad steer): '
              f'{np.median(d["ay"][cornering]):.2f} m/s2')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else sys.exit(__doc__))
