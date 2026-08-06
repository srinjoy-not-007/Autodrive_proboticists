#!/usr/bin/env python3
"""Validate laps from a recorded position trace, independent of the sim counter.

The simulator's lap counter can be fooled by a U-turn back across the line, so
a "lap" here means: the car passed through every one of N gates spaced evenly
around the centerline, in order, and returned to the start gate. Anything else
is reported as an incomplete or out-of-order traversal.

Usage: .venv/bin/python tools/validate_laps.py data/logs/race_X.csv \
           data/racelines/track_v2_centerline.csv
"""

import csv
import sys

import numpy as np

N_GATES = 12
GATE_RADIUS = 0.55      # m; must pass within this of the gate centre


def main(log_csv, centerline_csv):
    xs, ys, ts, cols = [], [], [], []
    with open(log_csv) as f:
        for r in csv.DictReader(f):
            xs.append(float(r['x']))
            ys.append(float(r['y']))
            ts.append(float(r['t']))
            cols.append(int(r['collision_count']))
    xy = np.column_stack([xs, ys])
    t = np.array(ts)
    cols = np.array(cols)

    cl = np.loadtxt(centerline_csv, delimiter=',')[:, :2]
    gates = cl[np.linspace(0, len(cl), N_GATES, endpoint=False).astype(int)]

    # Which gate (if any) is the car inside at each sample
    d = np.linalg.norm(xy[:, None, :] - gates[None, :, :], axis=2)
    nearest = np.argmin(d, axis=1)
    inside = d[np.arange(len(xy)), nearest] < GATE_RADIUS
    seq = [(i, nearest[i]) for i in range(len(xy)) if inside[i]]

    # Compress consecutive samples at the same gate
    visits = []
    for i, g in seq:
        if not visits or visits[-1][1] != g:
            visits.append((i, g))

    laps = []
    expected = 0
    start_i = None
    for i, g in visits:
        if g == 0 and expected == 0:
            start_i = i
            expected = 1
            continue
        if start_i is None:
            continue
        if g == expected:
            expected = (expected + 1) % N_GATES
            if expected == 0:      # completed the loop back to gate 0
                laps.append((start_i, i))
                start_i = i
                expected = 1
        elif g != (expected - 1) % N_GATES:
            # Out-of-order gate: abandon this attempt and wait for gate 0
            start_i = None
            expected = 0

    print(f'samples {len(xy)}, gate visits {len(visits)}')
    print(f'VALID LAPS (all {N_GATES} gates in order): {len(laps)}')
    for k, (a, b) in enumerate(laps, 1):
        lap_t = t[b] - t[a]
        ncol = int(cols[b] - cols[a])
        flag = '' if ncol == 0 else f'  <-- {ncol} collision(s)'
        print(f'  lap {k}: {lap_t:6.2f} s  collisions {ncol}{flag}')
    if laps:
        times = np.array([t[b] - t[a] for a, b in laps])
        clean = [k for k, (a, b) in enumerate(laps) if cols[b] == cols[a]]
        print(f'\nbest {times.min():.2f} s   mean {times.mean():.2f} s   '
              f'clean laps {len(clean)}/{len(laps)}')
        print(f'total collisions in log: {int(cols[-1] - cols[0])}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
