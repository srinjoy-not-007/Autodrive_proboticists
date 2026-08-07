#!/usr/bin/env python3
"""Quantify steering oscillation from a race log.

Swinging left-right is visible to the eye but needs a number to tune against.
Two measures: the rate of sign changes in the steering signal (how often the
car reverses lock) and the mean absolute steering rate (how violently).

Usage: .venv/bin/python tools/steering_smoothness.py data/logs/race_X.csv
"""
import csv
import sys

import numpy as np


def main(path):
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r['t']) for r in rows])
    st = np.array([float(r['steering']) for r in rows])
    ok = np.diff(t) > 1e-3
    rate = np.diff(st)[ok] / np.diff(t)[ok]
    # a "reversal" is a sign change of meaningful size, not sensor dither
    sig = st[np.abs(st) > 0.02]
    reversals = int(np.sum(np.diff(np.sign(sig)) != 0)) if len(sig) > 2 else 0
    dur = t[-1] - t[0]
    print(f'{path.split("/")[-1]}  {dur:.0f}s')
    print(f'  steering reversals : {reversals}  ({reversals/max(dur,1):.2f} per second)')
    print(f'  |steer rate|       : mean {np.abs(rate).mean():.2f}  '
          f'p95 {np.percentile(np.abs(rate),95):.2f} rad/s  (actuator limit 3.2)')
    print(f'  |steer|            : mean {np.abs(st).mean():.3f}  max {np.abs(st).max():.3f} rad')


if __name__ == '__main__':
    main(sys.argv[1])
