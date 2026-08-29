#!/usr/bin/env python3
"""Score a run the way the competition scores it, from a race log alone.

Format (6th RoboRacer Sim Racing League): one warm-up lap, then ten scored
laps, optional cool-down. Race time is 10 x median scored lap plus a ten second
penalty per collision. More than ten collisions is a disqualification.

Deliberately knows nothing about which controller produced the log -- it reads
positions, the simulator's own lap counter and its collision counter, and says
pass or fail. Laps are counted by gate traversal rather than by the simulator's
counter, because that counter only increments in the racing direction and we
have a reactive fallback that will happily drive a circuit backwards.

Usage: score_race.py <race_log.csv> <centerline.csv> [--laps 10]
"""

import csv
import sys

import numpy as np

N_GATES = 12
GATE_RADIUS = 0.55
PENALTY_S = 10.0
MAX_COLLISIONS = 10


def laps_from_gates(xy, t, centre):
    idx = np.linspace(0, len(centre), N_GATES, endpoint=False).astype(int)
    gates = centre[idx]
    laps, want, t0, seen = [], 0, None, 0
    for p, ti in zip(xy, t):
        if np.hypot(*(p - gates[want])) < GATE_RADIUS:
            if want == 0 and t0 is not None and seen == N_GATES:
                laps.append(ti - t0)
                t0, seen = ti, 0
            elif want == 0 and t0 is None:
                t0 = ti
            want = (want + 1) % N_GATES
            seen += 1
    return laps


def main():
    log, cl = sys.argv[1], sys.argv[2]
    n_scored = int(sys.argv[sys.argv.index('--laps') + 1]) \
        if '--laps' in sys.argv else 10

    rows = list(csv.DictReader(open(log)))
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    t = np.array([float(r['t']) for r in rows])
    coll = np.array([int(r['collision_count']) for r in rows])
    centre = np.loadtxt(cl, delimiter=',', skiprows=1)[:, :2]

    laps = laps_from_gates(xy, t, centre)
    total_coll = int(coll[-1] - coll[0])
    print(f'log {log.split("/")[-1]}: {t[-1]-t[0]:.0f} s, '
          f'{len(laps)} complete laps, {total_coll} collisions')

    if len(laps) < n_scored + 1:
        print(f'  RESULT: FAIL — needed {n_scored + 1} laps '
              f'(1 warm-up + {n_scored} scored), completed {len(laps)}')
        return 1

    scored = np.array(laps[1:1 + n_scored])          # drop the warm-up
    med = float(np.median(scored))
    # Collisions are cumulative over the whole log; scale to the scored window.
    share = total_coll * n_scored / max(len(laps), 1)
    race = 10.0 * med + PENALTY_S * share
    print(f'  warm-up {laps[0]:.2f} s; scored laps '
          f'{", ".join(f"{x:.2f}" for x in scored)}')
    print(f'  median {med:.3f} s  best {scored.min():.2f} s  '
          f'collisions in window ~{share:.1f}')
    if total_coll > MAX_COLLISIONS:
        print(f'  RESULT: DISQUALIFIED — {total_coll} collisions > '
              f'{MAX_COLLISIONS}')
        return 1
    print(f'  RESULT: PASS — race time {race:.1f} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
