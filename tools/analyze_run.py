#!/usr/bin/env python3
"""Analyze a race_logger CSV: lap stats + ASCII track map with collision spots.

Usage: python3 tools/analyze_run.py data/logs/race_XXXX.csv
"""

import csv
import sys
from collections import defaultdict


def main(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        print('empty log')
        return

    xs = [float(r['x']) for r in rows]
    ys = [float(r['y']) for r in rows]
    cols = [(float(r['x']), float(r['y']), float(r['speed']), float(r['steering']))
            for r in rows if r['collision_event'] == '1']

    print(f'samples: {len(rows)}')
    print(f'x range: [{min(xs):.1f}, {max(xs):.1f}]  '
          f'y range: [{min(ys):.1f}, {max(ys):.1f}]')

    # Lap stats: group samples by lap_count, note lap_time progression
    laps = defaultdict(list)
    for r in rows:
        laps[int(r['lap_count'])].append(r)
    print('\nlaps seen:', sorted(laps))
    for lap in sorted(laps):
        seg = laps[lap]
        speeds = [float(r['speed']) for r in seg]
        ncol = sum(int(r['collision_event']) for r in seg)
        print(f'  lap {lap}: {len(seg)} samples, '
              f'v avg {sum(speeds)/len(speeds):.2f} max {max(speeds):.2f} m/s, '
              f'collisions {ncol}')

    print(f'\ncollisions: {len(cols)}')
    for x, y, v, s in cols:
        print(f'  at ({x:7.2f}, {y:7.2f})  v={v:.2f}  steer={s:+.2f}')

    # ASCII map: trajectory '.', collisions 'X', start 'S'
    W, H = 79, 40
    def to_cell(x, y):
        cx = int((x - min(xs)) / (max(xs) - min(xs) + 1e-9) * (W - 1))
        cy = int((y - min(ys)) / (max(ys) - min(ys) + 1e-9) * (H - 1))
        return (H - 1 - cy), cx
    grid = [[' '] * W for _ in range(H)]
    for x, y in zip(xs, ys):
        i, j = to_cell(x, y)
        grid[i][j] = '.'
    for x, y, _, _ in cols:
        i, j = to_cell(x, y)
        grid[i][j] = 'X'
    i, j = to_cell(xs[0], ys[0])
    grid[i][j] = 'S'
    print('\n' + '\n'.join(''.join(row) for row in grid))


if __name__ == '__main__':
    main(sys.argv[1])
