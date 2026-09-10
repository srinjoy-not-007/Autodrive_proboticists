#!/usr/bin/env python3
"""Score actual simulator lap transitions; never infer race success from geometry.

One warm-up, ten consecutive scored laps, +10 seconds per scored collision.
CSV sampling timestamps approximate crossing times to one sensor update; official
simulator timing remains authoritative. Logs must begin before warm-up ends.
The optional legacy centerline argument is accepted but never used for scoring.
"""

import argparse
import csv
import math


def score_rows(rows, n_scored=10):
    """Return the scored interval or reject incomplete/ambiguous telemetry."""
    if not rows or n_scored < 1:
        raise ValueError('empty log or invalid lap count')
    samples = [(float(r['t']), int(r['lap_count']), int(r['collision_count']))
               for r in rows]
    if any(not math.isfinite(t) for t, _, _ in samples):
        raise ValueError('nonfinite timestamps')
    if samples[0][1] != 0:
        raise ValueError('log starts after warm-up; cannot certify the first race')
    if any(b[0] < a[0] or b[1] < a[1] or b[2] < a[2]
           for a, b in zip(samples, samples[1:])):
        raise ValueError('time/counter reset within log; score separate runs')
    crossings = []
    prev = samples[0]
    for sample in samples[1:]:
        if sample[1] != prev[1]:
            if sample[1] != prev[1] + 1:
                raise ValueError('missing lap transition; cannot certify timing')
            crossings.append(sample)
        prev = sample
    if len(crossings) < n_scored + 1:
        raise ValueError(f'need warm-up plus {n_scored} counted laps; '
                         f'only {len(crossings)} transitions')
    selected = crossings[:n_scored + 1]
    laps = [b[0] - a[0] for a, b in zip(selected, selected[1:])]
    collisions = selected[-1][2] - selected[0][2]
    elapsed = selected[-1][0] - selected[0][0]
    return dict(laps=laps, collisions=collisions, elapsed=elapsed,
                race_time=elapsed + 10.0 * collisions,
                passed=collisions <= 10, start=selected[0][0], end=selected[-1][0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log')
    parser.add_argument('centerline', nargs='?', help='legacy argument, unused')
    parser.add_argument('--laps', type=int, default=10)
    args = parser.parse_args()
    try:
        with open(args.log) as stream:
            result = score_rows(list(csv.DictReader(stream)), args.laps)
    except (ValueError, KeyError, OSError) as exc:
        print(f'UNVERIFIED: {exc}')
        return 1
    print('Scored laps: ' + ', '.join(f'{t:.3f}' for t in result['laps']))
    print(f"Elapsed {result['elapsed']:.3f} s; scored collisions "
          f"{result['collisions']}; race {result['race_time']:.3f} s; "
          f"best {min(result['laps']):.3f} s")
    print('PASS' if result['passed'] else 'DISQUALIFIED')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
