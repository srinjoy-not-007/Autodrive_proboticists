#!/usr/bin/env python3
"""Score actual simulator lap transitions; never infer race success from geometry.

One warm-up, ten consecutive scored laps, +10 seconds per scored collision.
Three lap-time sources are reported, most authoritative first:

* **official** -- the sequence of `last_lap_time` values the simulator latches at
  each line crossing. This is exactly what the organisers' Foxglove
  `race_time` / `adjusted_race_time` scripts accumulate, so it carries no
  sampling error at all. Needs a log written by a `race_logger` new enough to
  record the column.
* **sim-timer estimate** -- the running `lap_time` on the last sample before a
  transition: the same clock, but read up to one sensor period early.
* **sample clock** -- differences of our own CSV timestamps, subject to logger
  jitter on both ends.

Logs must begin before warm-up ends. The optional legacy centerline argument is
accepted but never used for scoring.
"""

import argparse
import csv
import math


def _official_lap_times(rows, laps_wanted):
    """Per-lap times as the simulator itself latched them, or None.

    `laps_wanted` is the `lap_count` value reached at each crossing of interest,
    in order; the value latched into `last_lap_time` just after the counter
    reaches L is lap L's official duration -- the same convention the running
    `lap_time` estimate below uses.

    `lap_count` and `last_lap_time` ride the same ~10 Hz bridge cycle but land
    in separate callbacks, so the latch may be recorded one row either side of
    the counter step; each lap's search window therefore starts one row early
    and a value is accepted only once it differs from the previous lap's. A
    dropped or never-published latch yields None so the caller falls back
    rather than mis-attributing a lap.
    """
    if not rows or 'last_lap_time' not in rows[0]:
        return None
    try:
        values = [float(r['last_lap_time']) for r in rows]
        counts = [int(r['lap_count']) for r in rows]
    except (TypeError, ValueError):
        return None
    if any(not math.isfinite(v) for v in values):
        return None
    first_row_of = {}
    for i in range(1, len(rows)):
        if counts[i] != counts[i - 1]:
            first_row_of.setdefault(counts[i], i)
    out = []
    previous = None
    for lap in laps_wanted:
        if lap not in first_row_of:
            return None
        start = max(0, first_row_of[lap] - 1)
        stop = first_row_of.get(lap + 1, len(rows))
        found = None
        for k in range(start, max(stop, start + 1)):
            if values[k] > 0.0 and values[k] != previous:
                found = values[k]
                break
        if found is None:
            return None
        out.append(found)
        previous = found
    return out


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
    # The stewards sum the simulator's own lap timer, not our sample clock.
    # The logger records `lap_time` (the running timer, reset at each
    # crossing), so the value on the last sample before a transition is that
    # lap's official time to within one sensor period of *its* end -- but,
    # unlike sample-timestamp differences, unaffected by logger jitter.
    sim_laps = None
    if rows and 'lap_time' in rows[0]:
        try:
            before = {}
            prev_row = rows[0]
            for row in rows[1:]:
                if int(row['lap_count']) != int(prev_row['lap_count']):
                    before[int(row['lap_count'])] = float(prev_row['lap_time'])
                prev_row = row
            sim_laps = [before[c[1]] for c in selected[1:]]
            if any(not math.isfinite(v) or v <= 0 for v in sim_laps):
                sim_laps = None
        except (KeyError, ValueError):
            sim_laps = None
    official_laps = _official_lap_times(rows, [c[1] for c in selected])
    if official_laps is not None:
        # The first entry belongs to the warm-up lap; it only seeds the
        # "differs from the previous lap" test.
        official_laps = official_laps[1:]
    collisions = selected[-1][2] - selected[0][2]
    elapsed = selected[-1][0] - selected[0][0]
    return dict(laps=laps, collisions=collisions, elapsed=elapsed,
                race_time=elapsed + 10.0 * collisions, sim_laps=sim_laps,
                official_laps=official_laps,
                official_race_time=(None if official_laps is None
                                    else sum(official_laps) + 10.0 * collisions),
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
    penalty = 10.0 * result['collisions']
    if result['official_laps']:
        ol = result['official_laps']
        print('OFFICIAL (simulator last_lap_time) laps: '
              + ', '.join(f'{t:.3f}' for t in ol))
        print(f"OFFICIAL race {sum(ol):.3f} s + {penalty:.0f} s penalties = "
              f"{result['official_race_time']:.3f} s; best {min(ol):.3f} s")
    else:
        print('OFFICIAL race time unavailable: log has no usable '
              'last_lap_time column (pre-2026-09-12 logger)')
    if result['sim_laps']:
        sl = result['sim_laps']
        print(f"Sim-timer estimate: {sum(sl):.3f} s + penalties = "
              f"{sum(sl) + penalty:.3f} s; "
              f"best {min(sl):.3f} s; laps " + ', '.join(f'{t:.2f}' for t in sl))
    print('Sample-clock laps: ' + ', '.join(f'{t:.3f}' for t in result['laps']))
    print(f"Sample-clock elapsed {result['elapsed']:.3f} s; scored collisions "
          f"{result['collisions']}; race {result['race_time']:.3f} s; "
          f"best {min(result['laps']):.3f} s")
    print('PASS' if result['passed'] else 'DISQUALIFIED')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
