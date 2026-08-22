"""Sweep corridor margin x grip cap and rank the candidate racing lines.

Everything cheap on Porto has been swept already; what has never been explored
is the *middle* of the clearance frontier. 0.146 m of worst-corner clearance
collects ~57 collisions, 0.325 m runs 22 laps clean, and nothing in between has
been driven. That gap is where the remaining lap time is, because corner speed
goes as sqrt(ay/kappa) and the only way to lower kappa on this track is to spend
corridor width opening the corners.

The ranking is not by ideal lap time alone. From the measured grip curve
(tools/grip_curve.py) the front tyres track the bicycle model up to ~0.10 rad of
steering, are visibly saturating by 0.15, and at full lock deliver 37% of the
curvature asked of them. A line is therefore only as good as the fraction of it
the car can actually follow, so each candidate also reports how much of the lap
demands steering past that saturation point. That is what went wrong at ay 4.0:
the car ran wide at kappa 0.70 with the wheel pinned at full lock, and the
collisions came from the recovery, not the corner.

Usage:
  .venv/bin/python tools/sweep_lines.py <track> [--drive-top N]
"""

import argparse
import itertools
import subprocess
import sys

import numpy as np

sys.path.insert(0, 'tools')
from footprint_analysis import load_dist, clearance, CORNERS      # noqa: E402
from make_track_lines import MIN_CLEARANCE, drivability, worst_corner, ideal_lap  # noqa: E402

VENV = '.venv/bin/python'
WHEELBASE = 0.324
MAX_STEER = 0.5236

# Where the measured grip curve says the front axle stops responding linearly.
LINEAR_STEER = 0.10        # rad — ratio 1.00 below this
SATURATED_STEER = 0.15     # rad — ratio 0.80 and falling above this

# The corridor margin the optimizer keeps on top of the swept footprint. 0.16 is
# what the shipping pipeline uses and yields 0.146 m of measured clearance; the
# larger values buy clearance back at the cost of path length.
# None = the safe centerline geometry, profiled at the same caps. It is longer
# but its curvature peaks far lower, so it keeps steering authority in hand.
W_VEH = (None, 0.16, 0.20, 0.24, 0.28, 0.32)
# Capped at the measured 4.5 m/s2 plateau: above it the tyre gives nothing back.
AY = (3.0, 3.5, 4.0, 4.5)
JERK = 20.0                # m/s3 — the level the clean shipping profile survives


def steering_demand(line_csv):
    """How much of the lap asks for more steering than the tyres will give."""
    rl = np.loadtxt(line_csv, delimiter=',')
    kappa = np.abs(rl[:, 4])
    delta = np.arctan(WHEELBASE * kappa)
    ds = np.diff(np.append(rl[:, 0], rl[-1, 0] + (rl[1, 0] - rl[0, 0])))
    total = ds.sum()
    return {
        'kappa_max': float(kappa.max()),
        'steer_max': float(delta.max()),
        # What is left of the steering lock at the tightest point. This is the
        # authority the controller has to correct an error exactly where it can
        # least afford one, and min-curvature optimization spends it: it lowers
        # the *integral* of curvature and is free to spike locally.
        'steer_reserve': float(MAX_STEER - delta.max()),
        'frac_saturated': float(ds[delta > SATURATED_STEER].sum() / total),
        'frac_linear': float(ds[delta <= LINEAR_STEER].sum() / total),
    }


def generate(track, w_veh, ay, out):
    """w_veh None means the safe centerline geometry: no lateral optimization."""
    cmd = [VENV, 'tools/optimize_raceline.py',
           f'data/racelines/{track}_centerline.csv', '--out', out,
           '--ay', str(ay), '--ax', str(ay), '--jerk-lat', str(JERK)]
    cmd += ['--centerline-only'] if w_veh is None else ['--w-veh', str(w_veh)]
    return subprocess.run(cmd, capture_output=True, text=True).returncode == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track')
    ap.add_argument('--drive-top', type=int, default=0,
                    help='write the top N candidates as <track>_cand<i>.csv')
    args = ap.parse_args()
    track, mp = args.track, f'data/maps/{args.track}'

    # The shipping line, as the bar every candidate has to clear.
    base = f'data/racelines/{track}_safe.csv'
    base_lap, base_clear = ideal_lap(base), worst_corner(mp, base)
    base_dem = steering_demand(base)
    print(f'baseline (safe, ay 3.0): ideal {base_lap:5.2f} s  '
          f'clearance {base_clear:.3f} m  kappa_max {base_dem["kappa_max"]:.2f}  '
          f'saturated {base_dem["frac_saturated"]*100:.0f}% of the lap\n')

    rows = []
    for w_veh, ay in itertools.product(W_VEH, AY):
        tag = 'safe' if w_veh is None else f'w{w_veh}'
        out = f'/tmp/sweep_{track}_{tag}_ay{ay}'
        if not generate(track, w_veh, ay, out):
            continue
        csv = out + '.csv'
        bad = drivability(csv)
        wc = worst_corner(mp, csv)
        dem = steering_demand(csv)
        lap = ideal_lap(csv)
        ok = not bad and wc >= MIN_CLEARANCE
        rows.append(dict(w_veh=w_veh, ay=ay, lap=lap, clear=wc, ok=ok,
                         why='; '.join(bad) if bad else
                             ('' if ok else f'clearance < {MIN_CLEARANCE}'),
                         path=out, **dem))

    print(f'{"geom":>6} {"ay":>4} {"ideal":>7} {"clear":>7} {"k_max":>6} '
          f'{"steer":>6} {"resv":>6} {"sat%":>5}  verdict')
    for r in sorted(rows, key=lambda r: r['lap']):
        v = 'ok' if r['ok'] else f'REJECT ({r["why"]})'
        g = 'safe' if r['w_veh'] is None else f'{r["w_veh"]:.2f}'
        print(f'{g:>6} {r["ay"]:4.1f} {r["lap"]:7.2f} {r["clear"]:7.3f} '
              f'{r["kappa_max"]:6.2f} {r["steer_max"]:6.3f} {r["steer_reserve"]:6.3f} '
              f'{r["frac_saturated"]*100:5.0f}  {v}')

    viable = sorted([r for r in rows if r['ok']], key=lambda r: r['lap'])
    if not viable:
        sys.exit('\nno candidate clears the clearance gate')
    print(f'\n{len(viable)} candidates clear the gate; best ideal '
          f'{viable[0]["lap"]:.2f} s vs baseline {base_lap:.2f} s '
          f'({base_lap - viable[0]["lap"]:+.2f} s)')

    for i, r in enumerate(viable[:args.drive_top]):
        dst = f'data/racelines/{track}_cand{i}.csv'
        subprocess.run(['cp', r['path'] + '.csv', dst], check=True)
        g = 'safe geometry' if r['w_veh'] is None else f'w_veh {r["w_veh"]:.2f}'
        print(f'  cand{i}: {g}, ay {r["ay"]:.1f} -> {dst}  '
              f'(ideal {r["lap"]:.2f} s, clearance {r["clear"]:.3f} m, '
              f'steer reserve {r["steer_reserve"]:.3f} rad)')


if __name__ == '__main__':
    main()
