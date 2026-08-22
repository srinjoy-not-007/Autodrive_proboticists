#!/usr/bin/env python3
"""Generate a track's racing lines, choosing both the centerline method AND the
line to race by measurement.

Neither centerline method wins everywhere: the skeleton is better on a simple
loop (Porto worst-corner clearance 0.325 m vs 0.079 m) while on a circuit with
an inner passage it cuts straight across the infield and is unusable, where the
trajectory-anchored method works (ICRA 0.215 m). Rather than pick by hand per
track — which is exactly the sort of choice that does not survive an unseen
track — build both and keep whichever actually measures better.

The same argument applies to safe-centerline vs optimized raceline, and used to
be decided by a constant. On Porto the optimized line is faster in theory
(10.41 s vs 11.05 s ideal) and unusable in practice: 0.146 m of corner
clearance against a localization p95 of 0.298 m gave ~57 collisions, where the
safe line's 0.325 m runs clean. That is a fact about Porto's narrow corridor,
not about optimized racelines, and the finals track is released three days
before the deadline with no time to rediscover it. So: build both, reject
whatever cannot cover the measured error budget, and race the fastest of what
survives.

Outputs, all under data/racelines/<track>_:
  centerline.csv  the chosen centerline
  safe.csv        the conservative centerline-following line (unchanged)
  raceline.csv    the min-curvature optimized line
  race.csv        whichever of the two measured best — the one to race

`race.csv` is written but nothing points at it yet: the shipping entrypoint
still names `safe.csv` deliberately, because a line should be driven before it
is trusted (see docs/STATUS.md).

Usage: .venv/bin/python tools/make_track_lines.py <track_name>
"""
import subprocess
import sys

sys.path.insert(0, 'tools')
from footprint_analysis import load_dist, clearance, CORNERS   # noqa: E402

import numpy as np   # noqa: E402

VENV = '.venv/bin/python'

# The validated profile. These are not tuning knobs — each traces to a
# measurement (docs/STATUS.md, "BREAKTHROUGH (Aug 23)"):
#   ay 3.0    lateral, well under the 4.5 m/s2 the tyres sustain, because the
#             margin is what absorbs localization and tracking error.
#   ax 6.5    longitudinal mu is 0.72, i.e. 7.1 m/s2 available. Acceleration
#             happens on straights where there is no lateral demand, so this is
#             nearly free: ideal lap 11.0 -> 10.1 s on Porto.
#   brake 4.0 braking overlaps corner entry, where lateral demand is rising, so
#             it stays at the value proven over 22 clean laps.
#   jerk 20   the lateral-jerk ceiling the clean shipping profile already met.
# Measured with these: Porto 25 laps 0 collisions median 10.75 s (was 11.5),
# ICRA 15 laps 0 collisions median 18.91 s (was 20.9).
PROFILE = ['--ay', '3.0', '--ax', '6.5', '--brake', '4.0', '--jerk-lat', '20']


def worst_corner(map_prefix, line_csv):
    dist, res, ox, oy = load_dist(map_prefix)
    rl = np.loadtxt(line_csv, delimiter=',')
    xy = rl[:, 1:3]
    d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    head = np.arctan2(d[:, 1], d[:, 0])
    worst = np.full(len(xy), np.inf)
    for cx, cy in CORNERS:
        px = xy[:, 0] + cx * np.cos(head) - cy * np.sin(head)
        py = xy[:, 1] + cx * np.sin(head) + cy * np.cos(head)
        worst = np.minimum(worst, np.nan_to_num(
            clearance(dist, res, ox, oy, np.column_stack([px, py])), nan=np.inf))
    return float(worst.min())


MIN_TURN_R = 0.561      # wheelbase / tan(max steer)

# A line has to leave more room than our own error consumes — but only the part
# of that error which moves the car *sideways*, towards a wall. Decomposing the
# live localization trace against the direction of travel (data/logs/
# loc_norandom.csv):
#
#     total euclidean       median 0.092  p95 0.298  max 0.427
#     along-track           median 0.074  p95 0.246  max 0.410
#     normal (lateral)      median 0.043  p95 0.171  max 0.196
#
# The headline 0.298 m is mostly along-track *lag*: it makes the car early or
# late along the path, not closer to the wall. Using it as a per-side clearance
# floor was the wrong statistic and roughly twice as conservative as the
# geometry requires. The lateral error never exceeded 0.196 m over the run.
#
# So the floor rejects what is physically impossible, not what is merely
# untested: worst observed lateral error (0.196) plus the map's own 0.025 m
# resolution. Measured either side of the gap this leaves open: 0.325 m -> 0
# collisions over 22 laps, 0.146 m -> ~57 collisions and better tracking did not
# rescue it. Anything between 0.22 and 0.325 m is admitted but UNPROVEN — it
# gets written to race.csv, which nothing races until it has been driven.
MIN_CLEARANCE = 0.22


def ideal_lap(line_csv):
    """Lap time the velocity profile claims for this line, in seconds."""
    rl = np.loadtxt(line_csv, delimiter=',')
    s, vx = rl[:, 0], rl[:, 5]
    return float(np.sum(np.diff(s) / np.maximum(vx[:-1], 0.1)))


def drivability(line_csv):
    """Reasons the car physically cannot follow this line, if any.

    Clearance alone is not enough: the ICRA line measured 0.414 m of corner
    clearance and was still undrivable, because a skeleton spur made it double
    back on itself at one point. Anything that reverses direction or turns
    tighter than the steering lock is disqualifying, however roomy it looks.
    """
    rl = np.loadtxt(line_csv, delimiter=',')
    xy = rl[:, 1:3]
    d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    head = np.arctan2(d[:, 1], d[:, 0])
    dh = np.abs((np.diff(np.concatenate([head, head[:1]])) + np.pi)
                % (2 * np.pi) - np.pi)
    bad = []
    n_rev = int((dh > 2.0).sum())
    if n_rev:
        bad.append(f'{n_rev} direction reversal(s)')
    kmax = float(np.abs(rl[:, 4]).max())
    if kmax > 1.0 / MIN_TURN_R:
        bad.append(f'min radius {1 / kmax:.3f} m < {MIN_TURN_R:.3f} m lock')
    return bad


def build(track, tag, extra):
    """Both lines for one centerline method. Returns None if extraction fails."""
    mp = f'data/maps/{track}'
    cl = f'data/racelines/{track}_centerline_{tag}'
    if subprocess.run([VENV, 'tools/extract_centerline.py', mp, cl] + extra,
                      capture_output=True, text=True).returncode != 0:
        print(f'  {tag}: centerline extraction failed')
        return None

    out = {'centerline': cl}
    for kind, args in (('safe', ['--centerline-only']), ('raceline', [])):
        args = args + PROFILE
        path = f'data/racelines/{track}_{kind}_{tag}'
        if subprocess.run([VENV, 'tools/optimize_raceline.py', cl + '.csv',
                           '--out', path] + args,
                          capture_output=True, text=True).returncode != 0:
            print(f'  {tag:9s} {kind:9s} generation failed')
            continue
        bad = drivability(path + '.csv')
        try:
            wc = worst_corner(mp, path + '.csv')
        except Exception as e:
            print(f'  {tag:9s} {kind:9s} clearance check failed ({e})')
            continue
        lap = ideal_lap(path + '.csv')
        why = '; '.join(bad) if bad else (
            f'clearance {wc:.3f} < {MIN_CLEARANCE:.2f} m budget'
            if wc < MIN_CLEARANCE else '')
        print(f'  {tag:9s} {kind:9s} clearance {wc:.3f} m  ideal lap {lap:6.2f} s'
              f'{"   REJECTED: " + why if why else ""}')
        out[kind] = {'path': path, 'clearance': wc, 'lap': lap, 'ok': not why}
    return out


def main():
    track = sys.argv[1]
    built = {}
    for tag, extra in (('skeleton', ['--skeleton']), ('anchored', [])):
        got = build(track, tag, extra)
        if got and 'safe' in got:
            built[tag] = got

    if not built:
        sys.exit('no line produced by either centerline method')

    # The centerline method is still chosen on the safe line's clearance: it is
    # the one line guaranteed to exist for both methods, and it is what exposes
    # a skeleton cutting across an infield.
    best_tag = max(built, key=lambda t: built[t]['safe']['clearance'])
    chosen = built[best_tag]
    print(f'centerline method: {best_tag} '
          f'({chosen["safe"]["clearance"]:.3f} m on the safe line)')

    for src, dst in ((chosen['centerline'] + '.csv', f'{track}_centerline'),
                     (chosen['safe']['path'] + '.csv', f'{track}_safe')):
        subprocess.run(['cp', src, f'data/racelines/{dst}.csv'], check=True)
    if 'raceline' in chosen:
        subprocess.run(['cp', chosen['raceline']['path'] + '.csv',
                        f'data/racelines/{track}_raceline.csv'], check=True)

    # Race the fastest line that can actually cover our error budget.
    viable = [(k, v) for k, v in chosen.items()
              if isinstance(v, dict) and v.get('ok')]
    if not viable:
        sys.exit('no line clears the error budget on this track — '
                 'widen the corridor or improve localization before racing')
    kind, win = min(viable, key=lambda kv: kv[1]['lap'])
    subprocess.run(['cp', win['path'] + '.csv',
                    f'data/racelines/{track}_race.csv'], check=True)
    other = [f'{k} {v["lap"]:.2f} s' for k, v in viable if k != kind]
    print(f'race line: {kind} — ideal {win["lap"]:.2f} s, '
          f'clearance {win["clearance"]:.3f} m'
          + (f' (beat {", ".join(other)})' if other else
             ' (only candidate that cleared the budget)'))
    print(f'wrote data/racelines/{track}_{{centerline,safe,raceline,race}}.csv')


if __name__ == '__main__':
    main()
