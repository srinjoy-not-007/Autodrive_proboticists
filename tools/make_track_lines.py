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
import os
import shutil
import subprocess
import sys

sys.path.insert(0, 'tools')
from footprint_analysis import load_dist, clearance, CORNERS   # noqa: E402

import numpy as np   # noqa: E402

VENV = '.venv/bin/python'

# The validated profile. These are not tuning knobs — each traces to a
# measurement (docs/STATUS.md, "BREAKTHROUGH (Aug 23)"):
#   ay 3.5    lateral. Swept against race time, not best lap, with the steering
#             cap in place: 3.0 -> 107.5 s, 3.5 -> 101.2 s, 4.0 -> 105.4 s. 4.0
#             holds the quickest single lap of the sweep (9.44 s) and loses on
#             race time to two collisions, which is the whole lesson of this
#             project in one row.
#   ax 6.5    longitudinal mu is 0.72, i.e. 7.1 m/s2 available. Acceleration
#             happens on straights where there is no lateral demand, so this is
#             nearly free: ideal lap 11.0 -> 10.1 s on Porto.
#   brake 4.0 braking overlaps corner entry, where lateral demand is rising, so
#             it stays at the value proven over 22 clean laps.
#   jerk 20   the lateral-jerk ceiling the clean shipping profile already met.
# Measured with these: Porto 25 laps 0 collisions median 10.75 s (was 11.5),
# ICRA 15 laps 0 collisions median 18.91 s (was 20.9).
# ay 5.0, not the 3.5 that shipped until 2026-08-28. The old cap was set because
# ay 4.0 collided 23 times and ay 5.0 collided 31 -- but that was measured while
# the CONTROLLER HAD NO BRAKES: ThrottleController's braking branch was gated
# behind kp>0, which nothing sets, so every throttle command was positive and
# the car shed speed only by coasting. It could plan a corner-entry speed it had
# no way to reach. With braking enabled (mpc `brake_gain`, default 0.3) the same
# ay 5.0 profile runs perfectly clean. Driven, Porto, ~40 laps each:
#     ay 3.5  9.72 s median  0 collisions   race 97.2 s   (no braking)
#     ay 4.0  9.54 s         0              race 95.4 s
#     ay 4.5  9.31 s         0              race 93.1 s
#     ay 5.0  9.14 s         0              race 91.4 s   <-- optimum
#     ay 5.5  9.20 s         0              race 92.0 s
#     ay 6.0  9.44 s         0              race 94.4 s
# ay 6.0 turning back is a real limit, not an artefact: past ~5 the car cannot
# deliver the planned curvature and runs wide. Confirmed on ICRA, never tuned
# on: 17.36 -> 16.53 s median, 0 collisions across 29 laps.
# Aug 30: ay 5.0 -> 4.5 and jerk 20 -> 40, together. They only make sense
# together. The lateral-jerk limiter's sign-reversal branch was infeasible --
# it drove one side of the reversal to the 0.5 m/s floor instead of splitting
# the budget -- and at j_max 20 that accident was ALSO the corner-entry brake
# into Porto's s ~ 16 corner, the one that produces every collision. Fixing the
# branch and freeing the cap (at j_max >= 40 it stops binding: ideal 8.5 -> 8.2 s
# at ay 5.0) removes that accidental brake, so the ay cap has to do the job
# honestly instead. Driven on Porto, one session, scored the competition's way:
#
#   ay 5.0  jerk 20 (old shipping)  18 laps  0 coll  median 9.179   91.8 s
#   ay 5.0  jerk 40                 33 laps  1 coll  median 8.944   92.5 s
#   ay 5.5  jerk 40                 34 laps  8 coll  median 8.899  111.8 s
#   ay 4.8  jerk 40                 37 laps  1 coll  median 8.979   92.5 s
#   ay 4.5  jerk 40                 36 laps  0 coll  median 9.112   91.1 s  <--
#
# The collision threshold sits between 4.5 and 4.8, which is exactly the
# measured sustained cornering limit of 4.64 m/s2. Raising the cap does not
# work even with the limiter fixed: ay 5.5 buys 0.045 s of median and pays 8
# collisions for it.
PROFILE = ['--ay', '4.5', '--ax', '6.5', '--brake', '4.0', '--jerk-lat', '40']

# How much lateral margin the min-curvature QP is told to keep, in metres, on
# top of the swept body footprint. It used to be left at the 0.16 default, and
# a sweep that stopped at 0.32 produced the standing conclusion that "on Porto
# the geometry is settled" -- the optimized line buying 0.05 s for 44% of the
# clearance. That conclusion was an artefact of the grid being too narrow.
#
# Widening the margin pushes the solution off the corridor edges, which is what
# stops min-curvature from spiking locally: it minimises the INTEGRAL of kappa
# and is otherwise free to spend all its curvature in one corner. Measured on
# Porto with the shipping profile, worst-corner clearance / peak |kappa| /
# ideal lap:
#
#     safe centerline   0.325 m   1.019   9.59 s
#     w_veh 0.16        0.146 m   1.717   9.56 s   (rejected on clearance)
#     w_veh 0.32        0.225 m   1.506   9.44 s
#     w_veh 0.44        0.283 m   1.153   9.28 s
#     w_veh 0.52        0.309 m   1.266   9.90 s
#
# Non-monotonic in both clearance and time, so the value cannot be reasoned to
# and must be swept per track -- on ICRA every one of these is rejected on the
# steering lock and the safe centerline still wins. Credit: Vidit Gupta's
# review spotted that the parameter had never been pushed past its default.
W_VEH = (0.16, 0.24, 0.32, 0.40, 0.44, 0.48, 0.52)


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
# rescue it.
#
# That middle range has since been DRIVEN, and 0.22 was too generous. Same
# session, same controller, 37-40 laps each, Porto:
#
#     0.283 m (w_veh 0.44)        4 collisions / 38 laps   race 104.8 s
#     0.301 m (corrected map)     0 collisions / 37 laps   race  99.0 s
#     0.325 m (safe centerline)   0 collisions / 37 laps   race  97.2 s
#
# The 0.283 m line was 0.29 s/lap FASTER and still lost the race, because every
# one of its collisions was at its own tightest point. Worse for this file: its
# *ideal* lap (9.28 s) beat the safe line's (9.59 s) while its DRIVEN lap lost.
# Offline lap time mispredicted the ranking, so the selector below -- which
# picks the fastest ideal lap that clears this floor -- will ship the slower
# line unless the floor stops it. On an unseen track nobody would notice.
#
# 0.30 m sits above everything that has collided here and below everything that
# has run clean. The 0.301 m line ran clean only with the corrected-LiDAR map's
# better localization, so on an uncorrected map treat 0.30 as the edge rather
# than a comfortable margin.
MIN_CLEARANCE = 0.30

# ...but 0.30 is derived entirely from PORTO collisions, and the finals track is
# released three days before the deadline. On a genuinely narrow circuit every
# candidate could sit below it, and refusing outright would abort the drill and
# leave us with nothing but the reactive fallback.
#
# So the floor is two-tier. 0.30 is the EVIDENCE bar: above it, lines have been
# driven clean here. ABSOLUTE_FLOOR is the PHYSICS bar and is not track-specific
# — worst observed lateral localization error (0.196 m) plus the map's own
# 0.025 m resolution. Below that the car cannot be kept off the wall by any
# amount of good driving, so refusing is correct. Between the two, race the
# roomiest line available and say loudly that it is unproven.
ABSOLUTE_FLOOR = 0.22


def ideal_lap(line_csv):
    """Lap time the velocity profile claims for this line, in seconds."""
    rl = np.loadtxt(line_csv, delimiter=',')
    s, vx = rl[:, 0], rl[:, 5]
    return float(np.sum(np.diff(s) / np.maximum(vx[:-1], 0.1)))


def drivability(line_csv):
    """Reasons the car physically cannot follow this line, if any.

    Clearance alone is not enough: the ICRA line measured 0.414 m of corner
    clearance and was still undrivable, because a skeleton spur made it double
    back on itself at one point. A line that reverses direction is disqualifying
    however roomy it looks.

    A line merely TIGHTER THAN THE STEERING LOCK is a different case, and used
    to be rejected here just as hard. That was wrong, and it cost us both 2025
    circuits: their safe lines came out with healthy clearance (0.364 m and
    0.325 m) and good ideal laps, and were thrown away for peak radii of 0.543
    and 0.487 m against the 0.561 m lock. What being tight actually costs is a
    BOUNDED apex deviation of (r_lock - r_line) -- 0.018 m and 0.074 m -- which
    belongs in the clearance budget, not in a separate veto. `apex_deviation`
    below returns it and the caller charges it against the corner clearance.
    Only a radius less than half the lock is hopeless enough to veto outright.
    Porto and ICRA-2026 never reach the lock at all, so nothing there moves.
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
    if kmax > 2.0 / MIN_TURN_R:
        bad.append(f'min radius {1 / kmax:.3f} m, under half the '
                   f'{MIN_TURN_R:.3f} m lock')
    return bad


def apex_deviation(line_csv):
    """How far outside its own line the car is pushed at the tightest corner.

    Zero unless the line is tighter than the car can steer; then the car traces
    the lock radius through the apex and sits (r_lock - r_line) wide of the
    plan. Charged against corner clearance by the caller.
    """
    rl = np.loadtxt(line_csv, delimiter=',')
    kmax = float(np.abs(rl[:, 4]).max())
    if kmax <= 0.0:
        return 0.0
    return max(0.0, MIN_TURN_R - 1.0 / kmax)


def build(track, tag, extra):
    """Both lines for one centerline method. Returns None if extraction fails."""
    mp = f'data/maps/{track}'
    cl = f'data/racelines/{track}_centerline_{tag}'
    if subprocess.run([VENV, 'tools/extract_centerline.py', mp, cl] + extra,
                      capture_output=True, text=True).returncode != 0:
        print(f'  {tag}: centerline extraction failed')
        return None

    # A plain centerline follows the corridor middle, and on a track with a real
    # hairpin the middle is tighter than the car can steer. Measured on both
    # 2025 circuits: once the tracing bug was fixed the safe line came out with
    # healthy clearance (0.325 m and 0.364 m) and was still rejected, for
    # `min radius 0.487 m` / `0.543 m` against the 0.561 m steering lock -- a
    # smooth ramp to a 3-4 point plateau, i.e. a genuine hairpin, not a spike.
    # Opening those corners costs track width, which is exactly the trade the
    # car needs there, and `open_corners.py` projects every iteration back
    # inside the corridor so clearance stays a hard constraint. Porto and
    # ICRA-2026 never reach the lock limit, so this candidate only ever adds an
    # option on tracks that would otherwise produce nothing.
    opened = f'data/racelines/{track}_opened_{tag}'
    has_opened = subprocess.run(
        [VENV, 'tools/open_corners.py', cl + '.csv', '--out', opened],
        capture_output=True, text=True).returncode == 0

    out = {'centerline': cl}
    cands = [('safe', ['--centerline-only'], None)]
    cands += [('raceline', ['--w-veh', str(w)], w) for w in W_VEH]
    if has_opened:
        cands += [('opened', ['--centerline-only'], None)]

    best = None
    for kind, args, w in cands:
        args = args + PROFILE
        suffix = tag if w is None else f'{tag}_w{w:g}'
        path = f'data/racelines/{track}_{kind}_{suffix}'
        label = f'{kind}' if w is None else f'{kind} w{w:.2f}'
        src = (opened if kind == 'opened' else cl) + '.csv'
        if subprocess.run([VENV, 'tools/optimize_raceline.py', src,
                           '--out', path] + args,
                          capture_output=True, text=True).returncode != 0:
            # The QP runs out of corridor at large w_veh. That is an answer
            # about the track, not an error -- report and carry on.
            print(f'  {tag:9s} {label:15s} no solution (corridor exhausted)')
            continue
        bad = drivability(path + '.csv')
        try:
            wc = worst_corner(mp, path + '.csv')
        except Exception as e:
            print(f'  {tag:9s} {label:15s} clearance check failed ({e})')
            continue
        lap = ideal_lap(path + '.csv')
        dev = apex_deviation(path + '.csv')
        eff = wc - dev
        why = '; '.join(bad) if bad else (
            f'clearance {eff:.3f} < {MIN_CLEARANCE:.2f} m budget'
            if eff < MIN_CLEARANCE else '')
        note = f' (-{dev:.3f} apex)' if dev > 1e-4 else ''
        print(f'  {tag:9s} {label:15s} clearance {wc:.3f}{note} m  '
              f'ideal lap {lap:6.2f} s'
              f'{"   REJECTED: " + why if why else ""}')
        rec = {'path': path, 'clearance': eff, 'lap': lap, 'ok': not why,
               'w_veh': w}
        if kind == 'safe':
            out['safe'] = rec
        elif kind == 'opened':
            out['opened'] = rec
        elif rec['ok'] and (best is None or lap < best['lap']):
            best = rec
        elif best is None:
            out.setdefault('_rejected', rec)   # something to show if all fail
    # Degrade gracefully rather than shipping nothing: the plain centerline is
    # preferred, and the opened-corner line only stands in when the plain one is
    # not drivable. On Porto and ICRA-2026 the plain line always passes, so this
    # never fires and those tracks are untouched.
    if not out.get('safe', {}).get('ok') and out.get('opened', {}).get('ok'):
        print(f'  {tag:9s} {"->":15s} safe line is not drivable; shipping the '
              f'opened-corner line instead '
              f'({out["opened"]["clearance"]:.3f} m, '
              f'{out["opened"]["lap"]:.2f} s ideal)')
        out['safe'] = out['opened']
    if best is not None:
        out['raceline'] = best
    elif '_rejected' in out:
        out['raceline'] = out.pop('_rejected')
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
        shutil.copy(src, f'data/racelines/{dst}.csv')
    # The raceline is promoted to its canonical name ONLY if it passed both
    # gates. It used to be copied unconditionally, which is how an ICRA line
    # that reversed direction twice and asked for a 0.327 m turn radius came to
    # sit in data/racelines/ and be copied into the submission image -- gated
    # out of race.csv, but one PIT_CREW_LINE away from being driven. A line
    # that failed the check must not be reachable by name.
    rl = chosen.get('raceline')
    stale = f'data/racelines/{track}_raceline.csv'
    if rl and rl['ok']:
        shutil.copy(rl['path'] + '.csv', stale)
    else:
        if os.path.exists(stale):
            os.remove(stale)
        if rl:
            shutil.copy(rl['path'] + '.csv',
                        f'data/racelines/{track}_raceline_REJECTED.csv')
            print(f'  raceline REJECTED, not promoted; kept as '
                  f'{track}_raceline_REJECTED.csv for inspection')

    # Race the fastest line that can actually cover our error budget.
    viable = [(k, v) for k, v in chosen.items()
              if isinstance(v, dict) and v.get('ok')]
    if viable:
        kind, win = min(viable, key=lambda kv: kv[1]['lap'])
    else:
        # Nothing cleared the driven-evidence floor. Fall back to the ROOMIEST
        # line above the physics floor rather than producing nothing -- but
        # never to the fastest, because the whole reason we are here is that
        # clearance is scarce, and ideal lap time has already been shown to
        # mispredict which line actually drives better.
        fallback = [(k, v) for k, v in chosen.items()
                    if isinstance(v, dict) and not drivability(v['path'] + '.csv')
                    and v['clearance'] >= ABSOLUTE_FLOOR]
        if not fallback:
            sys.exit(f'no drivable line reaches even the {ABSOLUTE_FLOOR:.2f} m '
                     'physical floor on this track — race the reactive stack '
                     '(PIT_CREW_MODE=reactive) rather than this map')
        kind, win = max(fallback, key=lambda kv: kv[1]['clearance'])
        print(f'*** WARNING: nothing cleared the {MIN_CLEARANCE:.2f} m evidence '
              f'floor. Racing the roomiest drivable line instead '
              f'({kind}, {win["clearance"]:.3f} m) — UNPROVEN, drive it before '
              f'trusting it, and keep the reactive stack as the fallback.')
    shutil.copy(win['path'] + '.csv', f'data/racelines/{track}_race.csv')
    other = [f'{k} {v["lap"]:.2f} s' for k, v in viable if k != kind]
    if not viable:
        why = ' (roomiest drivable line; nothing cleared the evidence floor)'
    elif other:
        why = f' (beat {", ".join(other)})'
    else:
        why = ' (only candidate that cleared the budget)'
    print(f'race line: {kind} — ideal {win["lap"]:.2f} s, '
          f'clearance {win["clearance"]:.3f} m' + why)
    wrote = ['centerline', 'safe', 'race'] + (['raceline'] if rl and rl['ok'] else [])
    print(f'wrote data/racelines/{track}_{{{",".join(sorted(wrote))}}}.csv')


if __name__ == '__main__':
    main()
