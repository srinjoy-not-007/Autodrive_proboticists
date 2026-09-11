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

Mesh maps
---------
A map rasterised from the simulator's own collision mesh (`map_from_mesh.py`,
stamped `source: mesh`) is ground truth, and three of the constants above were
set against maps that were not. So `make_track_lines.py porto_mesh` switches
itself over: no wall dilation before the flood fill, a trajectory synthesised
from the scene's ordered checkpoints instead of a recon lap, a w_veh sweep that
reaches far enough to buy clearance back, extra smoothing on the QP's lateral
shift, a clearance floor of MESH_MIN_CLEARANCE, and candidates ranked across
both centerline methods rather than only the winning one. Each of those is
justified where it is defined. A recon map takes exactly the path it always
did and produces byte-identical files.

Usage:
  .venv/bin/python tools/make_track_lines.py <track_name>
  .venv/bin/python tools/make_track_lines.py porto_mesh --min-clearance 0.25
  .venv/bin/python tools/make_track_lines.py porto_mesh --out-name try1
"""
import argparse
import os
import shutil
import subprocess
import sys

import yaml

sys.path.insert(0, 'tools')
from footprint_analysis import load_dist, clearance, CORNERS   # noqa: E402
from extract_centerline import ensure_traj                     # noqa: E402

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
PROFILE = ['--ay', '6.0', '--ax', '6.5', '--brake', '4.0', '--jerk-lat', '40',
           '--ay-tight', '4.5', '--kappa-tight', '1.15']

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

# On a MESH map the corridor is the real one -- see `is_mesh` below -- so the
# usable half-width is ~0.075 m larger per side than the recon map reported and
# the small end of the sweep is no longer interesting: w_veh 0.16 there means
# the QP is allowed to put the body 0.16 m from the true wall, which no
# clearance floor will ever accept.
#
# The top end matters far more, and is where the recon-era sweep (which stopped
# at 0.52) could not look: on a recon map the corridor was already 0.15 m
# narrower, so a margin this large had no solution. Measured on Porto's mesh
# map, trajectory-anchored reference, ay 6.0, --alpha-smooth 11, against the
# shipping line's own numbers on the SAME geometry (0.275 m clearance,
# 7.79 s ideal, peak |kappa| 1.019, peak d(kappa)/ds 1.67):
#
#     w_veh 0.54   0.246 m   7.25 s   kappa 0.852   dk/ds 1.32
#     w_veh 0.60   0.292 m   7.32 s   kappa 0.831   dk/ds 1.14
#     w_veh 0.66   0.325 m   7.40 s   kappa 0.837   dk/ds 1.41
#     w_veh 0.72   0.348 m   7.46 s   kappa 0.884   dk/ds 2.24
#     w_veh 0.78   0.334 m   7.51 s   kappa 0.864   dk/ds 3.13
#
# Everything from 0.60 up is MORE room than the shipping line has, while being
# faster and asking less of the tyres. That is the whole point of the exercise
# and it is not reachable without sweeping this far.
W_VEH_MESH = (0.36, 0.42, 0.48, 0.54, 0.60, 0.66, 0.72, 0.78)

# Width of the boxcar applied to the QP's lateral shift (optimize_raceline
# --alpha-smooth). 5 is what every shipping line was built with; on a mesh map
# 11 is better on every axis at once, because min-curvature bounds the INTEGRAL
# of curvature and is free to spend it at one spline junction. Porto mesh,
# anchored, w_veh 0.48, ay 6.0:
#
#     alpha smooth  5    7.31 s   0.237 m   kappa 1.049   dk/ds 1.89
#     alpha smooth 11    7.21 s   0.224 m   kappa 0.874   dk/ds 1.34
#     alpha smooth 15    7.18 s   0.224 m   kappa 0.870   dk/ds 1.34
#     alpha smooth 19    7.19 s   0.206 m   kappa 0.854   dk/ds 1.37
#
# It costs NOTHING in ideal lap -- it gains, because a lower curvature peak
# lets the ay cap allow more speed -- so there is no trade to argue about
# below ~15. 11 is taken rather than 15 because it holds more clearance at the
# wide end of the w_veh sweep, which is where the useful lines live.
MESH_ALPHA_SMOOTH = 11


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


def is_mesh(map_prefix):
    """True when this map was rasterised from the simulator's collision mesh.

    Three things change on a mesh map, and they all point the same way:

      * the walls are exact, closed 0.254 m tubes, so the flood fill needs no
        dilation to seal gaps and the corridor keeps the 0.075 m per side that
        the recon repair was throwing away (Porto: min corridor 1.00 -> 1.15 m);
      * there is no recon lap to anchor on, so the ordered checkpoints out of
        the Unity scene stand in for one (`extract_centerline.ensure_traj`);
      * localization against it is exact, so the clearance floor that was set
        from recon-era collisions is too conservative (see MESH_MIN_CLEARANCE).

    Detected from the `source: mesh` key `map_from_mesh.py` stamps into the
    YAML, not from the file name, so a mesh map called anything still works.
    """
    try:
        with open(map_prefix + '.yaml') as f:
            return yaml.safe_load(f).get('source') == 'mesh'
    except OSError:
        return False


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

# ...and 0.30 is a RECON-ERA number in a second way that matters more than the
# track it was measured on. Every collision behind it happened while the car
# knew its own lateral position to p95 0.17 m against a LiDAR-built map. On the
# exact mesh maps that figure is ~0.02 m (docs/STATUS.md, September 11-12: the
# promoted config localizes against data/maps/porto_mesh and ran 100 laps
# clean). Roughly 0.15 m of the 0.30 m floor was paying for map and pose error
# that no longer exists.
#
# MESH_MIN_CLEARANCE spends part of that back. It is the physics floor
# (ABSOLUTE_FLOOR, 0.22 = worst observed lateral error 0.196 + the map's own
# 0.025 m resolution) rather than a new evidence bar, because no line at this
# clearance has been DRIVEN on a mesh map yet.
#
# *** PROVISIONAL. *** 0.30 traces to collisions; 0.22 traces to an argument.
# Treat any line selected under it as unproven until it has run a race
# distance clean, and keep the safe centerline as the fallback.
#
# What the driven evidence becomes when it is re-measured on the true geometry
# is a separate and much firmer number, because a recon map systematically
# OVERSTATES clearance -- its dilated walls sit inside the real ones. Every
# Porto line we have, scored against both maps:
#
#     recon 0.325 -> mesh 0.275   track_v2_safe, the shipping line: 100 laps clean
#     recon 0.309 -> mesh 0.255
#     recon 0.301 -> mesh 0.250   corrected-map line: 37 laps clean
#     recon 0.283 -> mesh 0.214   w_veh 0.44 line: 4 collisions in 38 laps
#
# The offset is 0.034-0.069 m, mean 0.053. So on true geometry the driven
# boundary is: 0.214 collided (with the old 0.17 m lateral error), 0.250 and
# 0.275 ran clean. A mesh floor of 0.25 is therefore the EVIDENCE bar and
# 0.22 is the physics bar -- and on Porto neither is binding, because the
# w_veh sweep above produces lines at 0.29-0.35 m that are still faster than
# anything shipped. Prefer buying clearance with w_veh over spending it.
MESH_MIN_CLEARANCE = 0.22

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


def build(track, tag, extra, cfg):
    """Both lines for one centerline method. Returns None if extraction fails."""
    mp = f'data/maps/{track}'
    stem = cfg.out_name
    cl = f'data/racelines/{stem}_centerline_{tag}'
    if subprocess.run([VENV, 'tools/extract_centerline.py', mp, cl]
                      + extra + cfg.extract_args,
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
    opened = f'data/racelines/{stem}_opened_{tag}'
    has_opened = subprocess.run(
        [VENV, 'tools/open_corners.py', cl + '.csv', '--out', opened],
        capture_output=True, text=True).returncode == 0

    out = {'centerline': cl}
    cands = [('safe', ['--centerline-only'], None)]
    cands += [('raceline', ['--w-veh', str(w)], w) for w in cfg.w_veh]
    if has_opened:
        cands += [('opened', ['--centerline-only'], None)]

    best = None
    for kind, args, w in cands:
        args = args + cfg.profile
        suffix = tag if w is None else f'{tag}_w{w:g}'
        path = f'data/racelines/{stem}_{kind}_{suffix}'
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
            f'clearance {eff:.3f} < {cfg.min_clearance:.2f} m budget'
            if eff < cfg.min_clearance else '')
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


def configure(argv=None):
    """Everything that used to be a constant, resolved for one track.

    The defaults are exactly the old constants, so a recon map run the old way
    -- `make_track_lines.py track_v2` -- takes the same path it always did and
    produces byte-identical files. Only a mesh map moves.
    """
    ap = argparse.ArgumentParser(
        description='Generate and rank a track\'s racing lines.')
    ap.add_argument('track', help='map stem under data/maps/, e.g. track_v2 '
                                  'or porto_mesh')
    ap.add_argument('--out-name', default=None, metavar='STEM',
                    help='write data/racelines/<STEM>_* instead of '
                         '<track>_*. For generating candidates without '
                         'overwriting a shipping line.')
    ap.add_argument('--min-clearance', type=float, default=None, metavar='M',
                    help=f'worst-corner body clearance a line must leave to be '
                         f'raceable. Default {MIN_CLEARANCE:.2f} on a recon '
                         f'map, {MESH_MIN_CLEARANCE:.2f} on a mesh map '
                         f'(PROVISIONAL -- see the comment on '
                         f'MESH_MIN_CLEARANCE).')
    ap.add_argument('--w-veh', default=None, metavar='LIST',
                    help='comma-separated corridor margins to sweep the '
                         'min-curvature QP over, overriding the default sweep')
    ap.add_argument('--ay', type=float, default=None,
                    help='override the PROFILE lateral-acceleration cap')
    ap.add_argument('--alpha-smooth', type=int, default=None, metavar='W',
                    help=f'smoothing window on the QP\'s lateral shift. '
                         f'Default 5 on a recon map (what every shipping line '
                         f'was built with), {MESH_ALPHA_SMOOTH} on a mesh map.')
    ap.add_argument('--dilate', type=int, default=None, metavar='N',
                    help='cells of wall dilation before the flood fill. '
                         'Default 3 on a recon map (seals LiDAR gaps), 0 on a '
                         'mesh map (the walls are already closed).')
    cfg = ap.parse_args(argv)

    cfg.mesh = is_mesh(f'data/maps/{cfg.track}')
    cfg.out_name = cfg.out_name or cfg.track
    if cfg.min_clearance is None:
        cfg.min_clearance = MESH_MIN_CLEARANCE if cfg.mesh else MIN_CLEARANCE
    if cfg.w_veh is None:
        cfg.w_veh = W_VEH_MESH if cfg.mesh else W_VEH
    else:
        cfg.w_veh = tuple(float(x) for x in cfg.w_veh.split(','))
    dil = cfg.dilate if cfg.dilate is not None else (0 if cfg.mesh else 3)
    cfg.extract_args = ['--dilate', str(dil)] if dil != 3 else []

    cfg.profile = list(PROFILE)
    if cfg.ay is not None:
        cfg.profile[cfg.profile.index('--ay') + 1] = f'{cfg.ay:g}'
    aw = cfg.alpha_smooth if cfg.alpha_smooth is not None else (
        MESH_ALPHA_SMOOTH if cfg.mesh else 5)
    if aw != 5:
        cfg.profile += ['--alpha-smooth', str(aw)]

    if cfg.mesh:
        # The launch scripts read the first row of <map>_traj.csv for the
        # initial pose, and the centerline extractor wants a trajectory to
        # seed and orient from. A mesh map has no recon lap behind it, so
        # stand one up from the scene's own checkpoints.
        ensure_traj(f'data/maps/{cfg.track}')
        print(f'mesh map: no wall dilation, clearance floor '
              f'{cfg.min_clearance:.2f} m '
              f'(PROVISIONAL -- set from the physics bound, not from anything '
              f'driven; 0.25 is the mesh-equivalent of what has run clean), '
              f'alpha smoothing {aw}, w_veh sweep '
              f'{", ".join(f"{w:g}" for w in cfg.w_veh)}')
    return cfg


def main():
    cfg = configure()
    track, stem = cfg.track, cfg.out_name
    built = {}
    for tag, extra in (('skeleton', ['--skeleton']), ('anchored', [])):
        got = build(track, tag, extra, cfg)
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

    for src, dst in ((chosen['centerline'] + '.csv', f'{stem}_centerline'),
                     (chosen['safe']['path'] + '.csv', f'{stem}_safe')):
        shutil.copy(src, f'data/racelines/{dst}.csv')
    # The raceline is promoted to its canonical name ONLY if it passed both
    # gates. It used to be copied unconditionally, which is how an ICRA line
    # that reversed direction twice and asked for a 0.327 m turn radius came to
    # sit in data/racelines/ and be copied into the submission image -- gated
    # out of race.csv, but one PIT_CREW_LINE away from being driven. A line
    # that failed the check must not be reachable by name.
    rl = chosen.get('raceline')
    stale = f'data/racelines/{stem}_raceline.csv'
    if rl and rl['ok']:
        shutil.copy(rl['path'] + '.csv', stale)
    else:
        if os.path.exists(stale):
            os.remove(stale)
        if rl:
            shutil.copy(rl['path'] + '.csv',
                        f'data/racelines/{stem}_raceline_REJECTED.csv')
            print(f'  raceline REJECTED, not promoted; kept as '
                  f'{stem}_raceline_REJECTED.csv for inspection')

    # Race the fastest line that can actually cover our error budget.
    #
    # `pool` is normally the chosen method's lines only: the method is picked on
    # the safe line's clearance, and everything else follows it. That is the
    # RECON-map rule and it does not move, because a recon map is exactly where
    # a skeleton that cut across an infield has to be distrusted wholesale.
    #
    # On a mesh map the corridor is ground truth, so there is no infield-cut
    # failure mode to defend against and confining the search to one method
    # just throws away candidates: measured on Porto's mesh map at ay 6.0, the
    # skeleton has the roomier safe line (0.403 m vs 0.200 m) while the
    # trajectory-anchored w_veh 0.48 raceline is both roomier AND faster than
    # the skeleton's (0.237 m / 7.31 s vs 0.224 m / 7.52 s). Every candidate
    # still passes the same drivability and clearance gates.
    pool = dict(chosen)
    if cfg.mesh:
        for tag, got in built.items():
            for k, v in got.items():
                if isinstance(v, dict):
                    pool[f'{k} ({tag})' if tag != best_tag else k] = v
    viable = [(k, v) for k, v in pool.items()
              if isinstance(v, dict) and v.get('ok')]
    if viable:
        kind, win = min(viable, key=lambda kv: kv[1]['lap'])
    else:
        # Nothing cleared the driven-evidence floor. Fall back to the ROOMIEST
        # line above the physics floor rather than producing nothing -- but
        # never to the fastest, because the whole reason we are here is that
        # clearance is scarce, and ideal lap time has already been shown to
        # mispredict which line actually drives better.
        fallback = [(k, v) for k, v in pool.items()
                    if isinstance(v, dict) and not drivability(v['path'] + '.csv')
                    and v['clearance'] >= min(ABSOLUTE_FLOOR,
                                              cfg.min_clearance)]
        if not fallback:
            sys.exit(f'no drivable line reaches even the '
                     f'{min(ABSOLUTE_FLOOR, cfg.min_clearance):.2f} m '
                     'physical floor on this track — race the reactive stack '
                     '(PIT_CREW_MODE=reactive) rather than this map')
        kind, win = max(fallback, key=lambda kv: kv[1]['clearance'])
        print(f'*** WARNING: nothing cleared the {cfg.min_clearance:.2f} m '
              f'evidence '
              f'floor. Racing the roomiest drivable line instead '
              f'({kind}, {win["clearance"]:.3f} m) — UNPROVEN, drive it before '
              f'trusting it, and keep the reactive stack as the fallback.')
    shutil.copy(win['path'] + '.csv', f'data/racelines/{stem}_race.csv')
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
    print(f'wrote data/racelines/{stem}_{{{",".join(sorted(wrote))}}}.csv')


if __name__ == '__main__':
    main()
