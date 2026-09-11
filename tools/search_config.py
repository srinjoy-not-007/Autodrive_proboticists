#!/usr/bin/env python3
"""Offline parameter / line search for the Porto trackers.

Everything here runs on the host against ``tools/sim_tracker.py``'s closed-loop
model and the EXACT mesh map (``data/maps/porto_mesh``), which is the truth for
where the walls are.  Nothing here touches a container.

Three entry points:

    python tools/search_config.py calibrate          # sim vs the known live runs
    python tools/search_config.py search --line ...  # (A) mpc2 params at a line
    python tools/search_config.py lines              # (B) drive a set of lines

Objective (one evaluation = ``--laps`` laps on each of ``--seeds`` seeds, with a
0.03 m inside localization bias and 0.045 s of pose latency so that a
configuration that only works from a perfect pose is not rewarded):

    J = median lap + COLL_PEN * collisions/seed + CLEAR_PEN * max(0, 0.10 - min clearance)

A collision costs +10 s in the real scoring, i.e. 1 s of lap over a ten-lap
race; it is charged 5 s here on purpose, because the sim is the optimistic
side of every disagreement we have measured with the live car.
"""

import argparse
import copy
import itertools
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

import sim_tracker as ST                                        # noqa: E402
from pit_crew.mpc2_core import RefPath                          # noqa: E402

MAP = os.path.join(ROOT, 'data/maps/porto_mesh')
COLL_PEN = 5.0
CLEAR_PEN = 60.0
CLEAR_FLOOR = 0.10
# HARD floor, mpc3 onwards.  The September 12 search drove mpc2 to the soft
# 0.10 floor (cut_offset 0.27, J 7.6 s predicted) and the configuration it found
# collided live.  A configuration whose worst footprint clearance is under
# `HARD_CLEAR` is not ranked against the feasible ones at all -- it is out.
HARD_CLEAR = 0.14
HARD_PEN = 500.0

_MAP_CACHE = {}
_PATH_CACHE = {}


def get_map(prefix=MAP):
    if prefix not in _MAP_CACHE:
        from pit_crew import pf_core
        dist, _img, meta = pf_core.load_map(prefix)
        _MAP_CACHE[prefix] = (dist, meta)
    return _MAP_CACHE[prefix]


def get_path(line, kappa_span=0.5, map_prefix=MAP):
    """RefPath with the corridor loaded, cached per worker (read-only after)."""
    key = (line, round(kappa_span, 4), map_prefix)
    if key not in _PATH_CACHE:
        dist, meta = get_map(map_prefix)
        p = RefPath(line, kappa_span_m=kappa_span)
        p.load_corridor(dist, meta)
        _PATH_CACHE[key] = p
    return _PATH_CACHE[key]


# ---------------------------------------------------------------------------
# one evaluation
# ---------------------------------------------------------------------------

def drive(line, params, tracker='mpc2', laps=20, seeds=(1, 2), ay_max=ST.AY_MAX,
          pose_bias=0.03, pose_latency=0.045, pose_noise=0.02, yaw_noise=0.0,
          map_prefix=MAP, warm=1, hard_clear=0.0):
    """Drive ``line`` and return the aggregated metrics."""
    dist, meta = get_map(map_prefix)
    kspan = float(params.get('kappa_span_m', 0.5))
    kw = {k: v for k, v in params.items() if k != 'kappa_span_m'}
    lap_all, coll, clear, paths, p95 = [], 0, [], [], []
    for sd in seeds:
        path = get_path(line, kspan, map_prefix)
        tr = (ST.MPC2Sim(path, **kw) if tracker == 'mpc2' else
              ST.MPC3Sim(path, **kw) if tracker == 'mpc3' else
              ST.LegacyMPCSim(path, **kw))
        out = ST.run(tr, path, laps=laps, dist=dist, meta=meta, seed=sd,
                     pose_noise=pose_noise, yaw_noise=yaw_noise,
                     pose_bias_inside=pose_bias, ay_max=ay_max,
                     pose_latency=pose_latency)
        lt = out['lap_times'][warm:]
        if len(lt) < max(3, laps - warm - 2):
            return dict(ok=False, J=1e3, laps=out['laps'], collisions=99,
                        median=float('nan'), clear=0.0, coll_s=out['coll_s'])
        lap_all += lt
        coll += out['collisions']
        r = out['rec']
        r = r[r[:, 9] >= warm]
        clear.append(float(r[:, 8].min()))
        p95.append(float(np.percentile(np.abs(r[:, 7]), 95)))
        paths.append(out['path_m'] / max(out['laps'], 1))
    n = len(seeds)
    med = float(np.median(lap_all))
    mc = float(min(clear))
    J = (med + COLL_PEN * coll / n
         + CLEAR_PEN * max(0.0, CLEAR_FLOOR - mc))
    if hard_clear and mc < hard_clear:
        J = HARD_PEN + HARD_PEN * (hard_clear - mc) + med
    return dict(ok=True, J=J, median=med, best=float(min(lap_all)),
                collisions=coll, coll_per_seed=coll / n, clear=mc,
                p95=float(max(p95)),
                path_m=float(np.mean(paths)), laps=int(len(lap_all) / n) + warm)


# ---------------------------------------------------------------------------
# search space (A)
# ---------------------------------------------------------------------------
# (lo, hi, kind).  'int' is rounded, everything else is continuous.
SPACE = [
    ('cut_offset_m',     0.00, 0.30, 'f'),
    ('cut_kappa_ref',    0.30, 1.10, 'f'),
    ('w_cut',            0.00, 6.00, 'f'),
    ('barrier_margin',   0.04, 0.26, 'f'),
    ('horizon',         14,   34,    'i'),
    ('step_dt',          0.035, 0.085, 'f'),
    ('cap_ff_headroom',  1.00, 1.60, 'f'),
    ('kappa_bias_gain',  0.00, 1.00, 'f'),
    ('w_lat',            4.0, 40.0,  'f'),
    ('w_yaw',            0.5, 10.0,  'f'),
    ('w_rate',           0.03, 1.50, 'f'),
    ('apply_ahead_s',    0.045, 0.18, 'f'),
]
BASE = dict(cut_offset_m=0.14)

# mpc3 has far fewer knobs on purpose: no cut geometry, no bias estimator, and
# the delay/actuator numbers are measured rather than tuned.  What is left is
# the cost balance and the speed plan's two free parameters.
SPACE3 = [
    ('w_lat',           6.0,  40.0, 'f'),
    ('w_yaw',           0.5,  10.0, 'f'),
    ('w_rate',          0.05,  2.00, 'f'),
    ('w_steer',         0.02,  1.20, 'f'),
    ('barrier_margin',  0.08,  0.26, 'f'),
    ('ay_budget',       5.5,   9.5, 'f'),
    ('k_e',             0.0,   3.0, 'f'),
    ('horizon',        14,    30,   'i'),
    ('step_dt',         0.040, 0.080, 'f'),
]


def decode(vec, space=None):
    p = {}
    for (name, lo, hi, kind), z in zip(space or SPACE, vec):
        v = lo + (hi - lo) * min(1.0, max(0.0, float(z)))
        p[name] = int(round(v)) if kind == 'i' else round(float(v), 5)
    return p


def encode(p, space=None):
    out = []
    for name, lo, hi, kind in (space or SPACE):
        out.append((float(p[name]) - lo) / (hi - lo))
    return np.array(out)


_JOB = {}


def _init(job):
    _JOB.update(job)


def _eval(vec):
    space = SPACE3 if _JOB['tracker'] == 'mpc3' else SPACE
    p = decode(vec, space)
    try:
        r = drive(_JOB['line'], p, tracker=_JOB['tracker'], laps=_JOB['laps'],
                  seeds=tuple(_JOB['seeds']), ay_max=_JOB['ay_max'],
                  pose_bias=_JOB['pose_bias'], pose_latency=_JOB['pose_latency'],
                  pose_noise=_JOB['pose_noise'],
                  hard_clear=_JOB.get('hard_clear', 0.0))
    except Exception as exc:                              # a bad corner of the
        return 1e3, p, dict(ok=False, err=str(exc))       # space, not a bug
    return r['J'], p, r


def search(line, n=400, tracker='mpc2', laps=20, seeds=(1, 2), workers=30,
           ay_max=ST.AY_MAX, pose_bias=0.03, pose_latency=0.045,
           pose_noise=0.02, method='cma', seed=0, out=None, hard_clear=0.0):
    space = SPACE3 if tracker == 'mpc3' else SPACE
    job = dict(line=line, tracker=tracker, laps=laps, seeds=list(seeds),
               ay_max=ay_max, pose_bias=pose_bias, pose_latency=pose_latency,
               pose_noise=pose_noise, hard_clear=hard_clear)
    rng = np.random.default_rng(seed)
    results = []
    with Pool(workers, initializer=_init, initargs=(job,)) as pool:
        # Always start from a random sample: CMA from a single point on a
        # 12-dimensional space with a hard collision cliff finds the cliff and
        # stops.
        n_rand = max(min(60, n // 3), n // 4)
        X = list(rng.random((n_rand, len(space))))
        X.append(encode({**decode([0.5] * len(space), space),
                         **_defaults_vec(space)}, space))
        for j, p, r in pool.imap_unordered(_eval, X, chunksize=1):
            results.append((j, p, r))
        print(f'  random phase: best J {min(r[0] for r in results):.3f}')
        if method == 'cma':
            import cma
            results.sort(key=lambda t: t[0])
            x0 = np.clip(np.mean([encode(p, space) for _j, p, _r in results[:5]],
                                 axis=0), 0.02, 0.98)
            es = cma.CMAEvolutionStrategy(
                list(x0), 0.22,
                {'bounds': [0, 1], 'popsize': workers, 'seed': seed + 1,
                 'verbose': -9, 'maxfevals': n - len(X)})
            while not es.stop():
                X = es.ask()
                got = list(pool.map(_eval, X))
                es.tell(X, [g[0] for g in got])
                results += got
                print(f'  cma gen: best so far J '
                      f'{min(r[0] for r in results):.3f}  '
                      f'gen best {min(g[0] for g in got):.3f}')
    results.sort(key=lambda t: t[0])
    if out:
        with open(out, 'w') as fh:
            json.dump([dict(J=j, params=p, metrics=r) for j, p, r in results[:80]],
                      fh, indent=1)
    return results


def _defaults_vec(space=None):
    space = space or SPACE
    if space is SPACE3:
        from pit_crew import mpc3_core
        d = dict(mpc3_core.DEFAULTS)
    else:
        from pit_crew import mpc2_core
        d = dict(mpc2_core.DEFAULTS)
        d['cut_offset_m'] = 0.14
    return {k: d[k] for k, _lo, _hi, _t in space if k in d}


# ---------------------------------------------------------------------------
# calibration against the live runs
# ---------------------------------------------------------------------------

LIVE = [
    # (label, line, tracker, params, live median, live collisions per 32 laps)
    ('mpc2 cut0.14 ay6.5', 'track_v2_safe_ay6.5', 'mpc2',
     dict(cut_offset_m=0.14), 8.06, 0.0),
    ('mpc2 cut0.18 ay6.5', 'track_v2_safe_ay6.5', 'mpc2',
     dict(cut_offset_m=0.18), 8.09, 1.0),
    ('mpc2 ay6.0', 'track_v2_safe_ay6.0', 'mpc2', dict(cut_offset_m=0.0),
     8.48, 0.0),
    ('legacy ay6.0', 'track_v2_safe_ay6.0', 'mpc', dict(ay_limit=7.8),
     8.02, 0.0),
    ('legacy ay6.5', 'track_v2_safe_ay6.5', 'mpc', dict(ay_limit=8.45),
     7.83, 1.0),
    ('legacy ay7.0', 'track_v2_safe_ay7.0', 'mpc', dict(ay_limit=9.1),
     7.65, 5.0),
]


def calibrate(ay_max_list=(5.5, 6.0, 6.5, 7.5), laps=20, workers=12,
              pose_bias=0.0):
    print(f'{"config":24s} {"live":>12s} ' +
          ' '.join(f'{"ay_max " + str(a):>16s}' for a in ay_max_list))
    for label, stem, tracker, params, med, coll in LIVE:
        line = os.path.join(ROOT, 'data/racelines', stem + '.csv')
        cells = []
        for a in ay_max_list:
            r = drive(line, params, tracker=tracker, laps=laps, seeds=(1,),
                      ay_max=a, pose_bias=pose_bias, pose_noise=0.0)
            cells.append(f'{r["median"]:7.3f}/{r["collisions"]:<2d}'
                         f'{r["clear"]:6.3f}')
        print(f'{label:24s} {med:7.2f}/{coll:<4.0f} ' +
              ' '.join(f'{c:>16s}' for c in cells))



# ---------------------------------------------------------------------------
# (B) hybrid line: let the min-curvature QP move the line ONLY at the hairpins
# ---------------------------------------------------------------------------

def pinch_centerline(src, dst, windows=((10.0, 14.0), (25.0, 29.0)),
                     taper=1.5, keep=0.03):
    """Write a centerline whose corridor is closed except inside ``windows``.

    ``optimize_raceline.py`` has no notion of "optimize this corner only": the
    QP sees one corridor for the whole lap.  Closing the corridor to ``keep``
    metres everywhere but the two hairpins is the same instruction expressed in
    the input, and needs no change to the planner.  The taper matters -- a step
    in the allowed width puts a curvature spike at the join, which the velocity
    profile then pays for over the whole straight.
    """
    cl = np.loadtxt(src, delimiter=',')
    xy = cl[:, :2]
    seg = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    tot = float(seg.sum())
    frac = np.full(len(cl), 0.0)
    for a, b in windows:
        d = np.minimum(np.abs(s - a), np.abs(s - b))
        d = np.where((s >= a) & (s <= b), 0.0, np.minimum(d, tot - d))
        frac = np.maximum(frac, np.clip(1.0 - d / max(taper, 1e-6), 0.0, 1.0))
    frac = 0.5 - 0.5 * np.cos(math.pi * frac)            # smooth the join
    out = cl.copy()
    for c in (2, 3):
        # `keep` has to stay above w_veh/2 or the QP is infeasible: the solver
        # subtracts half the vehicle width from the corridor before it starts.
        k = np.minimum(keep, cl[:, c])
        out[:, c] = k + (cl[:, c] - k) * frac
    np.savetxt(dst, out, delimiter=',', fmt='%.4f',
               header='x_m,y_m,w_tr_right_m,w_tr_left_m', comments='# ')
    print(f'wrote {dst}: corridor open over '
          f'{int((frac > 0.5).sum())} of {len(cl)} points')


def _arc_frac(xy, windows, taper):
    seg = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    tot = float(seg.sum())
    frac = np.zeros(len(xy))
    for a, b in windows:
        d = np.minimum(np.abs(s - a), np.abs(s - b))
        d = np.where((s >= a) & (s <= b), 0.0, np.minimum(d, tot - d))
        frac = np.maximum(frac, np.clip(1.0 - d / max(taper, 1e-6), 0.0, 1.0))
    return 0.5 - 0.5 * np.cos(math.pi * frac)


def hybrid_centerline(src, mc_line, dst, windows=((10.0, 14.0), (25.0, 29.0)),
                      taper=1.5):
    """Centerline that follows the min-curvature line ONLY at the hairpins.

    Pinching the corridor and re-running the QP is the obvious way to say
    "optimize this corner only" and it does not work: ``opt_min_curv`` needs
    more than w_veh/2 of corridor at EVERY point or it reports the problem
    infeasible, so the closed sections have to stay wider than the shift we are
    trying to forbid.  Taking the offsets from a line the QP has already solved
    and masking them is the same idea without the feasibility problem, and the
    result is still a legal centerline input: ``--centerline-only`` then builds
    the speed profile for the blended geometry.
    """
    cl = np.loadtxt(src, delimiter=',')
    xy = cl[:, :2]
    d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    head = np.arctan2(d[:, 1], d[:, 0])
    nx, ny = -np.sin(head), np.cos(head)             # unit left normal
    mc = np.loadtxt(mc_line, delimiter=',')[:, 1:3]
    off = np.empty(len(xy))
    for i in range(len(xy)):
        j = int(np.argmin(np.hypot(mc[:, 0] - xy[i, 0], mc[:, 1] - xy[i, 1])))
        off[i] = (mc[j, 0] - xy[i, 0]) * nx[i] + (mc[j, 1] - xy[i, 1]) * ny[i]
    a = off * _arc_frac(xy, windows, taper)
    a = np.convolve(np.concatenate([a[-5:], a, a[:5]]),
                    np.ones(5) / 5, mode='same')[5:-5]
    out = cl.copy()
    out[:, 0] = xy[:, 0] + a * nx
    out[:, 1] = xy[:, 1] + a * ny
    out[:, 2] = np.maximum(cl[:, 2] + a, 0.05)       # right width grows
    out[:, 3] = np.maximum(cl[:, 3] - a, 0.05)       # left width shrinks
    np.savetxt(dst, out, delimiter=',', fmt='%.4f',
               header='x_m,y_m,w_tr_right_m,w_tr_left_m', comments='# ')
    print(f'wrote {dst}: shift max {np.abs(a).max():.3f} m over '
          f'{int((np.abs(a) > 0.01).sum())} of {len(cl)} points')


# ---------------------------------------------------------------------------
# (B) drive a set of lines with a fixed controller, and (A') stress-test
# ---------------------------------------------------------------------------

_LJOB = {}


def _linit(job):
    _LJOB.update(job)


def _ldrive(item):
    line, params, tracker, tag = item
    try:
        r = drive(line, params, tracker=tracker, laps=_LJOB['laps'],
                  seeds=tuple(_LJOB['seeds']), ay_max=_LJOB['ay_max'],
                  pose_bias=_LJOB['pose_bias'],
                  pose_latency=_LJOB['pose_latency'],
                  pose_noise=_LJOB['pose_noise'])
    except Exception as exc:
        r = dict(ok=False, J=1e3, err=str(exc), median=float('nan'),
                 collisions=99, clear=0.0)
    return tag, os.path.basename(line), tracker, r


def drive_many(items, workers=10, laps=20, seeds=(1, 2), ay_max=ST.AY_MAX,
               pose_bias=0.03, pose_latency=0.045, pose_noise=0.02,
               title=''):
    """items: list of (line_csv, params, tracker, tag)."""
    job = dict(laps=laps, seeds=list(seeds), ay_max=ay_max,
               pose_bias=pose_bias, pose_latency=pose_latency,
               pose_noise=pose_noise)
    with Pool(workers, initializer=_linit, initargs=(job,)) as pool:
        got = list(pool.map(_ldrive, items))
    got.sort(key=lambda t: t[3].get('J', 1e3))
    print(f'\n### {title}  (laps {laps} x {len(seeds)} seeds, '
          f'ay_max {ay_max}, pose bias {pose_bias}, noise {pose_noise})')
    print(f'{"tag":26s} {"line":34s} {"trk":5s} {"J":>7s} {"lap":>7s} '
          f'{"coll":>5s} {"p95":>6s} {"clear":>6s} {"m/lap":>7s}')
    for tag, name, trk, r in got:
        print(f'{tag:26s} {name[:34]:34s} {trk:5s} {r.get("J", 1e3):7.3f} '
              f'{r.get("median", float("nan")):7.3f} '
              f'{r.get("collisions", 99):5d} {r.get("p95", float("nan")):6.3f} '
              f'{r.get("clear", 0.0):6.3f} '
              f'{r.get("path_m", float("nan")):7.3f}')
    return got


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=('calibrate', 'search', 'drive', 'pinch', 'hybrid'))
    ap.add_argument('--line', default=os.path.join(
        ROOT, 'data/racelines/track_v2_safe_ay6.5.csv'))
    ap.add_argument('--tracker', default='mpc2')
    ap.add_argument('--n', type=int, default=400)
    ap.add_argument('--laps', type=int, default=20)
    ap.add_argument('--seeds', default='1,2')
    ap.add_argument('--workers', type=int, default=30)
    ap.add_argument('--ay-max', type=float, default=ST.AY_MAX)
    ap.add_argument('--pose-bias', type=float, default=0.03)
    ap.add_argument('--pose-latency', type=float, default=0.045)
    ap.add_argument('--pose-noise', type=float, default=0.02)
    ap.add_argument('--method', default='cma', choices=('cma', 'random'))
    ap.add_argument('--hard-clear', type=float, default=0.0,
                    help='reject any configuration whose worst footprint '
                         'clearance is below this (m); 0 = soft floor only')
    ap.add_argument('--out', default='')
    ap.add_argument('--taper', type=float, default=3.0,
                    help='metres over which a hairpin-only shift is blended '
                         'back to the centerline; too short and the join is a '
                         'curvature spike the speed profile pays for')
    ap.add_argument('--mc', default='', help='hybrid: the solved '
                    'min-curvature line to borrow hairpin offsets from')
    ap.add_argument('--keep', type=float, default=0.03,
                    help='pinch: half-corridor left open away from the '
                         'hairpins; must exceed w_veh/2 for the QP to solve')
    ap.add_argument('-p', '--param', action='append', default=[])
    args = ap.parse_args()
    seeds = tuple(int(x) for x in args.seeds.split(','))

    if args.cmd == 'hybrid':
        hybrid_centerline(args.line, args.mc, args.out, taper=args.taper)
        return
    if args.cmd == 'pinch':
        pinch_centerline(args.line, args.out, keep=args.keep)
        return
    if args.cmd == 'calibrate':
        calibrate(laps=args.laps, pose_bias=args.pose_bias)
        return
    if args.cmd == 'drive':
        p = {}
        for it in args.param:
            k, v = it.split('=')
            p[k] = float(v)
        r = drive(args.line, p, tracker=args.tracker, laps=args.laps,
                  seeds=seeds, ay_max=args.ay_max, pose_bias=args.pose_bias,
                  pose_latency=args.pose_latency, pose_noise=args.pose_noise,
                  hard_clear=args.hard_clear)
        print(json.dumps(r, indent=1))
        return
    res = search(args.line, n=args.n, tracker=args.tracker, laps=args.laps,
                 seeds=seeds, workers=args.workers, ay_max=args.ay_max,
                 pose_bias=args.pose_bias, pose_latency=args.pose_latency,
                 pose_noise=args.pose_noise, method=args.method,
                 out=args.out or None, hard_clear=args.hard_clear)
    print(f'\ntop 10 for {os.path.basename(args.line)}')
    for j, p, r in res[:10]:
        print(f'  J {j:7.3f}  lap {r.get("median", float("nan")):6.3f}  '
              f'coll {r.get("collisions", -1)}  clear {r.get("clear", 0):.3f}  '
              + ' '.join(f'{k}={v}' for k, v in p.items()))


if __name__ == '__main__':
    main()
