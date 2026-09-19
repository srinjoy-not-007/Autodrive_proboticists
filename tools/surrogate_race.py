#!/usr/bin/env python3
"""Race a line on the LOG-FITTED surrogate and print laps / collisions / pace.

This is `tools/sim_tracker.py --plant-fit` with one screen of output: it runs
the REAL mpc3 control law (`src/pit_crew/mpc3_core.py`, imported, not a copy)
against the plant and sensor chain identified from the live logs, at a chosen
sensor period, and reports

  * laps completed and every collision with the `s` it happened at,
  * the lap-time median,
  * the per-2 m segment times, optionally differenced against a live log so the
    pace can be compared bin by bin rather than as one number.

The shipping controller flags are the default, so a candidate LINE is one
argument.  Anything after `-r` is passed to mpc3 exactly as `run_exp.sh` passes
`-p`, so a candidate PARAM is one argument too.

    tools/surrogate_race.py --line finals_mesh_exitm_m10t6_h3_race --frame 0.097
    tools/surrogate_race.py --line finals_mesh_hyb675b_race -r ay_limit=16.0
    tools/surrogate_race.py --line finals_mesh_exitm_m10t6_h3_race --frame 0.058 \
        --compare-log data/logs/race_20260919_044701.csv

What it is and is not: the surrogate reproduces the shipping config's per-2 m
segment times at both periods to a median of 0.02 s, and it reproduces the
LIVE VERDICT (clean / collides, and at which period) of the arms in
`tools/surrogate_validate.py`.  It is evidence for a screening decision, never
for shipping: the live gates are unchanged.

Offline only; never touches the simulator.  Pin it to your own cores:
    taskset -c 16-27 nice -n 19 env OMP_NUM_THREADS=1 ... .venv/bin/python ...
"""

import argparse
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src'))

import racelog as RL                                          # noqa: E402

# The Sep 19 shipping controller, exactly as `submission/autodrive_devkit.sh`
# leaves it: exit-margin h3 line, rate adaptivity, the real brake, the publish
# gate and the curvature-gated exit servo.
SHIPPING = [
    'rate_adaptive=1', 'w_dcmd=50.0', 'cmd_tau=0.10', 'steer_tau=0.04',
    'barrier_margin=0.40', 'thr_brake_lead_s=0.05',
    'brake_mode=zero', 'brake_a_min=1.5', 'brake_dv=0.3', 'brake_a_full=4.6',
    'pub_gate_corr=0.05',
    'servo_k_p=0.5', 'servo_head_max=0.3', 'servo_head_kappa=0.4',
]


def resolve_line(name):
    if os.path.exists(name):
        return name
    for cand in (os.path.join(ROOT, 'data/racelines', name),
                 os.path.join(ROOT, 'data/racelines', name + '.csv')):
        if os.path.exists(cand):
            return cand
    raise SystemExit(f'no such raceline: {name}')


def seg_times(s, t, total, bin_m=2.0):
    """Median time to cross each `bin_m` bin of the line, over laps."""
    nb = int(math.ceil(total / bin_m))
    edges = np.arange(nb) * bin_m
    ds = np.diff(s)
    su = s.copy()
    su[1:] += np.cumsum(np.where(ds < -total * 0.5, 1, 0)) * total
    pos, bidx = [], []
    for m in range(int(math.floor(su[0] / total)) - 1,
                   int(math.ceil(su[-1] / total)) + 2):
        for j in range(nb):
            pos.append(m * total + edges[j])
            bidx.append(j)
    pos = np.array(pos, float)
    bidx = np.array(bidx, int)
    sel = (pos >= su[0]) & (pos <= su[-1])
    pos, bidx = pos[sel], bidx[sel]
    o = np.argsort(pos)
    pos, bidx = pos[o], bidx[o]
    tc = np.interp(pos, su, t)
    out = [[] for _ in range(nb)]
    for b, dt in zip(bidx[:-1], np.diff(tc)):
        if 0.0 < dt < 5.0:
            out[b].append(dt)
    return edges, np.array([np.median(v) if v else np.nan for v in out])


def live_segments(log_path, line_path, bin_m=2.0):
    log = RL.load_log(log_path)
    keep = RL.dedupe_ips_mask(log)
    s_line, pts, _psi, _k, _v = RL.load_line(line_path)
    total = float(s_line[-1] + (s_line[1] - s_line[0]))
    idx = RL.project(pts, log['truth_xy'][keep])
    return seg_times(s_line[idx], log['t'][keep], total, bin_m) + (total,)


def run_one(args, seed, params):
    rec = os.path.join(args.tmp, 'surrogate_%s_%s_%d_%d_%d.csv'
                       % (args.tracker,
                          os.path.basename(args.line).replace('.csv', ''),
                          int(args.frame * 1000), seed, os.getpid()))
    cmd = [sys.executable, os.path.join(HERE, 'sim_tracker.py'),
           '--tracker', args.tracker, '--map', args.map, '--line', args.line,
           '--laps', str(args.laps), '--frame', str(args.frame),
           '--seed', str(seed), '--plant-fit', '--dump-rec', rec]
    for p in params:
        cmd += ['-r', p]
    for chunk in args.plant:
        cmd += chunk.split()
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    if p.returncode != 0:
        raise SystemExit(p.stderr[-3000:] or p.stdout[-3000:])
    laps = colls = None
    coll_s = ''
    for ln in p.stdout.splitlines():
        t = ln.strip()
        if t.startswith('laps '):
            laps = int(t.split()[1])
        if t.startswith('collisions'):
            bits = t.split()
            colls = int(bits[1])
            if 'at s' in t:
                coll_s = t.split('at s', 1)[1].strip()
    d = np.genfromtxt(rec, delimiter=',', names=True)
    try:
        os.remove(rec)
    except OSError:
        pass
    return dict(laps=laps, colls=colls, coll_s=coll_s, rec=d, out=p.stdout)


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__)
    ap.add_argument('--line', required=True,
                    help='raceline name under data/racelines, or a path')
    ap.add_argument('--map', default='data/maps/finals_mesh')
    ap.add_argument('--frame', type=float, default=0.097,
                    help="sensor period, s (organisers' box ~0.097, real "
                         'display 0.058, Xvfb 0.088-0.106)')
    ap.add_argument('--laps', type=int, default=32)
    ap.add_argument('--tracker', default='mpc3',
                    help='control law to race: mpc3 (shipping) or one of the '
                         'overhaul candidates mpcc / ffb / fastpub')
    ap.add_argument('--seeds', type=int, default=1,
                    help='independent localization-noise seeds to race')
    ap.add_argument('-r', '--mpc3-param', action='append', default=[],
                    metavar='K=V',
                    help='mpc3 override, exactly as run_exp.sh -p; repeatable')
    ap.add_argument('--no-shipping', action='store_true',
                    help='do NOT start from the shipping controller flags')
    ap.add_argument('--plant', action='append', default=[],
                    help='extra sim_tracker plant flag (repeatable)')
    ap.add_argument('--compare-log', default='',
                    help='live race log to difference the segment times against')
    ap.add_argument('--bin', type=float, default=2.0)
    ap.add_argument('--tmp', default='/tmp')
    ap.add_argument('--quiet', action='store_true',
                    help='laps/collisions/median only, no segment table')
    a = ap.parse_args()
    a.line = resolve_line(a.line)
    params = ([] if a.no_shipping else list(SHIPPING)) + list(a.mpc3_param)

    s_line = RL.load_line(a.line)[0]
    total = float(s_line[-1] + (s_line[1] - s_line[0]))

    runs = [run_one(a, s, params) for s in range(a.seeds)]
    laps = sum(r['laps'] for r in runs)
    colls = sum(r['colls'] for r in runs)
    where = '; '.join(r['coll_s'] for r in runs if r['coll_s'])

    lap_meds, segs = [], []
    for r in runs:
        e, m = seg_times(r['rec']['s'], r['rec']['t'], total, a.bin)
        segs.append(m)
        lt = np.diff(r['rec']['t'][np.diff(r['rec']['lap'], prepend=0) > 0])
        if len(lt):
            lap_meds.append(float(np.median(lt)))
    seg = np.nanmedian(np.vstack(segs), axis=0)

    print('line   %s' % os.path.relpath(a.line, ROOT))
    print('frame  %.3f s   laps %d (%d seeds x %d)   params %s'
          % (a.frame, laps, a.seeds, a.laps, ' '.join(params)))
    print('LAPS %d   COLLISIONS %d%s'
          % (laps, colls, ('   at s ' + where) if where else ''))
    print('lap time  median %.3f s (sum of segment medians %.3f s)'
          % (np.median(lap_meds) if lap_meds else float('nan'), np.nansum(seg)))
    if a.quiet:
        return
    if a.compare_log:
        el, ml, tot_l = live_segments(a.compare_log, a.line, a.bin)
        err = seg - ml
        print('live log %s: lap %.3f s, surrogate %+0.3f s'
              % (os.path.basename(a.compare_log), np.nansum(ml),
                 np.nansum(seg) - np.nansum(ml)))
        print('per-bin |error|  p50 %.3f  p95 %.3f  max %.3f s'
              % (np.nanmedian(np.abs(err)), np.nanquantile(np.abs(err), .95),
                 np.nanmax(np.abs(err))))
        print('%7s %8s %8s %8s' % ('s', 'live', 'surr', 'delta'))
        for i in range(len(seg)):
            print('%7.1f %8.3f %8.3f %+8.3f' % (el[i], ml[i], seg[i], err[i]))
    else:
        print('%7s %8s' % ('s', 'seg_s'))
        for i in range(len(seg)):
            print('%7.1f %8.3f' % (i * a.bin, seg[i]))


if __name__ == '__main__':
    main()
