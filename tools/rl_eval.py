#!/usr/bin/env python3
"""A/B a trained residual policy against the shipping controller.

Two plants, because they answer different questions:

* the **fitted** surrogate (`tools/sim_tracker.py --plant-fit`) is the one
  validated against live pace and live verdicts, so its lap times are the
  number to quote -- and its clean sheets mean nothing (section 4c);
* the **pessimistic, domain-randomised** plant from `tools/rl_env.py` is what
  the policy was trained against, so it is where a policy that learned to
  exploit the surrogate shows up as fragile.

    tools/rl_eval.py data/rl/residual.npz --laps 100 --seeds 2

HOST RULE: at most 6 single-threaded workers on cores 22-27, nice 19.
"""

import argparse
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src'))

import rl_env as E                                              # noqa: E402
from pit_crew import residual_policy as RP                      # noqa: E402

_T = {}


def _init(line, mp):
    _T['track'] = E.Track(line, mp)


def _pess(job):
    """One deterministic episode on the pessimistic plant."""
    npz, seed, laps, frame = job
    track = _T['track']
    pol = RP.ResidualPolicy(npz) if npz else None

    def act(obs):
        if pol is None:
            return np.zeros(2), None
        # The hook applies the bounds and the gate, so hand it the RAW tanh
        # outputs the net produced, not the already-scaled residual.
        return pol._forward(obs), None

    rng = np.random.default_rng(seed)
    dom = E.sample_domain(rng, frame=frame)
    r = E.rollout(track, act, dom, laps=laps)
    return dict(laps=r['laps'], contacts=int(r['hit'].sum()) if 'hit' in r
                else r['collisions'],
                sim_coll=r['collisions'],
                lap_med=float(np.median(r['lap_times']))
                if r['lap_times'] else float('nan'),
                min_clear=r.get('min_clear', float('nan')), frame=dom['frame'])


def fitted(line, mp, npz, frame, laps, seeds):
    cmd = [sys.executable, os.path.join(HERE, 'surrogate_race.py'),
           '--line', line, '--map', mp, '--frame', str(frame),
           '--laps', str(laps), '--seeds', str(seeds), '--quiet']
    if npz:
        cmd += ['-r', 'residual_policy=%s' % npz]
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    laps_d = coll = 0
    med = float('nan')
    where = ''
    for ln in p.stdout.splitlines():
        if ln.startswith('LAPS'):
            b = ln.split()
            laps_d, coll = int(b[1]), int(b[3])
            if 'at s' in ln:
                where = ln.split('at s', 1)[1].strip()[:60]
        if ln.startswith('lap time'):
            med = float(ln.split('median')[1].split('s')[0])
    if p.returncode:
        where = p.stderr[-200:]
    return laps_d, coll, med, where


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz')
    ap.add_argument('--line',
                    default='data/racelines/finals_mesh_exitm_m10t6_h3_race.csv')
    ap.add_argument('--map', default='data/maps/finals_mesh')
    ap.add_argument('--laps', type=int, default=100)
    ap.add_argument('--seeds', type=int, default=2)
    ap.add_argument('--pess-episodes', type=int, default=12)
    ap.add_argument('--pess-laps', type=int, default=8)
    ap.add_argument('--workers', type=int, default=6)
    a = ap.parse_args()
    line = a.line if os.path.isabs(a.line) else os.path.join(ROOT, a.line)
    mp = a.map if os.path.isabs(a.map) else os.path.join(ROOT, a.map)
    npz = os.path.abspath(a.npz)

    print('=== fitted surrogate (the validated one) ===')
    print('%-10s %-7s %6s %6s %8s  %s'
          % ('arm', 'period', 'laps', 'coll', 'lap_med', 'where'))
    base = {}
    for frame in (0.058, 0.097):
        for tag, p in (('shipping', ''), ('residual', npz)):
            l, c, m, w = fitted(line, mp, p, frame, a.laps, a.seeds)
            if tag == 'shipping':
                base[frame] = m
            d = '' if tag == 'shipping' else '  (%+0.3f)' % (m - base[frame])
            print('%-10s %-7.3f %6d %6d %8.3f%s  %s' % (tag, frame, l, c, m, d, w))

    print()
    print('=== pessimistic, domain-randomised plant (what it trained on) ===')
    print('%-10s %6s %6s %9s %8s %10s'
          % ('arm', 'eps', 'laps', 'contacts', 'lap_med', 'min_clear'))
    with ProcessPoolExecutor(max_workers=a.workers, initializer=_init,
                             initargs=(line, mp)) as pool:
        for tag, p in (('shipping', ''), ('residual', npz)):
            jobs = [(p, 1000 + i, a.pess_laps, None)
                    for i in range(a.pess_episodes)]
            res = list(pool.map(_pess, jobs))
            print('%-10s %6d %6d %9d %8.3f %10.3f'
                  % (tag, len(res), sum(r['laps'] for r in res),
                     sum(r['contacts'] for r in res),
                     float(np.nanmedian([r['lap_med'] for r in res])),
                     float(np.nanmin([r['min_clear'] for r in res]))))


if __name__ == '__main__':
    main()
