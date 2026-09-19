#!/usr/bin/env python3
"""Validate the log-fitted surrogate against arms whose LIVE verdict is known.

Phase 1 of the RL track: an RL policy trained on a non-predictive simulator is
worthless, so before any policy is trained the surrogate has to reproduce
results it was not fitted to.  Two tests, both held out:

  (a) PACE -- the per-2 m segment times of the shipping config at both sensor
      periods, against the two 100-lap logs.  Reported by `surrogate_race.py
      --compare-log`; the table below repeats the summary.
  (b) VERDICT -- for each arm in `ARMS`, does the surrogate collide where the
      live car collided, and stay clean where it stayed clean, AT THE SAME
      SENSOR PERIOD?  Every live outcome here is from docs/FINALS_TUNING.md
      section 3 and none of it was used to fit the plant.

The verdict rule mirrors the live gate: an arm "collides" at a period if it
takes at least one collision per 100 laps there.  The confusion table at the
end is printed whether it flatters the surrogate or not.

    tools/surrogate_validate.py                 # the whole table
    tools/surrogate_validate.py --laps 100 --seeds 3
    tools/surrogate_validate.py --only h2,h7,notch

Offline only.  Pin it: taskset -c 16-27 nice -n 19 env OMP_NUM_THREADS=1 ...
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

PG = ['pub_gate_corr=0.05']
SERVO_UNGATED = ['servo_k_p=0.5', 'servo_head_max=0.3']
SERVO_GATED = SERVO_UNGATED + ['servo_head_kappa=0.4']
BASE = ['rate_adaptive=1', 'w_dcmd=50.0', 'cmd_tau=0.10', 'steer_tau=0.04',
        'barrier_margin=0.40', 'thr_brake_lead_s=0.05', 'brake_mode=zero',
        'brake_a_min=1.5', 'brake_dv=0.3', 'brake_a_full=4.6']
NOTCH = ['weave_notch_k=0.227', 'weave_notch_q=0.7', 'straight_d=3.0',
         'straight_kappa=0.10']

H3 = 'finals_mesh_exitm_m10t6_h3_race'
HYB = 'finals_mesh_hyb675b_race'

# name, line, extra mpc3 params, {period: live verdict}, live note.
# Verdict: 'clean' or 'collide'.  'slow' = clean live but slower than shipping.
ARMS = [
    ('ship',   H3, PG + SERVO_GATED,
     {0.058: 'clean', 0.097: 'clean'},
     '100/0 display 108.37 s, 100/0 Xvfb 114.19 s (race_20260919_044701/_050630)'),
    ('h2', 'finals_mesh_exitm_m10t6_h2_race', PG,
     {0.058: 'collide', 0.097: 'collide'},
     'display 32/1, Xvfb 32/9, every hit the last hairpin exit s 44.1-44.3'),
    ('h7', 'finals_mesh_exitm_m10t6_h7_race', PG + SERVO_GATED,
     {0.097: 'collide'}, 'Xvfb 27 laps / 10 collisions (s 36-40 released, full ay 9)'),
    ('h8', 'finals_mesh_exitm_m10t6_h8_race', PG + SERVO_GATED,
     {0.097: 'collide'}, 'Xvfb 32 laps / 4 collisions (s 36-40 released, half)'),
    ('h3c_servo', 'finals_mesh_exitm_m10t6_h3c_race', PG + SERVO_UNGATED,
     {0.058: 'clean', 0.097: 'collide'},
     'display 100/0, Xvfb 100/6 -- all 0.30-0.37 m wide on a corner exit'),
    ('servo_ungated', H3, PG + SERVO_UNGATED,
     {0.058: 'collide', 0.097: 'clean'},
     'Xvfb 100/0, display 100/1 at s 26.7 (the gate `servo_head_kappa` fixes it)'),
    ('ay_limit16', HYB, PG + ['ay_limit=16.0'],
     {0.058: 'slow', 0.097: 'slow'},
     'clean at both, but SLOWER: Xvfb 117.79 vs 117.11, display 112.61 vs 111.65'),
    ('lead_tk40', H3, PG + ['lead_mode=time', 'lead_tk=4.0'],
     {0.097: 'collide'}, 'Xvfb 23 laps / 10 collisions'),
    ('notch', HYB, PG + NOTCH,
     {0.097: 'collide'},
     'two 100-lap Xvfb runs took 5 and 8 STRAIGHT-LINE collisions; 64 clean laps at 32'),
    ('sched_bo', HYB, PG + ['param_schedule=data/racelines/finals_mesh_sched_bo.json'],
     {0.097: 'collide'}, 'Xvfb 15 laps / 10 collisions'),
]

SHIP_LOGS = {0.058: 'data/logs/race_20260919_044701.csv',
             0.097: 'data/logs/race_20260919_050630.csv'}


def one(job):
    name, line, params, frame, laps, seeds, plant = job
    cmd = [sys.executable, os.path.join(HERE, 'surrogate_race.py'),
           '--line', line, '--frame', str(frame), '--laps', str(laps),
           '--seeds', str(seeds), '--no-shipping', '--quiet']
    for chunk in plant:
        cmd += ['--plant', chunk]
    for p in BASE + params:
        cmd += ['-r', p]
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    if p.returncode != 0:
        return dict(name=name, frame=frame, err=p.stderr[-800:])
    laps_done = colls = 0
    where = ''
    med = float('nan')
    for ln in p.stdout.splitlines():
        if ln.startswith('LAPS'):
            bits = ln.split()
            laps_done, colls = int(bits[1]), int(bits[3])
            if 'at s' in ln:
                where = ln.split('at s', 1)[1].strip()
        if ln.startswith('lap time'):
            med = float(ln.split('median')[1].split('s')[0])
    return dict(name=name, frame=frame, laps=laps_done, colls=colls,
                where=where, med=med, err='')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--laps', type=int, default=100)
    ap.add_argument('--seeds', type=int, default=3)
    # HOST RULE (Sep 19): more than ~6 single-threaded workers on this box
    # drags the simulator's Xvfb sensor period from 0.092-0.098 s out to
    # 0.101-0.104, which pushes a scored live run outside the organisers'
    # band.  Six on cores 22-27 is the ceiling while a live run can happen.
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--only', default='')
    ap.add_argument('--plant', action='append', default=[],
                    help='extra sim_tracker plant flag, passed to every cell')
    ap.add_argument('--grip-c', type=float, default=0.0,
                    help='shorthand for --plant "--plant-grip-c X"')
    ap.add_argument('--grip-ay0', type=float, default=0.0)
    a = ap.parse_args()
    pick = set(x.strip() for x in a.only.split(',')) if a.only else None
    if a.grip_c > 0.0:
        a.plant.append('--plant-grip-c %g' % a.grip_c)
    if a.grip_ay0 > 0.0:
        a.plant.append('--plant-grip-ay0 %g' % a.grip_ay0)

    jobs, meta = [], {}
    for name, line, params, live, note in ARMS:
        if pick and name not in pick:
            continue
        meta[name] = (live, note)
        for frame in live:
            jobs.append((name, line, params, frame, a.laps, a.seeds, a.plant))

    res = {}
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        for r in ex.map(one, jobs):
            res[(r['name'], r['frame'])] = r

    rate = {}
    print('%-14s %6s | %5s %5s %7s | %-8s %-8s | %s'
          % ('arm', 'period', 'laps', 'coll', 'lap_med', 'surrogate', 'live', 'agree'))
    agree = {}
    for name, line, params, live, note in ARMS:
        if pick and name not in pick:
            continue
        for frame in sorted(live):
            r = res[(name, frame)]
            if r.get('err'):
                print('%-14s %6.3f | ERROR %s' % (name, frame, r['err'][:200]))
                continue
            per100 = 100.0 * r['colls'] / max(r['laps'], 1)
            sur = 'collide' if per100 >= 1.0 else 'clean'
            lv = live[frame]
            ok = (sur == 'clean') if lv in ('clean', 'slow') else (sur == 'collide')
            agree[(name, frame)] = (sur, lv, ok)
            rate[(name, frame)] = per100
            print('%-14s %6.3f | %5d %5d %7.3f | %-8s %-8s | %s%s'
                  % (name, frame, r['laps'], r['colls'], r['med'], sur, lv,
                     'YES' if ok else 'NO ',
                     ('   ' + r['where'][:60]) if r['where'] else ''))
    tp = sum(1 for s, l, o in agree.values() if l == 'collide' and s == 'collide')
    fn = sum(1 for s, l, o in agree.values() if l == 'collide' and s == 'clean')
    tn = sum(1 for s, l, o in agree.values() if l != 'collide' and s == 'clean')
    fp = sum(1 for s, l, o in agree.values() if l != 'collide' and s == 'collide')
    print()
    print('confusion (cells = arm x sensor period)')
    print('                 live collide   live clean')
    print('  surr collide   %8d     %8d' % (tp, fp))
    print('  surr clean     %8d     %8d' % (fn, tn))
    print('  agreement %d / %d' % (tp + tn, len(agree)))
    print()
    for name, line, params, live, note in ARMS:
        if pick and name not in pick:
            continue
        print('  %-14s %s' % (name, note))


if __name__ == '__main__':
    main()
