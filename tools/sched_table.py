"""The before/after table for a candidate s-indexed schedule.

Runs each named arm at both sensor periods with the same seeds and prints one
markdown row per (arm, frame): laps / collisions, median lap, the three corner
passage times, min footprint clearance, the s 19 entry cut and the s 36-39
clearance.  This is the gate table -- a non-zero collision count at EITHER
period disqualifies an arm.

    python tools/sched_table.py --seeds 0,1,2,3,4,5,6,7 --laps 12 \
        base=data/racelines/finals_mesh_sched.json \
        bo=data/racelines/finals_mesh_sched_bo.json
"""

import os

for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')

import argparse                                                  # noqa: E402
import json                                                      # noqa: E402
import multiprocessing as mp                                     # noqa: E402
import sys                                                       # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def _cell(job):
    import sched_eval
    name, frame, seeds, laps, path, line = job
    r = sched_eval.run_cell(frame, seeds, laps, sched=path, line=line)
    r['arm'] = name
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('arms', nargs='+', help='name=path/to/sched.json '
                                            '(an empty path = no schedule)')
    ap.add_argument('--seeds', default='0,1,2,3,4,5,6,7')
    ap.add_argument('--laps', type=int, default=12)
    ap.add_argument('--line',
                    default=os.path.join(os.path.dirname(HERE),
                                         'data/racelines/'
                                         'finals_mesh_hyb675b_race.csv'))
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    seeds = [int(t) for t in a.seeds.split(',')]

    jobs = []
    for spec in a.arms:
        name, _, path = spec.partition('=')
        for f in (0.098, 0.058):
            jobs.append((name, f, seeds, a.laps, path, a.line))
    with mp.Pool(2) as pool:
        res = pool.map(_cell, jobs)

    print('| arm | frame | laps / coll | lap median | s 18-21 | s 36-39 | '
          's 42-44 | min clear | cut p50 / p95 | s 36-39 clear |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    for r in res:
        print(f"| {r['arm']} | {r['frame']:.3f} | {r['laps']} / "
              f"**{r['collisions']}** | {r['lap_med']:.3f} | "
              f"{r['seg_hp19']:.3f} | {r['seg_c37']:.3f} | {r['seg_hp43']:.3f} | "
              f"{r['clear_min']:.3f} | {r['cut_p50']:.3f} / {r['cut_p95']:.3f} | "
              f"{r['c37_clear_min']:.3f} |")
    for r in res:
        if r['coll_s']:
            print(f"  {r['arm']} @ {r['frame']}: collisions at s {r['coll_s']}")
    if a.json:
        with open(a.json, 'w') as fh:
            json.dump(res, fh, indent=1)


if __name__ == '__main__':
    main()
