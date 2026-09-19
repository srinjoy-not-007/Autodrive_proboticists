"""One evaluation cell of the s-indexed schedule, for the BO driver.

Runs `tools/sim_tracker.py`'s closed loop in-process (same plant flags as the
frozen harness line in docs/FINALS_TUNING.md section 5.4), pools N seeds x
`--laps` laps at one sensor period, and prints ONE json line of the metrics the
schedule is gated on: laps, collisions, median lap time, min footprint
clearance, the three corner passage times, the s 19 entry cut and the s 36-39
clearance.

Usage (one cell):

    python tools/sched_eval.py --frame 0.098 --seeds 0,1,2 --laps 12 \
        --sched data/racelines/finals_mesh_sched.json
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, HERE)

import sim_tracker as st                                        # noqa: E402
from pit_crew.mpc2_core import RefPath                          # noqa: E402

LINE = os.path.join(ROOT, 'data/racelines/finals_mesh_hyb675b_race.csv')
MAP = '/home/ankit/code/data/simracing/data/maps/finals_mesh'

# The frozen finals args (docs/FINALS_TUNING.md section 5) plus `pub_gate_corr`.
# The weave notch (`weave_notch_k/q`, `straight_d`, `straight_kappa`) is NOT
# here: it is suspected of the rare straight-line collisions seen live at
# 0.097 s (5/100 and 8/100 laps in the two 100-lap runs that carried it, 0/100
# in the two that did not), so the schedule is tuned on the notch-free config.
BASE = dict(rate_adaptive=1, w_dcmd=50.0, cmd_tau=0.10, steer_tau=0.04,
            barrier_margin=0.40, thr_brake_lead_s=0.05, brake_mode='zero',
            brake_a_min=1.5, brake_dv=0.3, brake_a_full=4.6,
            pub_gate_corr=0.05)

SEGMENTS = dict(hp19=(18.0, 21.0), c37=(36.0, 39.0), hp43=(42.0, 44.0))
CUT_WIN = (17.5, 19.5)
C37_WIN = (36.0, 39.0)


def _seg_time(rec, s0, s1):
    """Median per-lap time spent inside [s0, s1) of arc length."""
    t, s, lap = rec[:, 0], rec[:, 6], rec[:, 9]
    out = []
    for L in np.unique(lap[lap >= 1]):
        m = (lap == L) & (s >= s0) & (s < s1)
        if m.sum() > 3:
            out.append(t[m].max() - t[m].min())
    return float(np.median(out)) if out else float('nan')


def run_cell(frame, seeds, laps, sched='', extra=None, line=LINE, mapp=MAP):
    from pit_crew import pf_core
    dist, _img, meta = pf_core.load_map(mapp)
    kw = dict(BASE)
    if extra:
        kw.update(extra)
    if sched:
        kw['param_schedule'] = sched

    laps_tot = 0
    coll = 0
    coll_s = []
    lap_times = []
    clear_min = []
    segs = {k: [] for k in SEGMENTS}
    cut = []
    c37_clear = []
    for sd in seeds:
        path = RefPath(line, speed_scale=1.0, kappa_span_m=0.5)
        path.load_corridor(dist, meta)
        tr = st.MPC3Sim(path, **kw)
        out = st.run(tr, path, laps=laps, dist=dist, meta=meta,
                     pose_noise=0.03, pose_latency=0.044, frame=frame,
                     enc_alpha=0.35, seed=sd, enc_slip=st.ENC_SLIP,
                     log_enc_alpha=st.LOG_ENC_ALPHA,
                     servo=dict(st.PLANT_REAL, brake_at_zero=4.6),
                     actuator=dict(actuator='slew', steer_rate=None,
                                   tau=0.08, dead=0.045, tick_frac=1.0))
        rec = out['rec']
        laps_tot += out['laps']
        coll += out['collisions']
        coll_s += [round(float(x), 1) for x in out['coll_s']]
        lap_times += list(out['lap_times'][1:])
        r = rec[rec[:, 9] >= 1] if (rec[:, 9] >= 1).sum() > 50 else rec
        s, e, cl = r[:, 6], r[:, 7], r[:, 8]
        clear_min.append(float(cl.min()) if len(cl) else float('nan'))
        for k, (a, b) in SEGMENTS.items():
            segs[k].append(_seg_time(rec, a, b))
        kap = path.kappa[np.searchsorted(path.s, s).clip(0, path.n - 1)]
        m = (s >= CUT_WIN[0]) & (s < CUT_WIN[1])
        if m.sum():
            # inside-ness of the line through the s 19 hairpin entry: positive
            # means the car is on the inside of the bend, i.e. cutting.
            cut.append(e[m] * np.sign(np.median(kap[m])))
        m = (s >= C37_WIN[0]) & (s < C37_WIN[1])
        if m.sum():
            c37_clear.append(cl[m])

    cut = np.concatenate(cut) if cut else np.array([np.nan])
    c37 = np.concatenate(c37_clear) if c37_clear else np.array([np.nan])
    return dict(
        frame=frame, seeds=list(seeds), laps=int(laps_tot), collisions=int(coll),
        coll_s=coll_s,
        lap_med=float(np.median(lap_times)) if lap_times else float('nan'),
        lap_best=float(min(lap_times)) if lap_times else float('nan'),
        clear_min=float(np.nanmin(clear_min)),
        seg_hp19=float(np.nanmedian(segs['hp19'])),
        seg_c37=float(np.nanmedian(segs['c37'])),
        seg_hp43=float(np.nanmedian(segs['hp43'])),
        cut_p50=float(np.percentile(cut, 50)),
        cut_p95=float(np.percentile(cut, 95)),
        c37_clear_min=float(np.min(c37)),
        c37_clear_p2=float(np.percentile(c37, 2)),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', type=float, default=0.098)
    ap.add_argument('--seeds', default='0,1,2')
    ap.add_argument('--laps', type=int, default=12)
    ap.add_argument('--sched', default='')
    ap.add_argument('--line', default=LINE)
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    seeds = [int(x) for x in a.seeds.split(',')]
    res = run_cell(a.frame, seeds, a.laps, sched=a.sched, line=a.line)
    res['tag'] = a.tag
    res['sched'] = a.sched
    print(json.dumps(res))


if __name__ == '__main__':
    main()
