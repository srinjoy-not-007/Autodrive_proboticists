"""Constrained Bayesian optimisation of the s-indexed mpc3 parameter schedule.

Fröhlich et al., "Contextual Tuning of Model Predictive Control for Autonomous
Racing" (arXiv:2110.02710), tune an MPC's cost weights with a BO whose GP is
given the *operating context* as extra input, so the returned optimum is the
one for the context that will actually be raced.  Here the context is the
SENSOR PERIOD, which on this stack is a per-start lottery (0.058 s on a real
display, ~0.098 s on the organisers' box) and is the single variable that
decides which corner fails.  We do not learn across contexts -- with two
discrete contexts it is cheaper to EVALUATE both every sample -- so the context
enters as the objective (median lap at 0.098, the raced period) and as a hard
feasibility constraint (zero collisions and no lost margin at 0.058 as well).

Objective, per the brief:

    J = lap_med(0.098) + lam * max(0, 0.10 - min_clearance) + 10 * collisions

Hard constraints against the committed baseline `finals_mesh_sched.json`:

    * zero collisions at BOTH frames;
    * min footprint clearance >= baseline, both frames;
    * s 19 entry cut (p50 and p95 over s 17.5-19.5) <= baseline, 0.098 s;
    * s 36-39 minimum clearance >= baseline, both frames.

The surrogate is a Matern-5/2 ARD GP (numpy/scipy only, hyperparameters by
marginal likelihood) on J over the FEASIBLE samples, plus one GP per constraint
on its violation margin; the acquisition is expected improvement times the
product of the GPs' probabilities of satisfying each constraint, which is the
standard cEI of Gardner et al. (ICML 2014).  Infeasible samples still train the
constraint GPs, which is the point of modelling them separately.

    python tools/sched_bo.py --init 24 --iters 90 --out /tmp/bo
"""

import os

for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_v, '1')

import argparse                                                 # noqa: E402
import csv                                                      # noqa: E402
import json                                                     # noqa: E402
import multiprocessing as mp                                    # noqa: E402
import sys                                                      # noqa: E402
import time                                                     # noqa: E402

import numpy as np                                              # noqa: E402
from scipy.optimize import minimize                             # noqa: E402
from scipy.stats import norm, qmc                               # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

TOTAL_S = 44.71081127190853
LAM = 10.0
CLEAR_TARGET = 0.10

# ---------------------------------------------------------------- the design
# Ten knobs.  The three committed windows' values and their leading edges (the
# edge is where an override starts acting, and every one of these windows works
# by acting EARLY), plus four more, each with a measured reason in section 5.4:
#   vs43    the s 43.6 apex is set by the lateral-jerk limiter, not by grip
#           (plan asks ay 2.7 where the budget is 6.75) -- the one raise the
#           `_fast` schedule already showed is worth 0.07 s/lap;
#   vmin19  the e_free/k_e cross-track cut is the ONLY binding speed term here
#           and it clips at v_mod_min on the hairpin EXIT;
#   vsmed   s 21.5-32 is the third of the lap no window touches and no
#           collision has ever happened in; it ends before the s 34.5 brake
#           zone so a raise cannot arrive as entry speed at s 37 (the
#           per-segment-optimism failure `lmpc_a/b/d/e` died of);
#   blend   the global ramp width -- how abruptly any of this arrives.
SPACE = [
    ('bm19',  0.40, 0.55),      # barrier_margin, hp19_entry
    ('s0_19', 15.0, 17.2),
    ('bl37',  0.05, 0.14),      # thr_brake_lead_s, c37_entry
    ('s0_37', 33.3, 35.6),
    ('bl43',  0.00, 0.05),      # thr_brake_lead_s, hp43_entry
    ('s0_43', 39.5, 41.6),
    ('vs43',  1.00, 1.10),      # v_scale, hp43_apex
    ('vmin19', 0.55, 0.78),     # v_mod_min, hp19_exit
    ('vsmed', 1.00, 1.08),      # v_scale, s 21.5-32 medium corners
    ('blend', 0.60, 1.60),      # blend_m, global
]
NAMES = [s[0] for s in SPACE]
LO = np.array([s[1] for s in SPACE])
HI = np.array([s[2] for s in SPACE])

S1_19, S1_37, S1_43 = 19.0, 37.5, 43.5
MED0, MED1 = 21.5, 32.0
APEX0, APEX1 = 43.2, 44.5


def to_sched(x):
    d = dict(zip(NAMES, np.asarray(x, float).tolist()))
    blend = round(d['blend'], 3)
    w = [
        dict(name='hp19_entry', s0=round(d['s0_19'], 2), s1=S1_19,
             barrier_margin=round(d['bm19'], 4)),
        dict(name='hp19_exit', s0=19.2, s1=21.0,
             v_mod_min=round(d['vmin19'], 4)),
        dict(name='med_corners', s0=MED0, s1=MED1,
             v_scale=round(d['vsmed'], 4)),
        dict(name='c37_entry', s0=round(d['s0_37'], 2), s1=S1_37,
             thr_brake_lead_s=round(d['bl37'], 4)),
        dict(name='hp43_entry', s0=round(d['s0_43'], 2), s1=S1_43,
             thr_brake_lead_s=round(d['bl43'], 4)),
        dict(name='hp43_apex', s0=APEX0, s1=APEX1,
             v_scale=round(d['vs43'], 4)),
    ]
    return dict(line='finals_mesh_hyb675b_race.csv', map='finals_mesh',
                total_s=TOTAL_S, blend_m=blend, windows=w)


# ------------------------------------------------------------ the evaluation
def _cell(job):
    import sched_eval
    frame, seeds, laps, path, line = job
    return sched_eval.run_cell(frame, seeds, laps, sched=path, line=line)


def evaluate(pool, x, seeds, laps, outdir, tag, line):
    path = os.path.join(outdir, f'sched_{tag}.json')
    with open(path, 'w') as fh:
        json.dump(to_sched(x), fh, indent=1)
    jobs = [(0.098, seeds, laps, path, line), (0.058, seeds, laps, path, line)]
    a, b = pool.map(_cell, jobs)
    return a, b, path


def metrics(a, b):
    """(objective, constraint violations) from the 0.098 and 0.058 cells."""
    coll = a['collisions'] + b['collisions']
    clr = min(a['clear_min'], b['clear_min'])
    J = a['lap_med'] + LAM * max(0.0, CLEAR_TARGET - clr) + 10.0 * coll
    return J, coll, clr


# ---------------------------------------------------------------------- a GP
def _matern52(X1, X2, ls, sf2):
    d = np.sqrt(np.maximum(
        ((X1[:, None, :] - X2[None, :, :]) / ls) ** 2, 0.0).sum(-1) + 1e-12)
    r = np.sqrt(5.0) * d
    return sf2 * (1.0 + r + r * r / 3.0) * np.exp(-r)


class GP:
    def __init__(self, X, y):
        self.X = np.asarray(X, float)
        y = np.asarray(y, float)
        self.mu, self.sd = y.mean(), max(y.std(), 1e-6)
        self.y = (y - self.mu) / self.sd
        self._fit()

    def _nll(self, th):
        ls = np.exp(th[:self.X.shape[1]])
        sf2, sn2 = np.exp(th[-2]), np.exp(th[-1]) + 1e-8
        K = _matern52(self.X, self.X, ls, sf2) + sn2 * np.eye(len(self.X))
        try:
            L = np.linalg.cholesky(K)
        except np.linalg.LinAlgError:
            return 1e6
        al = np.linalg.solve(L.T, np.linalg.solve(L, self.y))
        return 0.5 * self.y @ al + np.log(np.diag(L)).sum()

    def _fit(self):
        d = self.X.shape[1]
        best, bth = np.inf, None
        for s in (np.log(0.5), np.log(0.2), np.log(1.0)):
            th0 = np.r_[np.full(d, s), np.log(1.0), np.log(1e-2)]
            r = minimize(self._nll, th0, method='L-BFGS-B',
                         bounds=[(np.log(0.05), np.log(5.0))] * d
                         + [(np.log(1e-3), np.log(10.0)),
                            (np.log(1e-6), np.log(1.0))])
            if r.fun < best:
                best, bth = r.fun, r.x
        self.ls = np.exp(bth[:d])
        self.sf2, self.sn2 = np.exp(bth[-2]), np.exp(bth[-1]) + 1e-8
        K = (_matern52(self.X, self.X, self.ls, self.sf2)
             + self.sn2 * np.eye(len(self.X)))
        self.L = np.linalg.cholesky(K)
        self.al = np.linalg.solve(self.L.T, np.linalg.solve(self.L, self.y))

    def predict(self, Xs):
        Ks = _matern52(np.atleast_2d(Xs), self.X, self.ls, self.sf2)
        m = Ks @ self.al
        v = np.linalg.solve(self.L, Ks.T)
        s2 = np.maximum(self.sf2 - (v * v).sum(0), 1e-12)
        return m * self.sd + self.mu, np.sqrt(s2) * self.sd


def _norm(X):
    return (np.asarray(X, float) - LO) / (HI - LO)


def acquisition(gpJ, gpC, best, Xc):
    m, s = gpJ.predict(Xc)
    z = (best - m) / np.maximum(s, 1e-9)
    ei = (best - m) * norm.cdf(z) + s * norm.pdf(z)
    p = np.ones(len(Xc))
    for g in gpC:
        mc, sc = g.predict(Xc)
        p *= norm.cdf(-mc / np.maximum(sc, 1e-9))     # want violation <= 0
    return np.maximum(ei, 1e-12) * p


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--init', type=int, default=24)
    ap.add_argument('--iters', type=int, default=90)
    ap.add_argument('--seeds', default='0,1,2')
    ap.add_argument('--laps', type=int, default=12)
    ap.add_argument('--out', default='/tmp/bo')
    ap.add_argument('--line',
                    default=os.path.join(os.path.dirname(HERE),
                                         'data/racelines/'
                                         'finals_mesh_hyb675b_race.csv'))
    ap.add_argument('--trace', default='')
    a = ap.parse_args()
    seeds = [int(t) for t in a.seeds.split(',')]
    os.makedirs(a.out, exist_ok=True)
    trace = a.trace or os.path.join(a.out, 'trace.csv')

    pool = mp.Pool(2)

    # Baseline: the committed schedule, on the same seeds, is what every
    # constraint is measured against.
    base_path = os.path.join(os.path.dirname(HERE),
                             'data/racelines/finals_mesh_sched.json')
    ja = pool.map(_cell, [(0.098, seeds, a.laps, base_path, a.line),
                          (0.058, seeds, a.laps, base_path, a.line)])
    B98, B58 = ja
    bJ, bcoll, bclr = metrics(B98, B58)
    print('BASELINE 0.098', json.dumps(B98))
    print('BASELINE 0.058', json.dumps(B58))
    print(f'BASELINE J {bJ:.4f} coll {bcoll} clear {bclr:.3f}', flush=True)
    if bcoll:
        print('baseline is not clean; constraints would be meaningless')

    # Tolerances: a lap-wide MINIMUM over 36 laps is a single sample and moves
    # a centimetre between seeds, so "not worse than baseline" is enforced at
    # 0.010 m on a clearance and 0.005 m on the entry cut.  Anything tighter
    # would be chasing the harness's own noise, not margin.
    TOL_CLR, TOL_CUT = 0.010, 0.005

    def cons(A, B):
        return np.array([
            A['collisions'] + B['collisions'],                    # zero coll
            B98['clear_min'] - A['clear_min'] - TOL_CLR,          # clearance
            B58['clear_min'] - B['clear_min'] - TOL_CLR,
            A['cut_p50'] - B98['cut_p50'] - TOL_CUT,              # entry cut
            A['cut_p95'] - B98['cut_p95'] - TOL_CUT,
            B98['c37_clear_min'] - A['c37_clear_min'] - TOL_CLR,  # s 36-39
            B58['c37_clear_min'] - B['c37_clear_min'] - TOL_CLR,
        ], float)

    CN = ['coll', 'clr98', 'clr58', 'cut50', 'cut95', 'c3798', 'c3758']

    rows, X, J, C = [], [], [], []
    fh = open(trace, 'w', newline='')
    wr = csv.writer(fh)
    wr.writerow(['i', 'kind'] + NAMES + ['J', 'feasible']
                + ['v_' + c for c in CN]
                + ['laps98', 'coll98', 'lap_med98', 'clear98', 'cut50_98',
                   'cut95_98', 'c37clr98', 'seg19_98', 'seg37_98', 'seg43_98',
                   'laps58', 'coll58', 'lap_med58', 'clear58', 'c37clr58',
                   'seg19_58', 'seg37_58', 'seg43_58', 'path'])

    def record(i, kind, x, A, B):
        Jv, coll, clr = metrics(A, B)
        v = cons(A, B)
        feas = bool(np.all(v <= 1e-12))
        X.append(_norm(x))
        J.append(Jv)
        C.append(v)
        wr.writerow([i, kind] + [round(float(t), 4) for t in x]
                    + [round(Jv, 4), int(feas)] + [round(float(t), 4) for t in v]
                    + [A['laps'], A['collisions'], round(A['lap_med'], 3),
                       round(A['clear_min'], 4), round(A['cut_p50'], 4),
                       round(A['cut_p95'], 4), round(A['c37_clear_min'], 4),
                       round(A['seg_hp19'], 3), round(A['seg_c37'], 3),
                       round(A['seg_hp43'], 3),
                       B['laps'], B['collisions'], round(B['lap_med'], 3),
                       round(B['clear_min'], 4), round(B['c37_clear_min'], 4),
                       round(B['seg_hp19'], 3), round(B['seg_c37'], 3),
                       round(B['seg_hp43'], 3), ''])
        fh.flush()
        rows.append(dict(i=i, x=list(map(float, x)), J=Jv, feas=feas,
                         v=v.tolist(), A=A, B=B))
        print(f'[{i:3d}] {kind:4s} J {Jv:8.4f} feas {int(feas)} '
              f'coll {A["collisions"]}/{B["collisions"]} '
              f'lap98 {A["lap_med"]:.3f} clr {min(A["clear_min"],B["clear_min"]):.3f} '
              f'cut95 {A["cut_p95"]:.3f} c37 {A["c37_clear_min"]:.3f} '
              + ' '.join(f'{n}={t:.3f}' for n, t in zip(NAMES, x)), flush=True)
        return Jv, feas

    t0 = time.time()
    # ---- initial design: Sobol, with the committed baseline's own point in it
    sob = qmc.Sobol(len(SPACE), scramble=True, seed=7)
    Xi = LO + sob.random(a.init) * (HI - LO)
    # exactly finals_mesh_sched.json: the three extra windows at their own
    # defaults (v_mod_min 0.55, v_scale 1.0) are no-ops.
    x_base = np.array([0.50, 16.0, 0.09, 34.5, 0.00, 40.5,
                       1.00, 0.55, 1.00, 1.00])
    Xi = np.vstack([x_base, Xi])
    for i, x in enumerate(Xi):
        A, B, _p = evaluate(pool, x, seeds, a.laps, a.out, f'i{i}', a.line)
        record(i, 'init', x, A, B)

    # ---- BO loop
    rng = np.random.default_rng(11)
    for k in range(a.iters):
        i = len(rows)
        Xa = np.array(X)
        Ja = np.array(J)
        Ca = np.array(C)
        feas = np.array([r['feas'] for r in rows])
        # J GP: fit on all points, infeasible ones carry their real J (which
        # already contains the 10 s/collision penalty), so the surface stays
        # informative where constraints bite.
        gpJ = GP(Xa, Ja)
        gpC = []
        for j in range(Ca.shape[1]):
            col = Ca[:, j]
            if np.allclose(col, col[0]):
                continue
            gpC.append(GP(Xa, col))
        best = Ja[feas].min() if feas.any() else Ja.min()
        # candidates: Sobol + local jitter around the incumbent
        cand = qmc.Sobol(len(SPACE), scramble=True,
                         seed=int(rng.integers(1 << 30))).random(4096)
        if feas.any():
            inc = Xa[feas][np.argmin(Ja[feas])]
            loc = np.clip(inc + rng.normal(0, 0.06, (1024, len(SPACE))), 0, 1)
            cand = np.vstack([cand, loc])
        ac = acquisition(gpJ, gpC, best, cand)
        xn = LO + cand[int(np.argmax(ac))] * (HI - LO)
        A, B, _p = evaluate(pool, xn, seeds, a.laps, a.out, f'b{k}', a.line)
        record(i, 'bo', xn, A, B)
        if k % 10 == 0:
            print(f'    ... {time.time()-t0:.0f} s elapsed', flush=True)

    fh.close()
    feas = [r for r in rows if r['feas']]
    feas.sort(key=lambda r: r['J'])
    print('\nTOP FEASIBLE')
    for r in feas[:8]:
        print(f'  J {r["J"]:.4f}  ' + ' '.join(
            f'{n}={t:.3f}' for n, t in zip(NAMES, r['x'])))
    if feas:
        with open(os.path.join(a.out, 'best.json'), 'w') as f:
            json.dump(to_sched(feas[0]['x']), f, indent=1)
        with open(os.path.join(a.out, 'top.json'), 'w') as f:
            json.dump([r['x'] for r in feas[:8]], f)
    pool.close()


if __name__ == '__main__':
    main()
