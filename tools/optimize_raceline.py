#!/usr/bin/env python3
"""Minimum-curvature raceline + velocity profile (TUM tph pipeline).

Input:  centerline CSV (x_m,y_m,w_tr_right_m,w_tr_left_m), closed track.
Output: raceline CSV (s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2) + QC plot.

Vehicle limits are conservative defaults until sim calibration (task #3);
tune via CLI flags.

Usage:
  .venv/bin/python tools/optimize_raceline.py data/racelines/track_v1_centerline.csv \
      [--out data/racelines/track_v1_raceline] [--vmax 8] [--ay 3] [--ax 3]

Note: the corridor mask behind the widths is already wall-dilated (~7.5 cm
safety per side), so w_veh only needs the physical car width + small margin.
"""

# Pin BLAS before numpy loads: unpinned, this tool takes every core (63 threads
# on the finals host) and disturbed the simulator's frame timing during scored
# runs on Sep 19.  setdefault, so a caller's own pins win.  Output is identical.
import os as _os
for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    _os.environ.setdefault(_k, '1')

import argparse
import math
import sys

import numpy as np
import trajectory_planning_helpers as tph
from scipy import ndimage

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# One source of truth for anything measured off the vehicle. This module used
# to carry an unsourced m_veh=3.906 while compare_sim_builds.py had read 3.47 kg
# straight off the Unity rigidbody -- the duplication is what let the two drift
# apart, so take it from the same file the car does.
sys.path.insert(0, 'src/pit_crew')
from vehicle import MASS as VEHICLE_MASS   # noqa: E402

# Body box relative to the rear axle (the pose point), from the tech guide:
# length 0.500 m, width 0.270 m.
REAR_OVERHANG = 0.088
FRONT = 0.500 - REAR_OVERHANG
HALF_W = 0.270 / 2.0


def corner_sweep(kappa):
    """Extra width the body sweeps beyond the rear-axle path, per point.

    The pose point is the rear axle, but the car is 0.5 m long: on a curve of
    radius R the outer front corner rides on a larger radius, so a path whose
    centre clears the wall can still put a corner through it. Measured on real
    runs this costs up to 0.45 m, which is why a raceline planned with only the
    vehicle width collided while staying nominally clear.
    """
    R = 1.0 / np.maximum(np.abs(kappa), 1e-3)
    swing = np.sqrt((R + HALF_W) ** 2 + FRONT ** 2) - R
    return np.clip(swing, HALF_W, 0.60)


def clearance_speed_cap(xy, kappa, map_prefix, per_metre):
    """Cap speed where the track is narrow, not just where it is curved.

    The velocity profile has only ever known curvature. On Porto that puts the
    highest planned speed (6.9 m/s) on the start straight, which is also the
    tightest part of the line (0.325 m of corner clearance, the minimum for the
    whole lap). Measured consequence at ax 6.5: four collisions, all of them
    between s = 29.2 and s = 2.2 m, two on a section with kappa = -0.00 and the
    car only 0.054 m off its own line. Nothing was wrong with the tracking; the
    plan asked for 6 m/s through the narrowest gap on the track.

    `per_metre` is a speed allowed per metre of clearance, calibrated from a run
    that is known clean rather than chosen: the shipping profile peaks at
    5.39 m/s where clearance is 0.325 m, i.e. 16.6 m/s per metre. Anything
    faster than that ratio is asking for margin no run has ever demonstrated.
    """
    sys.path.insert(0, 'tools')
    from footprint_analysis import load_dist, clearance as clear_at, CORNERS

    dist, res, ox, oy = load_dist(map_prefix)
    d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    head = np.arctan2(d[:, 1], d[:, 0])
    worst = np.full(len(xy), np.inf)
    for cx, cy in CORNERS:
        px = xy[:, 0] + cx * np.cos(head) - cy * np.sin(head)
        py = xy[:, 1] + cx * np.sin(head) + cy * np.cos(head)
        worst = np.minimum(worst, np.nan_to_num(
            clear_at(dist, res, ox, oy, np.column_stack([px, py])), nan=np.inf))
    return np.maximum(worst, 0.0) * per_metre, worst


def turn_in_kappa(kappa, el_lengths, window):
    """Curvature the ay cap should be sized for, looking `window` metres ahead.

    The velocity profile caps speed point-by-point on the curvature AT that
    point, which implicitly assumes the car can be at the cornering attitude
    the instant the corner starts. It cannot: it has to build steering angle
    and let the tyres build slip, and until it has, the plan is asking for a
    curvature the car is not yet producing.

    Measured on Porto (Aug 29): every collision in two ~70-lap runs -- 1 in 78
    with lateral jerk limiting, 3 in 69 without it -- happened at the SAME
    corner, s ~ 16.2, where kappa ramps 0.04 -> 0.38 in half a metre. Lap-wide
    cross-track p95 there was 0.18 m; through that entry it was 0.31 m with a
    0.59 m worst case. The car was running wide on turn-in, not sliding at the
    apex.

    Replacing kappa with its forward-window maximum makes the cap bind `window`
    metres early, which is a corner-entry brake that costs nothing on a
    straight, nothing on corner exit, and nothing in a corner of constant
    radius. It only ever reduces speed.
    """
    if window <= 0.0:
        return kappa
    ds = float(np.median(el_lengths))
    n = max(1, int(round(window / max(ds, 1e-6))))
    mag = np.abs(kappa)
    ahead = np.max([np.roll(mag, -k) for k in range(n + 1)], axis=0)
    # Keep the sign of the point itself: only the magnitude drives the ay cap,
    # and the sign is what the jerk limiter reads to spot a real reversal.
    return np.where(mag > 0, ahead * np.sign(kappa), ahead)


def limit_lat_jerk(vx, kappa, el_lengths, j_max, ax_max, ax_brake,
                   rounds=40, v_floor=0.5):
    """Cap how fast lateral acceleration is allowed to change along the lap.

    The tyre model peaks at mu 1.0 but only at 0.01 sideways slip, falling to
    0.5 by 0.10 slip (see docs/STATUS.md, "Step 1 DONE"). A profile that jumps
    from one steady cornering load to another forces the tyre to build slip
    faster than it can hold, so it overshoots the peak and lands on the
    asymptote at half the grip. That is the measured failure signature: raising
    the ay cap to 4.0 collided at curvature *transitions*, not at the tightest
    corners, with steering at full lock 20% of samples.

    An ay cap alone cannot express this — it bounds the value, not the rate.
    This bounds d(ay)/dt by slowing the car through transitions, leaving steady
    corners free to use the higher cap.

    Only ever reduces speed, so it converges; the longitudinal passes afterwards
    keep the result reachable by a car that has to brake into the slow points.
    """
    v = np.asarray(vx, dtype=float).copy()
    ds = np.asarray(el_lengths, dtype=float)
    kap = np.abs(kappa)
    sgn = np.sign(kappa)

    for _ in range(rounds):
        ay = v * v * kappa
        dt = ds / np.maximum(0.5 * (v + np.roll(v, -1)), 0.1)
        lim = j_max * dt
        nxt = np.roll(ay, -1)
        over = np.abs(nxt - ay) > lim + 1e-6
        if not over.any():
            break
        for i in np.flatnonzero(over):
            j = (i + 1) % len(v)
            big, small = (i, j) if abs(ay[i]) > abs(ay[j]) else (j, i)
            if kap[big] < 1e-6:
                continue                     # straight: ay is ~0 regardless of v
            same = sgn[big] == sgn[small]
            if same:
                allowed = lim[i] + abs(ay[small])
            else:
                # Opposite signs: |ay_big - ay_small| = |ay_big| + |ay_small|,
                # so |ay_big| <= lim - |ay_small|. The algebra is right and the
                # remedy used to be wrong. When the SMALLER side alone already
                # exceeds the budget the constraint is infeasible with it held
                # fixed, and the old code drove `big` to zero -- clamping it to
                # the 0.5 m/s floor -- instead of reducing both. Measured cost:
                # 0.24 s of ideal lap at ay 5.0 and 1.5 s at ay 9.0, where it
                # was pinning the plan to 0.5 m/s on nearly straight sections.
                # Splitting the budget is the feasible answer and, like every
                # other branch here, only ever reduces speed.
                allowed = lim[i] - abs(ay[small])
                if allowed < 0.0:
                    half = 0.5 * lim[i]
                    if kap[small] >= 1e-6:
                        t_s = min(abs(ay[small]), half)
                        v[small] = max(v_floor, math.sqrt(t_s / kap[small]))
                        ay[small] = v[small] * v[small] * kappa[small]
                    allowed = half
            target = min(abs(ay[big]), max(0.0, allowed))
            v[big] = max(v_floor, math.sqrt(target / kap[big]))
            ay[big] = v[big] * v[big] * kappa[big]
        v = limit_long_accel(v, ds, ax_max, ax_brake)
    return v


def limit_long_accel(v, ds, ax_max, ax_brake, passes=3):
    """Forward/backward passes so the profile is reachable within ax limits."""
    v = v.copy()
    n = len(v)
    for _ in range(passes):
        for i in range(n):
            j = (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2.0 * ax_max * ds[i]))
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2.0 * ax_brake * ds[i]))
    return v


def curvature_of(reftrack, normvec, alpha, stepsize):
    """Curvature of the candidate line, sampled back at reference points."""
    rl, _, cx, cy, inds, tv, _, _, _ = tph.create_raceline.create_raceline(
        refline=reftrack[:, :2], normvectors=normvec, alpha=alpha,
        stepsize_interp=stepsize)
    _, kap = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=cx, coeffs_y=cy, ind_spls=inds, t_spls=tv)
    kap = ndimage.median_filter(kap, size=5, mode='wrap')
    idx = np.linspace(0, len(kap) - 1, len(reftrack)).astype(int)
    return kap[idx]




def window_steps(ds, w_min, w_max):
    """Sample offsets whose arc length spans [w_min, w_max] metres.

    The curvature rate is bounded over a BAND of window lengths rather than at
    the sample spacing, because at the sample spacing it is not a property of
    the line at all. Measured on Porto: the finished 0.10 m line's raw
    consecutive-sample rate swings by 1.0 1/m2 when the lateral shift moves
    0.6 mm -- the analytic curvature of a spline through 0.14 m reference
    points wobbles at the knot scale and the size-5 median filter turns the
    wobble into a staircase, so the "max d(kappa)/ds" of a filtered line is
    mostly an artefact of the filter. Over 0.2-0.5 m it is the real ramp: on
    the shipping line 1.16 1/m2 at 0.2 m and 0.68 at 0.5 m, both at the hairpin
    (s 10.2-13.3), which is the corner that produces every collision.

    The upper end is the physical one: the steering command is held ~0.09-0.10 s
    and the car covers ~0.45 m in that time at hairpin-entry speed, so curvature
    the line asks for inside half a metre is curvature the actuator has to
    deliver inside one command.
    """
    n1 = max(1, int(round(w_min / max(ds, 1e-6))))
    n2 = max(n1, int(round(w_max / max(ds, 1e-6))))
    return list(range(n1, n2 + 1))


def rate_metric(kap, ds, steps):
    """Worst |d(kappa)/ds| over the window band, and which window/sample."""
    worst, where = 0.0, (steps[0], 0)
    for n in steps:
        d = np.abs(np.roll(kap, -n) - kap) / (n * ds)
        j = int(np.argmax(d))
        if d[j] > worst:
            worst, where = float(d[j]), (n, j)
    return worst, where

def min_curv_matrices(reftrack, normvectors, A):
    """The min-curvature QP's own matrices, so we can add constraints to it.

    `tph.opt_min_curv.opt_min_curv` builds these internally and then calls
    quadprog, with no way to pass an extra constraint block. Everything here is
    lifted verbatim from that function (closed-track branch only) so the cost
    and the curvature linearization are bit-for-bit the ones the library uses;
    the only reason it exists is to expose `E_kappa`, the LINEAR map from the
    lateral shift alpha to the curvature at each reference point:

        kappa(alpha) = E_kappa @ alpha + k_kappa_ref

    which makes a bound on consecutive curvature DIFFERENCES a linear
    constraint too. See `opt_min_curv_dkappa`.
    """
    n = reftrack.shape[0]
    if n != normvectors.shape[0] or A.shape[0] != n * 4 or A.shape[0] != A.shape[1]:
        raise RuntimeError('min_curv_matrices: closed-track shapes expected')

    A_ex_b = np.zeros((n, n * 4), dtype=int)
    A_ex_c = np.zeros((n, n * 4), dtype=int)
    for i in range(n):
        A_ex_b[i, i * 4 + 1] = 1
        A_ex_c[i, i * 4 + 2] = 2

    A_inv = np.linalg.inv(A)
    T_c = A_ex_c @ A_inv

    M_x = np.zeros((n * 4, n))
    M_y = np.zeros((n * 4, n))
    q_x = np.zeros((n * 4, 1))
    q_y = np.zeros((n * 4, 1))
    for i in range(n):
        j = i * 4
        nxt = (i + 1) % n
        M_x[j, i] = normvectors[i, 0]
        M_x[j + 1, nxt] = normvectors[nxt, 0]
        M_y[j, i] = normvectors[i, 1]
        M_y[j + 1, nxt] = normvectors[nxt, 1]
        q_x[j, 0] = reftrack[i, 0]
        q_x[j + 1, 0] = reftrack[nxt, 0]
        q_y[j, 0] = reftrack[i, 1]
        q_y[j + 1, 0] = reftrack[nxt, 1]

    x_prime = np.eye(n) * (A_ex_b @ A_inv @ q_x)
    y_prime = np.eye(n) * (A_ex_b @ A_inv @ q_y)
    x_prime_sq = np.power(x_prime, 2)
    y_prime_sq = np.power(y_prime, 2)
    x_prime_y_prime = -2 * (x_prime @ y_prime)

    curv_den = np.power(x_prime_sq + y_prime_sq, 1.5)
    curv_part = np.divide(1, curv_den, out=np.zeros_like(curv_den),
                          where=curv_den != 0)
    curv_part_sq = np.power(curv_part, 2)

    P_xx = curv_part_sq @ y_prime_sq
    P_yy = curv_part_sq @ x_prime_sq
    P_xy = curv_part_sq @ x_prime_y_prime

    T_nx = T_c @ M_x
    T_ny = T_c @ M_y

    H = T_nx.T @ P_xx @ T_nx + T_ny.T @ P_xy @ T_nx + T_ny.T @ P_yy @ T_ny
    H = (H + H.T) / 2
    f = (2 * (q_x.T @ T_c.T) @ (P_xx @ T_nx)
         + (q_x.T @ T_c.T) @ (P_xy @ T_ny) + (q_y.T @ T_c.T) @ (P_xy @ T_nx)
         + 2 * (q_y.T @ T_c.T) @ (P_yy @ T_ny))
    f = np.squeeze(f)

    Q_x = curv_part @ y_prime
    Q_y = curv_part @ x_prime
    E_kappa = Q_y @ T_ny - Q_x @ T_nx
    k_kappa_ref = np.squeeze(Q_y @ (T_c @ q_y) - Q_x @ (T_c @ q_x))
    return H, f, E_kappa, k_kappa_ref


def opt_min_curv_dkappa(reftrack, normvectors, A, kappa_bound, w_veh,
                        dk_bound, ds, steps):
    """Min-curvature QP with a bound on the curvature RATE along the path.

    Why this exists: min-curvature bounds the integral of kappa and the QP is
    free to spend it wherever it likes, so the solution can and does step from
    a straight into a hairpin in a couple of points. The car cannot. The
    steering actuator slews at 3.2 rad/s behind 0.135 s of dead time and holds
    each command for ~0.09-0.10 s, so at 4.5 m/s it gets one command per 0.45 m
    of track; a kappa ramp of 0.7 1/m over one metre asks for ~0.22 rad of
    steering inside ~0.2 s. Every collision measured at the fast sensor rate is
    at exactly such a ramp (Porto hairpin entry, s ~ 11.3, max d(kappa)/ds
    1.4-1.7 1/m2).

    `--alpha-smooth` only attacks this indirectly -- it smooths the SHIFT, not
    the curvature, and a smooth shift on a curved reference still produces a
    curvature step. This is the direct statement: with

        kappa(alpha) = E_kappa @ alpha + k_kappa_ref     (the QP's own
                                                          linearization)

    the constraint |kappa_{i+1} - kappa_i| <= dk_bound * ds_i is linear in
    alpha and goes straight into the same inequality block as the corridor and
    the kappa bound.

    It is a bound on the LINEARIZED curvature at the ~0.14 m reference points,
    not on the finished 0.10 m line, so the caller iterates (see the
    `--dkappa-max` outer loop in main).
    """
    import quadprog

    n = reftrack.shape[0]
    H, f, E_kappa, k_kappa_ref = min_curv_matrices(reftrack, normvectors, A)

    dev_max_right = reftrack[:, 2] - w_veh / 2
    dev_max_left = reftrack[:, 3] - w_veh / 2
    if np.any(-dev_max_right > dev_max_left) or np.any(-dev_max_left > dev_max_right):
        raise RuntimeError('Problem not solvable, track might be too small to '
                           'run with current safety distance!')

    con_ge = kappa_bound - k_kappa_ref
    con_le = kappa_bound + k_kappa_ref

    # circular curvature difference over each window in the band, per metre
    rows, rhs = [np.eye(n), -np.eye(n), E_kappa, -E_kappa], \
                [dev_max_right, dev_max_left, con_ge, con_le]
    ds_mean = float(np.mean(ds))
    for m in steps:
        S = np.roll(np.eye(n), -m, axis=0) - np.eye(n)
        E_d = (S @ E_kappa) / (m * ds_mean)
        k_d = (S @ k_kappa_ref) / (m * ds_mean)
        rows += [E_d, -E_d]
        rhs += [dk_bound - k_d, dk_bound + k_d]
    return quadprog.solve_qp(H, -f, -np.vstack(rows).T,
                             -np.concatenate(rhs), 0)[0]


def smooth_alpha(alpha, win, corridor):
    """Circular boxcar on the lateral shift, projected back into `corridor`.

    Factored out of main() unchanged so the --dkappa-max outer loop can measure
    the line the smoothing actually produces. `corridor` is (lo, hi) or None
    (centerline-only, or the default window, which is left unprojected exactly
    as before).
    """
    kern = np.ones(win) / win
    alpha = np.convolve(np.concatenate([alpha[-win:], alpha, alpha[:win]]),
                        kern, mode='same')[win:-win]
    if win != 5 and corridor is not None:
        lo, hi = corridor
        k3 = np.ones(3) / 3.0
        for _ in range(8):
            alpha = np.clip(alpha, lo, hi)
            alpha = np.convolve(np.concatenate([alpha[-3:], alpha, alpha[:3]]),
                                k3, mode='same')[3:-3]
        alpha = np.clip(alpha, lo, hi)
    return alpha


def line_kappa(reftrack, normvec, alpha, stepsize):
    """Curvature of the finished line, exactly as main() computes it.

    Returns (kappa, el_lengths, s, max |d(kappa)/ds|). This is the number the
    curvature-rate work is judged on: the median-filtered analytic curvature of
    the 0.10 m resample, not the QP's linearization at the reference points.
    """
    raceline, _, cx, cy, inds, tv, s_race, _, el = \
        tph.create_raceline.create_raceline(
            refline=reftrack[:, :2], normvectors=normvec, alpha=alpha,
            stepsize_interp=stepsize)
    _, kap = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=cx, coeffs_y=cy, ind_spls=inds, t_spls=tv)
    kap = ndimage.median_filter(kap, size=5, mode='wrap')
    dk = np.abs(np.diff(np.append(kap, kap[0]))) / np.maximum(el, 1e-6)
    return kap, el, s_race[:len(kap)], float(dk.max())


def line_rate(reftrack, normvec, alpha, args):
    """(windowed rate, raw per-sample rate) of the finished line."""
    kap, el, s, raw = line_kappa(reftrack, normvec, alpha, args.stepsize)
    ds = float(np.mean(el))
    steps = window_steps(ds, args.dkappa_window_min, args.dkappa_window)
    got, _ = rate_metric(kap, ds, steps)
    return got, raw, kap, el, s, steps, ds


def fine_jacobian(E_kappa, n_fine):
    """dkappa/dalpha at the 0.10 m samples, interpolated from the QP's rows.

    E_kappa is the Jacobian at the ~0.14 m reference points. The finished line
    is resampled at 0.10 m, and the rate metric lives there, so the refinement
    needs the same Jacobian on the fine grid. Curvature sensitivity varies
    smoothly between neighbouring reference points, so linear interpolation of
    the rows is enough for a Gauss-Newton step whose residual is MEASURED
    exactly each iteration.
    """
    n = E_kappa.shape[0]
    pos = np.linspace(0.0, n, n_fine, endpoint=False)
    i0 = np.floor(pos).astype(int) % n
    w = (pos - np.floor(pos))[:, None]
    return (1.0 - w) * E_kappa[i0] + w * E_kappa[(i0 + 1) % n]


def slew_limit_kappa(kap, ds, lim, rounds=60):
    """The nearest curvature profile whose rate stays inside `lim`.

    Shares the neighbours' excess equally instead of pushing one of them, so a
    ramp is spread rather than shifted: the result is the profile the line
    should have, which is what the refinement least-squares fits towards. It is
    not itself geometrically realizable -- the corridor and the spline decide
    how much of it can be reached.
    """
    k = np.asarray(kap, dtype=float).copy()
    n = len(k)
    for _ in range(rounds):
        changed = False
        for i in range(n):
            j = (i + 1) % n
            d = k[j] - k[i]
            if abs(d) > lim * ds:
                ex = (abs(d) - lim * ds) / 2.0 * np.sign(d)
                k[i] += ex
                k[j] -= ex
                changed = True
        if not changed:
            break
    return k


def refine_dkappa(reftrack, normvec, E_kappa, alpha, corridor, target, args):
    """Bend the FINISHED line towards a rate-limited curvature profile.

    Why a second stage exists: the constraint inside the QP is exact only for
    the LINEARIZED curvature at the 0.14 m reference points, and the pipeline
    then resamples the spline at 0.10 m, median-filters it and (at
    --alpha-smooth 11) moves the shift again. Measured on Porto: asking the QP
    for 1.0 1/m2 left the finished line at 1.6-2.1, and tightening the in-QP
    bound reaches infeasibility long before the line is smooth.

    Two things were tried here. Asking for the minimum-norm correction that
    satisfies the rate as an INEQUALITY does not work: the Jacobian from
    curvature to lateral shift is stiff (a 0.6 mm shift moves the measured rate
    by 1.0 1/m2), so the QP returns a high-frequency correction that the spline
    cannot express and the true metric wanders instead of falling.

    What works is fitting a TARGET: slew-limit the measured curvature, then ask
    for the smoothest corridor-legal shift whose linearized curvature change
    moves towards it, and accept a scaled step only if the MEASURED rate
    actually improves. Measured on Porto's w_veh 0.63 line, windowed rate
    1.155 -> 1.00 / 0.78 / 0.60 for targets 1.0 / 0.8 / 0.6, peak |kappa|
    unchanged at 0.83 and lateral shifts of a few millimetres. Below ~0.6 the
    corridor stops it (target 0.45 floors at 0.61).
    """
    import quadprog

    lo, hi = corridor
    n = len(alpha)
    tr = args.dkappa_step
    D2 = (np.roll(np.eye(n), -1, axis=0) - 2 * np.eye(n)
          + np.roll(np.eye(n), 1, axis=0))
    reg = args.dkappa_reg * np.eye(n) + args.dkappa_smooth_reg * (D2.T @ D2)
    best = None
    for it in range(max(1, args.dkappa_refine)):
        got, raw, kap, el, s, steps, ds = line_rate(reftrack, normvec, alpha,
                                                    args)
        if best is None or got < best[0]:
            best = (got, alpha.copy(), raw)
        if got <= target * 1.02:
            break
        # aim slightly under the target: the fit only ever gets part of the way
        k_t = slew_limit_kappa(kap, ds, target * 0.9)
        E_f = fine_jacobian(E_kappa, len(kap))
        H = E_f.T @ E_f + reg
        f = E_f.T @ (k_t - kap)
        G = np.vstack((np.eye(n), -np.eye(n)))
        h = np.concatenate((np.minimum(hi - alpha, tr),
                            np.minimum(alpha - lo, tr)))
        try:
            da = quadprog.solve_qp(H, f, -G.T, -h, 0)[0]
        except ValueError as e:
            print(f'  dkappa refine {it + 1}: step QP failed ({e})')
            break
        step_best = None
        for sc in (0.25, 0.5, 1.0, 1.5, 2.0):
            cand = np.clip(alpha + sc * da, lo, hi)
            g2 = line_rate(reftrack, normvec, cand, args)[0]
            if step_best is None or g2 < step_best[0]:
                step_best = (g2, cand)
        if step_best[0] >= got - 1e-4:
            break                      # the corridor, not the solver, is the wall
        alpha = step_best[1]
    got, raw, *_ = line_rate(reftrack, normvec, alpha, args)
    if got < best[0]:
        best = (got, alpha.copy(), raw)
    return best[1], best[0], best[2]


def wall_bounds(reftrack, normvec, args):
    """Per-reference-point alpha bounds that keep the PATH `M` from any wall.

    Measured against the map's own distance transform, not against the
    corridor half-widths `extract_centerline.py` reports. That distinction is
    the whole point and it was measured, not assumed: bounding the REPORTED
    width by 0.55 m on cdctf2025 produced a finished line whose true minimum
    wall distance was 0.470 m, because the reported width is taken along the
    centerline normal and a corner's nearest wall is not on that normal.

    For each reference point the clearance is sampled along the normal on a
    1 cm grid and the largest contiguous feasible run containing alpha = 0 is
    kept. Where no alpha satisfies M the bound collapses on the roomiest
    sample, so a genuine pinch pins the line to the widest point rather than
    making the track infeasible.
    """
    from footprint_analysis import load_dist, clearance as clear_at

    dist, res, ox, oy = load_dist(args.min_halfwidth_map)
    m = args.min_halfwidth
    reach = float(np.max(reftrack[:, 2:4])) + 0.10
    grid = np.arange(-reach, reach + 1e-9, 0.01)
    lo = np.empty(len(reftrack))
    hi = np.empty(len(reftrack))
    for i in range(len(reftrack)):
        pts = reftrack[i, :2][None, :] + grid[:, None] * normvec[i][None, :]
        c = np.nan_to_num(clear_at(dist, res, ox, oy, pts), nan=0.0)
        ok = c >= m
        z = int(np.argmin(np.abs(grid)))
        if not ok[z]:
            k = int(np.argmax(c))
            lo[i] = hi[i] = grid[k]
            continue
        a = z
        while a > 0 and ok[a - 1]:
            a -= 1
        b = z
        while b < len(grid) - 1 and ok[b + 1]:
            b += 1
        lo[i], hi[i] = grid[a], grid[b]
    # OPT-IN `--side-margin`: an EXTRA one-sided floor on top of `m`.
    # `hi` is the last offset along +normal (RIGHT of travel) at which the
    # path still keeps `m` from every wall, so `hi - M` keeps `m + M` off the
    # wall on the right, and likewise `lo + M` on the left. This is where the
    # margin has to be stated: shrinking the reported width columns alone is
    # inert wherever the corridor bound is not the active constraint (measured
    # on the finals track -- the line sits 0.33 m inside its own corridor
    # bound at the exits this exists for, so a 0.06-0.14 m corridor shrink
    # moved it by exactly zero).
    if getattr(args, 'side_margin', None):
        sh = side_shrink(reftrack, args.side_margin, args.side_margin_taper)
        hi_new = np.minimum(hi, hi - sh['right'])
        lo_new = np.maximum(lo, lo + sh['left'])
        # never invert the interval: a window placed on a pinch keeps the
        # roomiest position it can rather than making the track infeasible
        cross = hi_new < lo_new
        mid = 0.5 * (lo + hi)
        hi_new = np.where(cross, np.clip(mid, lo, hi), hi_new)
        lo_new = np.where(cross, hi_new, lo_new)
        n_side = int(np.sum((sh['right'] > 1e-3) | (sh['left'] > 1e-3)))
        print(f'  side margin: extra floor at {n_side} of {len(lo)} reference '
              f'points (max right {sh["right"].max():.3f} m, left '
              f'{sh["left"].max():.3f} m); {int(cross.sum())} could not take '
              f'it')
        lo, hi = lo_new, hi_new
    n_pinch = int(np.sum(hi - lo < 1e-9))
    print(f'  min-halfwidth {m:.2f} m: corridor from the map, median width '
          f'{np.median(hi - lo):.3f} m, {n_pinch} of {len(lo)} reference '
          f'points cannot reach it')
    return lo, hi


# --------------------------------------------------------------------------
# lap checkpoints (OPT-IN, September 12)
# --------------------------------------------------------------------------
# The sim only counts a lap when the car has passed every checkpoint, and the
# rules say so explicitly ("Failing to pass all the checkpoints before crossing
# the finish line ... will not stop the lap timer") while promising that their
# locations are NOT revealed.  On a track whose scene we can read they are, and
# `tools/map_from_mesh.py` writes them out: one Unity trigger box per
# checkpoint, measured a unit cube scaled to `width` laterally, 0.100 m deep
# along travel and 0.254 m tall, centred on the track.  A generated line that
# threads a corner outside one of those boxes would run a whole lap for
# nothing, which is worth more than any collision penalty.


def parse_side_margin(spec):
    """"s0:s1:side:metres,..." -> [(s0, s1, side, metres), ...]."""
    out = []
    for part in str(spec).split(','):
        part = part.strip()
        if not part:
            continue
        f = part.split(':')
        if len(f) != 4:
            raise ValueError(f'--side-margin: "{part}" is not '
                             f's0:s1:side:metres')
        s0, s1, side, m = float(f[0]), float(f[1]), f[2].strip().lower(), \
            float(f[3])
        if side not in ('left', 'right'):
            raise ValueError(f'--side-margin: side must be left|right, '
                             f'got "{f[2]}"')
        if m < 0.0:
            raise ValueError('--side-margin: metres must be >= 0')
        out.append((s0, s1, side, m))
    return out


def _window_weight(s, s0, s1, taper, total):
    """Smoothstep weight: 1 inside [s0, s1], fading to 0 over `taper` outside.

    `s0 > s1` wraps through s = 0.  The ramp lives outside the window so the
    stated window carries the full margin -- the same convention as
    `scripts/win_stack_blend.py` and the controller's `param_schedule`.
    """
    taper = max(float(taper), 1e-6)
    # Signed distance INTO the window, wrapped: >= 0 inside.
    d0 = (s - s0) % total                 # distance travelled past s0
    span = (s1 - s0) % total
    if span == 0.0:
        span = total
    inside = d0 <= span
    # distance outside the window, on whichever end is nearer
    before = (s0 - s) % total             # how far before s0
    after = (s - s1) % total              # how far after s1
    d_out = np.where(inside, 0.0, np.minimum(before, after))
    u = np.clip(1.0 - d_out / taper, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)        # smoothstep


def side_shrink(reftrack, spec, taper):
    """Per-reference-point margin to take off each side, metres.

    Returned as {'left': array, 'right': array} over `reftrack`'s OWN
    arc length, so the same spec gives the same physical windows whether it is
    evaluated on the full centerline or on the thinned reference the QP uses.
    """
    xy = reftrack[:, :2]
    d = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
    s = np.r_[0.0, np.cumsum(d[:-1])]
    total = float(s[-1] + d[-1])
    out = {'left': np.zeros(len(s)), 'right': np.zeros(len(s))}
    for s0, s1, side, m in parse_side_margin(spec):
        out[side] = np.maximum(out[side],
                               m * _window_weight(s, s0, s1, taper, total))
    return out


def apply_side_margin(reftrack, spec, taper, verbose=True):
    """Shrink one corridor side over arc-length windows of the centerline.

    Column 2 is `w_tr_right_m` and column 3 `w_tr_left_m`; tph's normals point
    to the RIGHT of travel, so column 2 really is the distance to the wall on
    the right (see `corridor_of` for the measurement that establishes this).
    Shrinking column 2 by M therefore tells every downstream stage -- the QP
    bound, the body sweep, `work`, the alpha smoothing projection -- that the
    right wall is M metres closer than it is, which is exactly "keep M of
    extra margin off the right wall here".

    A floor of 0.05 m is kept on the shrunk side so a window placed where the
    corridor is narrow cannot make it infeasible; the print says when it bit.
    """
    if not parse_side_margin(spec):
        return reftrack
    shrink = side_shrink(reftrack, spec, taper)
    s = np.arange(len(reftrack))
    out = reftrack.copy()
    for side, col in (('right', 2), ('left', 3)):
        if shrink[side].max() <= 0.0:
            continue
        before = out[:, col].copy()
        out[:, col] = np.maximum(out[:, col] - shrink[side], 0.05)
        clipped = int(np.sum(before - out[:, col] < shrink[side] - 1e-9))
        if verbose:
            print(f'  side margin {side}: up to '
                  f'{shrink[side].max():.3f} m off the corridor over '
                  f'{int(np.sum(shrink[side] > 1e-3))} of {len(s)} reference '
                  f'points (taper {taper:g} m)'
                  + (f' -- 0.05 m floor bit at {clipped} points'
                     if clipped else ''))
    return out


def load_checkpoints(path):
    """`(index, x, y, yaw, width)` for a map's numbered lap checkpoints.

    `width` is the FULL lateral extent of the trigger box, so the box spans
    +/- width/2 either side of (x, y) along the gate, which is the direction
    `(sin yaw, -cos yaw)`.  The two named finish lines duplicate numbered
    checkpoints and are dropped, leaving exactly the sequence the lap counter
    walks.
    """
    rows = []
    with open(path) as fh:
        next(fh)
        for line in fh:
            if not line.strip():
                continue
            kind, name, x, y, yaw, w = line.rstrip('\n').split(',')
            if kind == 'checkpoint' and name.isdigit():
                rows.append((int(name), float(x), float(y),
                             float(yaw), float(w)))
    if not rows:
        raise ValueError(f'{path}: no numbered checkpoints')
    rows.sort()
    return rows


def _gate_axes(yaw):
    """(travel direction, lateral direction) of a checkpoint gate."""
    return (np.array([math.cos(yaw), math.sin(yaw)]),
            np.array([math.sin(yaw), -math.cos(yaw)]))


def _plane_cuts(xy, cx, cy, yaw, closed=True):
    """Indices and interpolated lateral offsets where `xy` cuts a gate plane."""
    P = np.vstack([xy, xy[:1]]) if closed else np.asarray(xy, float)
    hd, lat = _gate_axes(yaw)
    r = P - np.array([cx, cy])
    d, l = r @ hd, r @ lat
    out = []
    for i in np.flatnonzero(np.sign(d[:-1]) != np.sign(d[1:])):
        f = abs(d[i]) / (abs(d[i]) + abs(d[i + 1]) + 1e-12)
        out.append((int(i), float(l[i] + f * (l[i + 1] - l[i]))))
    return out


def checkpoint_margins(xy, checkpoints, closed=True):
    """`{index: margin}` -- how far INSIDE each trigger box the path passes.

    ``margin = width/2 - |lateral offset at the crossing|``.  Negative means
    the path cuts that checkpoint's plane outside its box: the trigger never
    fires and the lap does not count.  ``None`` means the path never reaches
    the plane at all.  Where a path cuts one plane more than once (a gate whose
    plane extends across another part of the circuit) the crossing nearest the
    box centre is the one taken.
    """
    out = {}
    for name, cx, cy, yaw, w in checkpoints:
        cuts = _plane_cuts(xy, cx, cy, yaw, closed)
        out[name] = (None if not cuts
                     else w / 2.0 - abs(min((c[1] for c in cuts), key=abs)))
    return out


# --------------------------------------------------------------------------
# gate separation and occupancy (OPT-IN, September 12)
# --------------------------------------------------------------------------
# The lap counter (LapTimer.cs) keeps ONE global `CheckpointFlag` and
# overwrites `PreviousCheckpoint` on EVERY trigger exit, so a lap only counts
# when cp20's enter is accepted while PreviousCheckpoint == 19.  The car is
# 0.50 x 0.27 m with ~15 colliders, each firing its own enter/exit, so being
# inside two trigger boxes at once lets one box's exit overwrite the other's
# state and drop a checkpoint.  Measured on cdctf2025 fast3_m53: gates 17-20
# sit 0.26 m apart along that line (0.13 s/lap of TRIPLE occupancy) and one
# lap in 32 was not counted; the safe line holds 0.39-0.41 m and never even
# doubles.
#
# `GATE_DEPTH` is the trigger box's extent along travel, read off the scene by
# `tools/map_from_mesh.py` (width x 0.254 tall x 0.100 deep).

GATE_DEPTH = 0.100


def _rect_corners(cx, cy, hd, lat, half_along, half_lat):
    c = np.array([cx, cy], float)
    return np.array([c + a * half_along * hd + b * half_lat * lat
                     for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))])


def _obb_overlap(a, b):
    """SAT test for two convex boxes given as 4x2 corner arrays."""
    for poly in (a, b):
        e = np.roll(poly, -1, axis=0) - poly
        for ex, ey in e:
            ax = np.array([-ey, ex])
            n = math.hypot(ax[0], ax[1])
            if n < 1e-12:
                continue
            ax = ax / n
            pa, pb = a @ ax, b @ ax
            if pa.max() < pb.min() - 1e-12 or pb.max() < pa.min() - 1e-12:
                return False
    return True


def _arclength(xy, closed=True):
    P = np.vstack([xy, xy[:1]]) if closed else np.asarray(xy, float)
    d = np.linalg.norm(np.diff(P, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)]), float(d.sum())


def gate_crossings(xy, checkpoints, closed=True):
    """`{index: (s, lateral)}` -- where the path cuts each gate plane.

    `s` is arclength along the path at the crossing nearest the box centre and
    `lateral` its offset from that centre along the gate.  A gate the path
    never reaches is absent.
    """
    s, _L = _arclength(xy, closed)
    out = {}
    for name, cx, cy, yaw, w in checkpoints:
        cuts = _plane_cuts(xy, cx, cy, yaw, closed)
        if not cuts:
            continue
        i, lat = min(cuts, key=lambda c: abs(c[1]))
        hd, _ = _gate_axes(yaw)
        P = np.vstack([xy, xy[:1]]) if closed else np.asarray(xy, float)
        r0 = (P[i] - np.array([cx, cy])) @ hd
        r1 = (P[i + 1] - np.array([cx, cy])) @ hd
        f = abs(r0) / (abs(r0) + abs(r1) + 1e-12)
        out[name] = (float(s[i] + f * (s[i + 1] - s[i])), float(lat))
    return out


def gate_separations(xy, checkpoints, closed=True):
    """`{(a, b): along-line metres}` between consecutive gate planes.

    Measured along the PATH at the lateral position the path actually takes,
    because that is what the car has to traverse between one trigger and the
    next.  The pair that wraps the finish line is included.
    """
    cr = gate_crossings(xy, checkpoints, closed)
    _s, L = _arclength(xy, closed)
    names = [n for n, *_ in checkpoints if n in cr]
    out = {}
    for k, a in enumerate(names):
        b = names[(k + 1) % len(names)]
        if b == a:
            continue
        d = cr[b][0] - cr[a][0]
        if closed:
            d = d % L
        out[(a, b)] = float(d)
    return out


def _gate_boxes(checkpoints):
    out = []
    for name, cx, cy, yaw, w in checkpoints:
        hd, lat = _gate_axes(yaw)
        out.append((name, _rect_corners(cx, cy, hd, lat,
                                        GATE_DEPTH / 2.0, w / 2.0),
                    np.array([cx, cy]), max(w / 2.0, GATE_DEPTH / 2.0)))
    return out


def _occupancy_counts(px, py, head, gates):
    """How many of `gates` the body box overlaps at each pose."""
    car_reach = math.hypot(max(FRONT, REAR_OVERHANG), HALF_W)
    counts = np.zeros(len(px), int)
    worst = ()
    for k in range(len(px)):
        c, sn = math.cos(head[k]), math.sin(head[k])
        hd = np.array([c, sn])
        lat = np.array([-sn, c])
        mid = np.array([px[k], py[k]]) + hd * (FRONT - REAR_OVERHANG) / 2.0
        body = _rect_corners(mid[0], mid[1], hd, lat,
                             (FRONT + REAR_OVERHANG) / 2.0, HALF_W)
        hit = []
        for name, box, ctr, reach in gates:
            if np.linalg.norm(ctr - mid) > reach + car_reach:
                continue
            if _obb_overlap(body, box):
                hit.append(name)
        counts[k] = len(hit)
        if len(hit) > len(worst):
            worst = tuple(sorted(hit))
    return counts, list(worst)


def gate_occupancy_poses(x, y, yaw, t, checkpoints):
    """The same measurement on a DRIVEN trajectory (harness or race log).

    Poses are the car's own, so the count is what the sim's triggers would
    have seen, and the times are wall time rather than the plan's.
    """
    x, y, yaw, t = (np.asarray(v, float) for v in (x, y, yaw, t))
    counts, worst = _occupancy_counts(x, y, yaw, _gate_boxes(checkpoints))
    dt = np.gradient(t)
    return {
        'max_occupancy': int(counts.max()),
        'worst_gates': worst,
        'time_double_s': float(dt[counts >= 2].sum()),
        'time_triple_s': float(dt[counts >= 3].sum()),
    }


def gate_occupancy(xy, vx, checkpoints, step=0.01, closed=True):
    """How many trigger boxes the BODY is inside at once, and for how long.

    The 0.50 x 0.27 m body box is swept along the path (rear axle at the path
    point, as everywhere else in this repo) at `step` metre resolution and
    tested against every gate box with SAT.  Returns the maximum simultaneous
    occupancy, the time per lap spent at each level >= 2 at the line's own
    planned speed, and the worst run of gates seen together.
    """
    P = np.vstack([xy, xy[:1]]) if closed else np.asarray(xy, float)
    s, L = _arclength(xy, closed)
    v = np.concatenate([vx, vx[:1]]) if closed else np.asarray(vx, float)
    q = np.arange(0.0, L, step)
    px = np.interp(q, s, P[:, 0])
    py = np.interp(q, s, P[:, 1])
    pv = np.maximum(np.interp(q, s, v), 0.1)
    hx = np.gradient(px)
    hy = np.gradient(py)
    head = np.arctan2(hy, hx)

    counts, worst_set = _occupancy_counts(px, py, head,
                                          _gate_boxes(checkpoints))
    dt = step / pv
    return {
        'max_occupancy': int(counts.max()),
        'worst_gates': list(worst_set),
        'time_double_s': float(dt[counts >= 2].sum()),
        'time_triple_s': float(dt[counts >= 3].sum()),
        'dist_double_m': float(step * int((counts >= 2).sum())),
        'dist_triple_m': float(step * int((counts >= 3).sum())),
    }


def gate_pair_distance(ci, lati, li, cj, latj, lj):
    """Straight-line distance between two gate-plane crossings."""
    d = (ci + np.outer(li, lati)[:, None, :]
         - cj - np.outer(lj, latj)[None, :, :])
    return np.linalg.norm(d, axis=2)


def gate_separation_bands(checkpoints, ref_xy, sep_min, margin, limits=None,
                          geom=None, slope_max=0.5,
                          n_grid=81, reward_cap=0.90, tie_w=0.05,
                          band_floor=0.12,
                          verbose=True):
    """Per-gate lateral bands that keep consecutive gate planes `sep_min` apart.

    The car can only be inside one trigger box at a time if consecutive gate
    planes are further apart along its path than the body plus the box, and
    how far apart they are is a function of WHERE the path cuts each plane:
    around a hairpin the gates fan out, so a line that runs inside collapses
    the spacing (cdctf2025, gates 17-20).

    Structure of the problem: each gate contributes one scalar (the lateral
    offset of its crossing) and the constraint couples only CONSECUTIVE gates,
    so the whole thing is a chain and a dynamic program solves it exactly on a
    grid.  The reward saturates at `sep_min + reward_cap` so that a pair which
    cannot reach `sep_min` at any admissible lateral is pushed as far apart as
    the geometry allows instead of making the problem infeasible ("where
    infeasible, minimise occupancy"), and a small pull towards the reference
    line's own crossings breaks ties.

    The DP gives one point per gate; the QP needs an interval.  A crossing
    moved by r moves by at most r, so a pair keeps `sep_min` for ANY choice
    inside the bands as long as `r_i + r_j <= sep_ij - sep_min`.  Each band is
    the largest radius satisfying that against both its neighbours.

    Returns `({name: (lo, hi)}, info)` with the chosen laterals and the
    separations they realise.
    """
    names = [c[0] for c in checkpoints]
    ctr = {c[0]: np.array([c[1], c[2]]) for c in checkpoints}
    lat = {c[0]: _gate_axes(c[3])[1] for c in checkpoints}
    half = {c[0]: max(c[4] / 2.0 - margin, 0.0) for c in checkpoints}
    ref = gate_crossings(ref_xy, checkpoints)
    # A trigger box is often wider than the track (cdctf2025 gate 19 is 5.0 m
    # across), so the box alone would let the DP choose a crossing the corridor
    # cannot reach and hand the QP an empty interval.  `limits` is the lateral
    # range the PATH can actually take at each gate, from the corridor.
    span = {}
    for n in names:
        lo, hi = -half[n], half[n]
        if limits is not None and n in limits:
            lo, hi = max(lo, limits[n][0]), min(hi, limits[n][1])
            if lo > hi:
                mid = float(np.clip(0.5 * (limits[n][0] + limits[n][1]),
                                    -half[n], half[n]))
                lo = hi = mid
        span[n] = (lo, hi)
    grids = {n: (np.linspace(span[n][0], span[n][1], n_grid)
                 if span[n][1] - span[n][0] > 1e-9
                 else np.array([span[n][0]])) for n in names}
    K = len(names)
    edges = [(names[k], names[(k + 1) % K]) for k in range(K)]
    # How far the PATH has to be pushed off the reference to put a crossing at
    # a given lateral: `dev = (l - a0) / g`.  Two gates a metre apart cannot
    # ask for +1.0 m of deviation and then -0.1 m -- that is a curvature the
    # QP will refuse, and asking for it is what made the solver report
    # "corridor exhausted" rather than a line.  `slope_max` is the deviation
    # the chain may build up per metre of path between two gates.
    cr_ref = gate_crossings(ref_xy, checkpoints)
    dev = {}
    if geom is not None:
        for n in names:
            if n in geom:
                a0, g = geom[n]
                if abs(g) > 1e-6:
                    dev[n] = (a0, g)
    R = {}
    for a, b in edges:
        d = gate_pair_distance(ctr[a], lat[a], grids[a],
                               ctr[b], lat[b], grids[b])
        R[(a, b)] = np.minimum(d, sep_min + reward_cap)
        if a in dev and b in dev and a in cr_ref and b in cr_ref:
            gap = abs(cr_ref[b][0] - cr_ref[a][0])
            gap = min(gap, 1e3)
            da = (grids[a] - dev[a][0]) / dev[a][1]
            db = (grids[b] - dev[b][0]) / dev[b][1]
            bad = (np.abs(da[:, None] - db[None, :])
                   > slope_max * max(gap, 0.2))
            R[(a, b)] = np.where(bad, -1e3, R[(a, b)])
    pull = {n: -tie_w * (grids[n] - (ref[n][1] if n in ref else 0.0)) ** 2
            for n in names}

    best = None
    n0 = names[0]
    for k0 in range(len(grids[n0])):
        val = np.full(len(grids[n0]), -np.inf)
        val[k0] = pull[n0][k0]
        back = []
        for a, b in edges[:-1]:
            m = val[:, None] + R[(a, b)] + pull[b][None, :]
            back.append(np.argmax(m, axis=0))
            val = m.max(axis=0)
        # close the cycle back onto the fixed first state
        val = val + R[edges[-1]][:, k0]
        j = int(np.argmax(val))
        if best is None or val[j] > best[0]:
            idx = [j]
            for bk in reversed(back):
                idx.append(int(bk[idx[-1]]))
            idx.append(k0)
            best = (float(val[j]), list(reversed(idx[:-1])))
    l_star = {n: float(grids[n][i]) for n, i in zip(names, best[1])}

    sep = {}
    for a, b in edges:
        p = ctr[a] + l_star[a] * lat[a]
        q = ctr[b] + l_star[b] * lat[b]
        sep[(a, b)] = float(np.linalg.norm(p - q))
    # The band each gate gets is the CONTIGUOUS run of admissible laterals
    # around its own solution, with its two neighbours held at theirs: the
    # distance to a fixed neighbour crossing is convex in the lateral, so
    # `>= sep_min` excludes one interval and the band is what is left on the
    # side the solution sits.  That is deliberately permissive -- these bounds
    # only STEER the QP, and the finished line's separations are measured
    # afterwards (here, in make_track_lines and in pick_config), so a band
    # that is too generous is caught rather than believed.
    # A crossing moved by r moves by at most r, so a pair keeps `sep_min` for
    # ANY choice inside the bands as long as `r_i + r_j <= sep_ij - sep_min`.
    # That is the guarantee the QP needs, because it moves every gate at once.
    slack = {e: sep[e] - sep_min for e in edges}
    bands = {}
    for k, n in enumerate(names):
        p_prev = ctr[names[k - 1]] + l_star[names[k - 1]] * lat[names[k - 1]]
        nx = names[(k + 1) % K]
        p_next = ctr[nx] + l_star[nx] * lat[nx]
        g = grids[n]
        pts = ctr[n] + np.outer(g, lat[n])
        ok = ((np.linalg.norm(pts - p_prev, axis=1) >= sep_min - 1e-9)
              & (np.linalg.norm(pts - p_next, axis=1) >= sep_min - 1e-9))
        z = int(np.argmin(np.abs(g - l_star[n])))
        if not ok[z]:
            bands[n] = (l_star[n], l_star[n])
            continue
        a = z
        while a > 0 and ok[a - 1]:
            a -= 1
        b = z
        while b < len(g) - 1 and ok[b + 1]:
            b += 1
        # ... and the robust radius, which is what actually holds when the
        # neighbours move too.  `band_floor` keeps a QP that has to thread a
        # shrinking corridor from being handed a single point; where it is the
        # binding term the constraint is a steer, not a guarantee, and the
        # finished line is measured afterwards.
        r = max(min(slack[edges[k - 1]], slack[edges[k]]) / 2.0, band_floor)
        bands[n] = (float(max(g[a], l_star[n] - r)),
                    float(min(g[b], l_star[n] + r)))
    short = sorted(e for e in edges if sep[e] < sep_min - 1e-9)
    if verbose:
        worst = min(sep.items(), key=lambda kv: kv[1])
        print(f'  gate separation {sep_min:.2f} m: solved {K} gates, '
              f'worst pair {worst[0][0]}->{worst[0][1]} {worst[1]:.3f} m, '
              f'median band {np.median([b - a for a, b in bands.values()]):.3f} m'
              + (f', {len(short)} pair(s) SHORT: '
                 + ', '.join(f'{a}->{b} {sep[(a, b)]:.3f}' for a, b in short)
                 if short else ''))
    return bands, {'lateral': l_star, 'separation': sep, 'short': short}


def gate_reach(reftrack, normvec, checkpoints, w_veh):
    """`{name: (lo, hi)}` -- lateral offsets at each gate the corridor allows.

    The QP may shift a reference point by alpha along its normal, bounded by
    the corridor it was handed (`reftrack[:, 2:4]`, minus the vehicle width),
    and the crossing then sits at `a0 + alpha * g` along the gate.  Both
    reference points bracketing the plane are bounded, so the reach is the
    INTERSECTION of what each of them can offer.
    """
    out = {}
    n = len(reftrack)
    for name, cx, cy, yaw, w in checkpoints:
        cuts = _plane_cuts(reftrack[:, :2], cx, cy, yaw)
        if not cuts:
            continue
        i = min(cuts, key=lambda c: abs(c[1]))[0]
        _hd, lat = _gate_axes(yaw)
        lo, hi = -np.inf, np.inf
        for j in (i % n, (i + 1) % n):
            a0 = float((reftrack[j, :2] - np.array([cx, cy])) @ lat)
            g = float(normvec[j] @ lat)
            # tph's alpha runs along the normal, which points RIGHT of
            # travel, so it is bounded ABOVE by the right width and below by
            # the left one (verified against opt_min_curv's own box).
            al = -(reftrack[j, 3] - w_veh / 2.0)
            ah = reftrack[j, 2] - w_veh / 2.0
            if ah < al:
                al = ah = 0.5 * (al + ah)
            if abs(g) < 0.2:
                continue
            a, b = sorted((a0 + al * g, a0 + ah * g))
            lo, hi = max(lo, a), min(hi, b)
        if np.isfinite(lo) and np.isfinite(hi) and hi >= lo:
            out[name] = (lo, hi)
    return out


def gate_frames(reftrack, normvec, checkpoints):
    """`{name: (a0, g)}` -- the reference crossing offset and normal leverage.

    `a0` is where the REFERENCE line cuts the gate, measured along the gate
    from the box centre, and `g = normal . gate` is how much of a lateral
    shift of the reference point lands on the gate plane.  Together they turn
    a wanted crossing into the path deviation it costs: `dev = (l - a0) / g`.
    """
    out = {}
    n = len(reftrack)
    for name, cx, cy, yaw, w in checkpoints:
        cuts = _plane_cuts(reftrack[:, :2], cx, cy, yaw)
        if not cuts:
            continue
        i = min(cuts, key=lambda c: abs(c[1]))[0] % n
        _hd, lat = _gate_axes(yaw)
        a0 = float((reftrack[i, :2] - np.array([cx, cy])) @ lat)
        g = float(normvec[i] @ lat)
        if abs(g) >= 0.2:
            out[name] = (a0, g)
    return out


def checkpoint_bounds(reftrack, normvec, args):
    """Alpha bounds that hold the path inside every trigger box.

    The QP shifts each reference point by `alpha` along its own normal, so a
    gate constrains the two reference points that bracket its plane: with
    `a0` the reference point's own offset from the box centre along the gate
    and `g = normal . gate`, the path crosses at `a0 + alpha * g`, which must
    stay inside `width/2 - margin`.  A gate whose plane the normal is nearly
    parallel to (|g| < 0.2) gives alpha almost no authority there and is left
    alone rather than bounded by a near-singular division.
    """
    cps = load_checkpoints(args.checkpoints)
    m = args.checkpoint_margin
    n = len(reftrack)
    lo = np.full(n, -np.inf)
    hi = np.full(n, np.inf)
    n_bound = n_skip = 0
    # OPT-IN on top of the margin: --gate-separation also says WHERE inside
    # each box the crossing may sit, so that consecutive gate planes stay far
    # enough apart along the path for the body to leave one trigger before it
    # enters the next.
    for name, cx, cy, yaw, w in cps:
        half = w / 2.0 - m
        if half <= 0.0:
            sys.exit(f'--checkpoint-margin {m:g} leaves nothing of checkpoint '
                     f'{name}, whose trigger is only {w:.2f} m wide')
        cuts = _plane_cuts(reftrack[:, :2], cx, cy, yaw)
        if not cuts:
            print(f'  checkpoint {name}: the reference line never crosses it '
                  f'-- not constrained')
            n_skip += 1
            continue
        i = min(cuts, key=lambda c: abs(c[1]))[0]
        _hd, lat = _gate_axes(yaw)
        for j in (i % n, (i + 1) % n):
            a0 = float((reftrack[j, :2] - np.array([cx, cy])) @ lat)
            g = float(normvec[j] @ lat)
            if abs(g) < 0.2:
                continue
            a, b = sorted(((-half - a0) / g, (half - a0) / g))
            lo[j] = max(lo[j], a)
            hi[j] = min(hi[j], b)
            n_bound += 1
    tight = np.isfinite(lo) | np.isfinite(hi)
    print(f'  checkpoints: {len(cps)} gates, margin {m:.2f} m, '
          f'{n_bound} reference-point bounds on {int(tight.sum())} points'
          + (f', {n_skip} gate(s) unreachable' if n_skip else ''))
    return lo, hi


def gate_bounds(reftrack, normvec, args):
    """Alpha bounds from --gate-separation, in the SOLVER's own sign.

    tph's `alpha` runs along the normal, which points to the right of travel,
    so a crossing sits at `a0 + alpha * g` and a band maps straight through.
    These are kept apart from `checkpoint_bounds` because the two are consumed
    differently: the checkpoint-MARGIN bounds are symmetric about the
    reference and so survived `corridor_of`'s old mirrored convention
    (`-w_right, +w_left`), while a gate BAND is one-sided by design -- fed
    through that mirror it put the QP's alpha 1.4 m the wrong side of its own
    bound, measured.  `corridor_of` no longer mirrors on this path.
    """
    cps = load_checkpoints(args.checkpoints)
    floor = getattr(args, '_gate_relax', None)
    n = len(reftrack)
    lo = np.full(n, -np.inf)
    hi = np.full(n, np.inf)
    if floor is None:
        return lo, hi
    bands = gate_separation_bands(
        cps, reftrack[:, :2], args.gate_separation, args.checkpoint_margin,
        limits=gate_reach(getattr(args, '_gate_reach_track', reftrack),
                          normvec, cps, args.w_veh),
        geom=gate_frames(reftrack, normvec, cps), band_floor=floor)[0]
    for name, cx, cy, yaw, w in cps:
        cuts = _plane_cuts(reftrack[:, :2], cx, cy, yaw)
        if not cuts:
            continue
        i = min(cuts, key=lambda c: abs(c[1]))[0]
        _hd, lat = _gate_axes(yaw)
        b_lo, b_hi = bands[name]
        for j in (i % n, (i + 1) % n):
            a0 = float((reftrack[j, :2] - np.array([cx, cy])) @ lat)
            g = float(normvec[j] @ lat)
            if abs(g) < 0.2:
                continue
            a, b = sorted(((b_lo - a0) / g, (b_hi - a0) / g))
            lo[j] = max(lo[j], a)
            hi[j] = min(hi[j], b)
    return lo, hi


def corridor_of(reftrack, normvec, work, args, gates=True):
    """(lo, hi) bounds on the lateral shift alpha, as the solver sees them.

    SIGN CONVENTION (established numerically, `tests/test_corridor_sign.py`).
    tph's spline normals point to the RIGHT of travel, and `opt_min_curv`
    bounds its alpha by `alpha <= reftrack[:, 2] - w_veh/2` and
    `alpha >= -(reftrack[:, 3] - w_veh/2)`: column 2 (`w_tr_right_m`) is the
    UPPER bound and column 3 (`w_tr_left_m`) the lower one. Measured against
    the maps' own distance transforms, column 2 really is the wall distance
    along +normal (cdctf2025 mean |err| 0.021 m against 1.312 m for column 3,
    icra2025 0.015 against 0.541), so tph's box is the physical one and
    positive alpha means RIGHT.

    `(-work[:, 2], work[:, 3])` -- what this function returned for every line
    ever shipped -- is therefore MIRRORED with respect to the QP's own box: it
    clips alpha at the left wall on the right and vice versa. A bound that is
    symmetric about the reference survives that; a ONE-SIDED bound
    (`--min-halfwidth`, a gate band) does not. Restated through the mirror in
    `_sweep_solve`, a wall floor whose feasible interval was `[lo, hi]` reached
    the QP as `[-hi, -lo]` -- the mirror image of itself, which both forbids
    legal positions and permits illegal ones wherever the two walls are not
    equidistant. Measured cost on the two 2025 circuits at
    `--min-halfwidth 0.53`: it is NOT what was keeping those lines under their
    floor (held 0.522 -> 0.527 m on cdctf2025, 0.526 -> 0.520 m on icra2025;
    the clearance passes bind first), but it was stating the constraint wrong.

    So the mirror is corrected on the one-sided paths and left alone on the
    plain path. That split is deliberate, not laziness: stating the plain
    corridor in tph's sign is measured to move the Porto w_veh 0.60-0.78
    `--dkappa-max` candidates (the clip binds there), and the shipping
    `porto_mesh_cr_dk0.8.csv` comes from exactly that family. Fixing it means
    re-driving those lines; until then the plain path stays bit-for-bit what
    was validated.

    OPT-IN, September 12. `--min-halfwidth M` says: the PATH may not come
    closer than M to any wall. Why it is needed on top of `--w-veh`: w_veh
    only constrains the QP, while the alpha smoothing and the `--dkappa-max`
    refinement are projected back into `work`, i.e. into sweep + 0, not
    sweep + w_veh/2. On the two 2025 circuits that is where the fast lines'
    single pinch comes from -- the QP was told to keep 0.78/2 = 0.39 m and
    the finished line still passes 0.427 m (cdctf, s 40.8) / 0.432 m
    (icra2025, s 42.5) from a wall. `--min-halfwidth` states the floor once
    and applies it at every stage.
    """
    if (getattr(args, 'min_halfwidth', None) is None
            and getattr(args, 'checkpoints', None) is None):
        return -work[:, 2], work[:, 3]      # FROZEN, mirrored -- see above
    # tph's own sign: +alpha is right, bounded above by the right width.
    lo, hi = -work[:, 3], work[:, 2]
    if gates and getattr(args, 'gate_separation', None):
        glo, ghi = gate_bounds(reftrack, normvec, args)
        lo, hi = np.maximum(lo, glo), np.minimum(hi, ghi)
    if getattr(args, '_extra_bounds', None) is None:
        elo = np.full(len(reftrack), -np.inf)
        ehi = np.full(len(reftrack), np.inf)
        if getattr(args, 'min_halfwidth', None) is not None:
            a, b = wall_bounds(reftrack, normvec, args)
            elo, ehi = np.maximum(elo, a), np.minimum(ehi, b)
        if getattr(args, 'checkpoints', None) is not None:
            a, b = checkpoint_bounds(reftrack, normvec, args)
            elo, ehi = np.maximum(elo, a), np.minimum(ehi, b)
        args._extra_bounds = (elo, ehi)
    lo2, hi2 = args._extra_bounds
    lo = np.maximum(lo, lo2)
    hi = np.minimum(hi, hi2)
    return np.minimum(lo, hi), hi


def sweep_solve(reftrack, normvec, M, args, dk_bound=None, quiet=False):
    """`_sweep_solve`, with the gate bands relaxed until the QP can solve.

    A gate band is a HARD bound at two reference points, and two of them a
    metre apart can be harder to satisfy than the corridor itself once the
    curvature rows are in.  Rather than report "corridor exhausted" -- which
    on a mesh track means shipping the plain centerline -- widen the bands a
    step at a time and, in the last resort, drop them: the finished line's
    separations are measured either way, and a line that keeps some of its
    gate spacing beats one that was never solved.  With no --gate-separation
    this is a single call and the old behaviour byte for byte.
    """
    if not getattr(args, 'gate_separation', None):
        return _sweep_solve(reftrack, normvec, M, args, dk_bound, quiet)
    base = args.gate_band_floor
    kb0 = args.kappa_bound
    # Order of relaxation: the CURVATURE ceiling first, then the bands.
    # `--kappa-bound` is shaping -- it keeps the QP off the steering lock and
    # the finished line's own |kappa| is measured afterwards anyway -- while a
    # gate band is what stops a lap going uncounted, which costs a whole lap
    # time.  Measured on cdctf2025: at kappa_bound 1.0-2.2 the QP has no
    # solution with the bands in, at 3.0 it solves and the finished line
    # holds 0.74 m of gate separation with |kappa| max 1.82.
    for floor in (base, 2 * base, 4 * base, None):
        for kb in (kb0, 2 * kb0, 3 * kb0):
            args._gate_relax = floor
            args._extra_bounds = None
            args.kappa_bound = kb
            try:
                return _sweep_solve(reftrack, normvec, M, args, dk_bound,
                                    quiet)
            except (ValueError, RuntimeError):
                if not quiet:
                    print(f'  gate separation: no QP solution at band floor '
                          + (f'{floor:.2f} m' if floor is not None else 'OFF')
                          + f', kappa bound {kb:.2f}; relaxing')
                if floor is None and kb == 3 * kb0:
                    args.kappa_bound = kb0
                    raise
            if floor is None:
                break


def _sweep_solve(reftrack, normvec, M, args, dk_bound=None, quiet=False):
    """Optimize / measure body sweep / shrink the corridor / re-optimize.

    One pass with a constant w_veh cannot express this because the sweep
    depends on the solution's own curvature.
    """
    work = reftrack.copy()
    alpha = None
    ds = np.linalg.norm(np.diff(np.vstack([reftrack[:, :2],
                                           reftrack[:1, :2]]), axis=0), axis=1)
    for it in range(args.sweep_iters):
        if dk_bound is None:
            alpha = tph.opt_min_curv.opt_min_curv(
                reftrack=work, normvectors=normvec, A=M,
                kappa_bound=args.kappa_bound, w_veh=args.w_veh,
                closed=True, print_debug=False)[0]
        else:
            steps = window_steps(float(np.mean(ds)), args.dkappa_window_min,
                                 args.dkappa_window)
            alpha = opt_min_curv_dkappa(work, normvec, M, args.kappa_bound,
                                        args.w_veh, dk_bound, ds, steps)
        k_ref = curvature_of(work, normvec, alpha, args.stepsize)
        sweep = corner_sweep(k_ref)
        # `sweep` already includes the body half-width, so the usable
        # half-corridor for the *path* is the wall distance minus it.
        keep = np.maximum(reftrack[:, 2:4] - sweep[:, None], 0.05)
        if (getattr(args, 'min_halfwidth', None) is not None
                or getattr(args, 'checkpoints', None) is not None):
            if getattr(args, 'gate_separation', None):
                # The gate bands have to be asked of the corridor that is
                # LEFT, not the one we started with: the body-sweep passes
                # take up to 0.3 m of it at a hairpin, and a band the sweep
                # has already eaten is what makes the QP report "corridor
                # exhausted".  Re-derived every pass, from `work`.
                args._gate_reach_track = np.column_stack(
                    [reftrack[:, :2], keep])
                args._extra_bounds = None
            # Restate the wall floor as a corridor width, so the QP's own
            # `reftrack[:, 2] - w_veh/2` bound lands exactly on it.  `keep` is
            # read by tph as (right, left) = (upper, lower) bound on alpha, and
            # `corridor_of` now hands back (lo, hi) in that same sign, so the
            # RIGHT column takes `hi2` and the LEFT column `-lo2`.  It used to
            # take them the other way round, which handed the QP the MIRROR
            # IMAGE of a one-sided `--min-halfwidth` interval.
            inf = np.full((len(reftrack), 2), np.inf)
            lo2, hi2 = corridor_of(reftrack, normvec,
                                   np.column_stack([reftrack[:, :2], inf]),
                                   args, gates=False)
            keep_phys = keep.copy()
            keep = np.minimum(keep, np.column_stack([hi2, -lo2])
                              + args.w_veh / 2.0)
            if getattr(args, 'gate_separation', None):
                # The gate band bounds tph's own alpha too, so it is restated
                # the same way as the corridor above: `dev_right = keep_r -
                # w/2 = b`, `dev_left = keep_l - w/2 = -a`.  (It was already
                # written this way while `corridor_of` was mirrored, which is
                # how the two disagreed.)
                glo, ghi = gate_bounds(reftrack, normvec, args)
                fin = np.isfinite(glo) | np.isfinite(ghi)
                gk = np.column_stack([
                    np.where(np.isfinite(ghi), ghi, np.inf) + args.w_veh / 2.0,
                    args.w_veh / 2.0
                    - np.where(np.isfinite(glo), glo, -np.inf)])
                keep[fin] = np.minimum(keep[fin], gk[fin])
            if (getattr(args, 'gate_separation', None)
                    or getattr(args, 'checkpoints', None) is not None):
                # A GATE bound -- a band, or a box the reference line misses
                # -- legitimately makes one of the two `keep` entries
                # negative; that is how the QP is told "stay on the far
                # side".  The checkpoint branch used to floor them at 0.05 m,
                # which threw that half of the bound away: measured under
                # --gate-separation (where the floor was already skipped for
                # exactly this reason) the QP put alpha at +0.51 where the
                # band said -1.27..-0.79.  Nothing is floored now; what the
                # solver actually needs is that the two SUM to the vehicle
                # width, which the fallback here guarantees.
                #
                # The band is derived from the corridor the QP STARTED with,
                # but the body-sweep passes shrink that corridor by up to
                # 0.3 m at a hairpin, so the two can stop intersecting.  The
                # corridor wins where they do -- a line that keeps its gate
                # spacing by driving through a wall is not a line -- and the
                # finished-line report says what survived.
                #
                # NOT extended to a --min-halfwidth-only run: there the QP
                # reporting "corridor exhausted" is the honest answer and the
                # candidate is dropped, where the fallback instead hands back
                # lines with kappa 12 1/m for the gates to reject (measured on
                # icra2026_mesh).
                short = keep.sum(axis=1) < args.w_veh + 0.02
                if short.any():
                    keep[short] = keep_phys[short]
                    if not quiet:
                        print(f'  gate bounds: corridor wins at '
                              f'{int(short.sum())} of {len(keep)} reference '
                              f'points (bound and sweep do not intersect)')
        shrink = float(np.mean(reftrack[:, 2:4] - keep))
        work = np.column_stack([reftrack[:, :2], keep])
        if not quiet:
            print(f'  sweep pass {it+1}: corridor shrunk by {shrink:.3f} m '
                  f'(max sweep {sweep.max():.3f} m)')
    return alpha, work


def solve_dkappa(reftrack, normvec, M, args):
    """The --dkappa-max pipeline: constrained QP, then measured refinement.

    Stage 1 is the min-curvature QP with the curvature-rate rows in it
    (`opt_min_curv_dkappa`), which gets the shape roughly right and keeps the
    constraint where it belongs. Stage 2 (`refine_dkappa`) is what actually
    delivers the bound, because the QP only ever sees its own linearization.
    Both respect the corridor the body-sweep passes left.

    Returns the SMOOTHED alpha -- the refinement works on the finished line, so
    re-smoothing afterwards would undo part of the answer.
    """
    target = args.dkappa_max
    alpha = work = None
    # The in-QP rate rows are OFF by default and this is measured, not taste:
    # they bound the linearization at the reference points, and on Porto the
    # finished lines they produce are worse on every axis than refining the
    # plain min-curvature solution (at target 1.0: peak |kappa| 1.02 against
    # 0.83, windowed rate 1.43 against 1.00). They are kept because on a track
    # whose corridor forces the shape they are the only way to state the bound
    # before the shape is chosen.
    for ask in ((target, 1.5 * target, 2.5 * target, None)
                if args.dkappa_in_qp else (None,)):
        if ask is None:
            alpha, work = sweep_solve(reftrack, normvec, M, args)
            if args.dkappa_in_qp:
                print('  dkappa stage 1: no feasible in-QP bound; refining '
                      'the plain min-curvature solution')
            break
        try:
            alpha, work = sweep_solve(reftrack, normvec, M, args, ask,
                                      quiet=True)
            print(f'  dkappa stage 1: in-QP rate bound {ask:.2f} 1/m2 solved')
            break
        except (ValueError, RuntimeError):
            continue

    corridor = corridor_of(reftrack, normvec, work, args)
    alpha = smooth_alpha(alpha, args.alpha_smooth, corridor)
    before, raw0, *_ = line_rate(reftrack, normvec, alpha, args)
    _, _, E_kappa, _ = min_curv_matrices(work, normvec, M)
    alpha, got, raw = refine_dkappa(reftrack, normvec, E_kappa, alpha,
                                    corridor, target, args)
    note = '' if got <= target * 1.02 else '  *** TARGET NOT MET ***'
    print(f'curvature-rate limit {target:.2f} 1/m2 over '
          f'{args.dkappa_window_min:.2f}-{args.dkappa_window:.2f} m windows: '
          f'{before:.3f} -> {got:.3f} 1/m2 (raw per-sample {raw0:.2f} -> '
          f'{raw:.2f}){note}')
    return alpha, work


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('centerline')
    ap.add_argument('--out', default=None)
    ap.add_argument('--vmax', type=float, default=8.0)
    ap.add_argument('--ay', type=float, default=3.0)
    ap.add_argument('--ax', type=float, default=3.0)
    ap.add_argument('--brake', type=float, default=4.0)
    ap.add_argument('--w-veh', type=float, default=0.16,
                    help='extra safety on top of the swept footprint')
    ap.add_argument('--kappa-bound', type=float, default=1.6)
    ap.add_argument('--side-margin', default=None, metavar='SPEC',
                    help='OPT-IN (off by default, outputs byte-identical '
                         'without it): shrink the corridor on ONE side over '
                         'arc-length windows of the INPUT CENTERLINE, before '
                         'the QP sees it, so the optimiser itself trades '
                         'corner radius against the margin. '
                         'SPEC is a comma-separated list of '
                         '"s0:s1:side:metres", side = left|right relative to '
                         'travel; a window with s0 > s1 wraps through s = 0. '
                         'Overlapping windows on the same side take the '
                         'largest margin. Needs --min-halfwidth, which is '
                         'where the one-sided floor is applied (the reported '
                         'width columns are shrunk too, but on a track where '
                         'the corridor bound is slack that alone does '
                         'nothing). See --side-margin-taper.')
    ap.add_argument('--side-margin-taper', type=float, default=1.0,
                    metavar='M',
                    help='length of the smoothstep ramp on each side of a '
                         '--side-margin window, in metres (default 1.0). The '
                         'window itself carries the full margin; the ramp '
                         'lives OUTSIDE it, so "19.8:24" is at full strength '
                         'from 19.8 to 24 and fades to zero by 18.8 / 25.')
    ap.add_argument('--min-halfwidth', type=float, default=None, metavar='M',
                    help='OPT-IN (off by default, outputs byte-identical '
                         'without it): keep the PATH at least M metres from '
                         'either wall, enforced in the QP corridor, in the '
                         'alpha smoothing AND in the --dkappa-max refinement. '
                         'Car half-width is 0.135 m, so M = 0.50 leaves '
                         '0.365 m of body clearance on the pinch. Collapses '
                         'to the widest reachable point where the track is '
                         'narrower than 2M. Needs --min-halfwidth-map.')
    ap.add_argument('--checkpoints', default=None, metavar='CSV',
                    help='OPT-IN. <map>_checkpoints.csv from map_from_mesh.py. '
                         'The line is then constrained to pass inside every '
                         'lap-checkpoint trigger box, so it cannot skip one '
                         'and run a lap the sim refuses to count.')
    ap.add_argument('--gate-separation', type=float, default=None, metavar='M',
                    help='OPT-IN, needs --checkpoints. Hold consecutive lap-'
                         'checkpoint planes at least M metres apart along the '
                         'line (0.60 = body 0.50 + trigger box 0.10), so the '
                         'car is never inside two trigger boxes at once and '
                         'cannot drop a checkpoint. Where the geometry cannot '
                         'reach M the gates are pushed as far apart as they go.')
    ap.add_argument('--gate-band-floor', type=float, default=0.12, metavar='M',
                    help='narrowest lateral band --gate-separation may impose '
                         'at a gate before the solver widens it (default '
                         '0.12 m). Only used with --gate-separation.')
    ap.add_argument('--checkpoint-margin', type=float, default=0.25,
                    metavar='M',
                    help='metres of the trigger half-width to keep in hand at '
                         'each checkpoint (default 0.25). Only used with '
                         '--checkpoints.')
    ap.add_argument('--min-halfwidth-map', default=None, metavar='PREFIX',
                    help='map the --min-halfwidth clearance is measured '
                         'against, e.g. data/maps/cdctf2025_mesh. Separate '
                         'from --map-prefix so the wall floor can be used '
                         'without also enabling the clearance SPEED cap.')
    ap.add_argument('--map-prefix', default=None,
                    help='enables the clearance speed cap; needs the map the '
                         'line was built from, e.g. data/maps/track_v2')
    ap.add_argument('--clearance-speed', type=float, default=16.6,
                    help='m/s allowed per metre of corner clearance. 16.6 is '
                         'the shipping profile\'s own ratio: 5.39 m/s at '
                         '0.325 m, a combination measured clean over 22 laps.')
    ap.add_argument('--jerk-lat', type=float, default=None,
                    help='cap d(ay)/dt in m/s3. Off by default. The shipping '
                         'ay-3.0 profile peaks at 21 m/s3 and runs clean, so '
                         'that is the level known to be survivable.')
    ap.add_argument('--inside-bias', type=float, default=0.0,
                    metavar='M',
                    help='centerline-only: shift the reference toward the '
                         'inside of each corner by min(M, M*|kappa|/kappa_ref) '
                         'metres. Measured with the dead-time-compensated MPC: '
                         'the car runs 0.15-0.17 m WIDE through the hairpins '
                         '(kappa > 0.6) at every ay, and every collision above '
                         'ay 5 is there. Biasing the reference inward puts the '
                         'driven path back on the corridor centre. Off by '
                         'default.')
    ap.add_argument('--bias-kappa', type=float, default=0.7,
                    help='|kappa| at which --inside-bias reaches its full value')
    ap.add_argument('--bias-kappa-start', type=float, default=0.0,
                    help='|kappa| below which the bias is zero (ramp from here '
                         'to --bias-kappa). Default 0: ramp from zero curvature.')
    ap.add_argument('--drag', type=float, default=0.0,
                    help='linear velocity damping, 1/s. The Unity rigidbody '
                         'carries drag 0.273 (read from the asset), i.e. '
                         '0.273*v of deceleration at all times: 2.2 m/s2 at '
                         '8 m/s. Off by default.')
    ap.add_argument('--ax-servo', type=float, default=None,
                    help='m/s2 the throttle servo can add before drag. '
                         'Measured ~6 in both directions. With --drag the '
                         'drive limit becomes min(--ax, servo - drag*v).')
    ap.add_argument('--ay-tight', type=float, default=None,
                    help='lower ay cap applied where |kappa| exceeds '
                         '--kappa-tight. Every collision at ay 5.5 on Porto '
                         'was at the hairpin exit (kappa 0.7-1.0), where the '
                         'steering angle is large and the front tyres deliver '
                         'less curvature than the bicycle model promises; the '
                         'faster corners ran clean. Off by default.')
    ap.add_argument('--kappa-tight', type=float, default=0.6,
                    help='1/m; the ay cap ramps from --ay at this curvature '
                         'to --ay-tight at 1.5x it')
    ap.add_argument('--turn-in', type=float, default=0.0,
                    metavar='M',
                    help='look this many metres ahead when applying the ay '
                         'cap, so the car is already slow when a corner '
                         'tightens. 0 disables (default).')
    ap.add_argument('--alpha-smooth', type=int, default=5, metavar='W',
                    help='width of the circular boxcar applied to the QP\'s '
                         'lateral shift before the line is built. 5 (default) '
                         'is what every shipping line was generated with. '
                         'Wider trades path optimality for curvature '
                         'continuity: min-curvature bounds the INTEGRAL of '
                         'kappa and is free to spend it in one place, and on a '
                         'pixel-skeleton reference that shows up as isolated '
                         'd(kappa)/ds spikes at spline junctions -- 4.3 1/m2 '
                         'on Porto\'s mesh skeleton line against 1.7 on the '
                         'shipping safe line. Above 5 the smoothed shift is '
                         'clipped back inside the corridor the QP solved.')
    ap.add_argument('--dkappa-max', type=float, default=None, metavar='K',
                    help='cap how fast curvature may change along the line, '
                         'in 1/m2, as a hard linear constraint inside the '
                         'min-curvature QP (opt-in; off by default, and the '
                         'shipping lines are byte-identical without it). The '
                         'steering actuator slews at 3.2 rad/s behind 0.135 s '
                         'of dead time and holds a command for ~0.09-0.10 s, '
                         'so at 4.5 m/s the car gets one command per 0.45 m: a '
                         'kappa ramp of 0.7 over a metre asks for 0.22 rad in '
                         '0.2 s. Porto\'s shipping line peaks at 1.4-1.7 1/m2 '
                         'and that peak is where every fast-rate collision is. '
                         'Ignored with --centerline-only (there is no lateral '
                         'freedom to spend).')
    ap.add_argument('--dkappa-refine', type=int, default=25,
                    help='Gauss-Newton passes that pull the FINISHED line '
                         'under --dkappa-max. The in-QP bound only holds for '
                         'the linearized curvature at the reference points; '
                         'each pass here measures the real 0.10 m line and '
                         'asks for the smallest corridor-legal correction '
                         'that fixes the remaining violations.')
    ap.add_argument('--dkappa-window', type=float, default=0.50, metavar='M',
                    help='longest window the rate bound is applied over. 0.50 '
                         'is one steering command at hairpin-entry speed '
                         '(commands are held ~0.09-0.10 s, 4.5 m/s).')
    ap.add_argument('--dkappa-window-min', type=float, default=0.20,
                    metavar='M',
                    help='shortest window the rate bound is applied over. '
                         'Below ~0.2 m the measured rate is dominated by the '
                         'spline-knot wobble and the median filter\'s '
                         'staircase, not by the line: a 0.6 mm change of the '
                         'lateral shift moves it by 1.0 1/m2.')
    ap.add_argument('--dkappa-step', type=float, default=0.05, metavar='M',
                    help='trust region on one refinement pass, in metres of '
                         'lateral shift.')
    ap.add_argument('--dkappa-reg', type=float, default=1e-4,
                    help='ridge on the refinement step, metres^-2.')
    ap.add_argument('--dkappa-smooth-reg', type=float, default=1e-3,
                    help='penalty on the second difference of the refinement '
                         'step, which is what keeps it inside what the spline '
                         'can actually express.')
    ap.add_argument('--dkappa-in-qp', action='store_true',
                    help='also put the rate bound in the min-curvature QP as '
                         'a linear constraint on the linearized curvature at '
                         'the reference points. Off by default: measured on '
                         'Porto it makes the FINISHED line worse than refining '
                         'the plain min-curvature solution.')
    ap.add_argument('--stepsize', type=float, default=0.10)
    ap.add_argument('--sweep-iters', type=int, default=2,
                    help='corridor shrink / re-optimize passes for body sweep')
    ap.add_argument('--centerline-only', action='store_true',
                    help='no lateral optimization: follow the centerline with a '
                         'curvature-limited speed profile. Safest line when the '
                         'corridor is too narrow to optimize into.')
    args = ap.parse_args()
    out = args.out or args.centerline.replace('_centerline.csv', '_raceline')

    if args.min_halfwidth is not None and not args.min_halfwidth_map:
        sys.exit('--min-halfwidth needs --min-halfwidth-map')
    if args.checkpoint_margin < 0.0:
        sys.exit('--checkpoint-margin must not be negative')
    if args.gate_separation is not None:
        if args.checkpoints is None:
            sys.exit('--gate-separation needs --checkpoints')
        if args.gate_separation <= 0.0:
            sys.exit('--gate-separation must be positive')
    reftrack = np.loadtxt(args.centerline, delimiter=',')
    print(f'centerline: {len(reftrack)} pts')

    # OPT-IN, September 19. Shrink one corridor side over s windows BEFORE
    # anything else reads the widths, so the min-curvature QP trades corner
    # radius against the margin itself instead of a hand shift bending the
    # finished line (which costs 0.3-0.9 s of ideal lap in added S-bend
    # curvature). Default None -> byte-identical output.
    if args.side_margin:
        if args.min_halfwidth is None:
            sys.exit('--side-margin needs --min-halfwidth (and '
                     '--min-halfwidth-map): the one-sided floor is applied '
                     'against the map, in wall_bounds')
        parse_side_margin(args.side_margin)      # fail fast on a bad spec
        reftrack = apply_side_margin(reftrack, args.side_margin,
                                     args.side_margin_taper)

    # Centerline is already spline-smoothed/uniform; thin to ~0.2 m spacing
    # for a tractable QP (tph.spline_approximation is incompatible with
    # modern scipy).
    step = max(1, len(reftrack) // 160)
    reftrack = reftrack[::step]
    print(f'regularized: {len(reftrack)} pts')

    refline_cl = np.vstack((reftrack[:, :2], reftrack[0, :2]))
    coeffs_x, coeffs_y, M, normvec = tph.calc_splines.calc_splines(
        path=refline_cl, use_dist_scaling=False)

    presmoothed = False
    if args.centerline_only:
        alpha = np.zeros(len(reftrack))
        print('centerline-only: no lateral shift')
        if args.inside_bias != 0.0:
            # Curvature of the reference itself (sign gives the corner side).
            # Verified numerically: positive alpha shifts RIGHT of travel and
            # positive kappa is a LEFT turn, so the inside is -sign(kappa).
            k_ref = curvature_of(reftrack, normvec, alpha, args.stepsize)
            k_ref = ndimage.uniform_filter1d(k_ref, size=7, mode='wrap')
            k0, k1 = args.bias_kappa_start, args.bias_kappa
            mag = np.clip((np.abs(k_ref) - k0) / max(k1 - k0, 1e-6), 0.0, 1.0)
            alpha = -np.sign(k_ref) * args.inside_bias * mag
            alpha = ndimage.uniform_filter1d(alpha, size=5, mode='wrap')
            # never leave the corridor the centerline reports (w_right, w_left)
            alpha = np.clip(alpha, -reftrack[:, 3] + 0.05, reftrack[:, 2] - 0.05)
            print(f'inside bias up to {args.inside_bias:.2f} m at |kappa| >= '
                  f'{args.bias_kappa:.2f}: max shift {np.abs(alpha).max():.3f} m')
    elif args.dkappa_max is not None:
        alpha, work = solve_dkappa(reftrack, normvec, M, args)
        presmoothed = True
    else:
        alpha, work = sweep_solve(reftrack, normvec, M, args)

    # Light circular smoothing of the lateral shift: isolated single-point
    # solutions show up as curvature spikes that crater the velocity profile.
    #
    # A wider window moves the solution further, so a wider window stops
    # trusting the measured-zero-violations result the default window earns
    # and makes the corridor a hard constraint again -- see `smooth_alpha`,
    # which is this code, factored out so the --dkappa-max outer loop can
    # measure what the smoothing actually produces.
    win = args.alpha_smooth
    alpha_raw = alpha.copy()
    if not presmoothed:
        alpha = smooth_alpha(alpha, win,
                             None if args.centerline_only
                             else corridor_of(reftrack, normvec,
                                              work, args))

    # Smoothing a CONSTRAINED solution can push it back outside the constraint,
    # and nothing downstream re-checked that. Measured on Porto at w_veh 0.44:
    # smoothing moves alpha by at most 0.019 m and leaves zero points outside
    # the corridor the solver was given, so it is safe as written -- but that
    # is a fact about this track, not a guarantee, so the check stays.
    #
    # The bound is `work`, the corridor AFTER the body-sweep shrink, with no
    # further w_veh/2 taken off: `sweep` already contains the body half-width,
    # so subtracting it twice would be double-counting.
    if not args.centerline_only:
        c_lo, c_hi = corridor_of(reftrack, normvec, work, args)
        viol = np.maximum(np.maximum(c_lo - alpha, alpha - c_hi), 0.0)
        n_v = int((viol > 1e-6).sum())
        moved = float(np.abs(alpha - alpha_raw).max())
        msg = (f'  alpha smoothing: moved at most {moved:.3f} m, '
               f'{n_v} points outside the solved corridor')
        if n_v:
            msg += f' -- WARNING, worst by {viol.max():.3f} m'
        print(msg)

    raceline, _, coeffs_x_r, coeffs_y_r, spline_inds, t_vals, s_race, \
        spline_lengths, el_lengths = tph.create_raceline.create_raceline(
            refline=reftrack[:, :2], normvectors=normvec, alpha=alpha,
            stepsize_interp=args.stepsize)
    psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x_r, coeffs_y=coeffs_y_r,
        ind_spls=spline_inds, t_spls=t_vals)

    # Spline junctions leave isolated few-cm wiggles whose analytic curvature
    # spikes; a real corner spans many points, so a circular median filter
    # removes the artifacts without touching genuine corners. Unfiltered these
    # spikes crater the velocity profile (they cost ~6 s/lap here).
    kappa_raw = kappa.copy()
    kappa = ndimage.median_filter(kappa, size=5, mode='wrap')
    n_fixed = int(np.sum(np.abs(kappa_raw - kappa) > 1.0))
    print(f'raceline: {len(raceline)} pts, length {s_race[-1]:.1f} m, '
          f'|kappa| max {np.abs(kappa).max():.2f} 1/m '
          f'(min radius {1/max(np.abs(kappa).max(), 1e-6):.2f} m); '
          f'{n_fixed} curvature spikes filtered')

    # Velocity profile from a simple friction-circle ggv
    v_steps = np.arange(0.0, args.vmax + 0.5, 0.5)
    ggv = np.column_stack([v_steps,
                           np.full_like(v_steps, args.ax),
                           np.full_like(v_steps, args.ay)])
    ax_drive = np.full_like(v_steps, args.ax)
    if args.ax_servo is not None:
        ax_drive = np.minimum(ax_drive,
                              np.maximum(args.ax_servo - args.drag * v_steps, 0.3))
        print(f'drive limit servo {args.ax_servo:.1f} - drag {args.drag:.3f}*v: '
              f'{ax_drive[-1]:.2f} m/s2 at {v_steps[-1]:.1f} m/s')
    ax_machines = np.column_stack([v_steps, ax_drive])
    kappa_cap = turn_in_kappa(kappa, el_lengths, args.turn_in)
    if args.turn_in > 0.0:
        n_early = int(np.sum(np.abs(kappa_cap) > np.abs(kappa) + 1e-9))
        print(f'turn-in lookahead {args.turn_in:.2f} m: ay cap raised early at '
              f'{n_early} of {len(kappa)} points')
    vx = tph.calc_vel_profile.calc_vel_profile(
        ggv=ggv, ax_max_machines=ax_machines, v_max=args.vmax,
        kappa=kappa_cap, el_lengths=el_lengths, closed=True,
        drag_coeff=0.0, m_veh=VEHICLE_MASS)
    # Braking-aware acceleration profile
    vx_cl = np.append(vx, vx[0])
    ax_prof = tph.calc_ax_profile.calc_ax_profile(
        vx_profile=vx_cl, el_lengths=el_lengths, eq_length_output=False)

    if args.ay_tight is not None and args.ay_tight < args.ay:
        k = np.abs(kappa_cap)
        k0, k1 = args.kappa_tight, 1.5 * args.kappa_tight
        frac = np.clip((k - k0) / max(k1 - k0, 1e-6), 0.0, 1.0)
        ay_pt = args.ay + (args.ay_tight - args.ay) * frac
        v_cap = np.sqrt(ay_pt / np.maximum(k, 1e-6))
        n_cut = int(np.sum(v_cap < vx - 1e-6))
        if n_cut:      # a no-op must stay byte-identical (Porto never binds)
            vx = limit_long_accel(np.minimum(vx, v_cap), el_lengths, args.ax,
                                  args.brake)
        print(f'tight-corner cap ay {args.ay_tight:.1f} above kappa '
              f'{k0:.2f}: speed limited at {n_cut} of {len(vx)} points')

    if args.map_prefix:
        cap, clear = clearance_speed_cap(raceline, kappa, args.map_prefix,
                                         args.clearance_speed)
        bound = np.minimum(vx, cap)
        n_cut = int(np.sum(bound < vx - 1e-6))
        if n_cut:
            vx = limit_long_accel(bound, el_lengths, args.ax, args.brake)
        print(f'clearance cap: min clearance {clear.min():.3f} m, '
              f'speed limited at {n_cut} of {len(vx)} points '
              f'(worst allows {cap.min():.2f} m/s)')

    if args.min_halfwidth is not None and not args.centerline_only:
        from footprint_analysis import load_dist, clearance as clear_at
        _d, _r, _ox, _oy = load_dist(args.min_halfwidth_map)
        _c = np.nan_to_num(clear_at(_d, _r, _ox, _oy, raceline[:, :2]), nan=0.0)
        print(f'min-halfwidth {args.min_halfwidth:.2f} m asked: finished line '
              f'holds {_c.min():.3f} m at s {s_race[int(np.argmin(_c))]:.1f}')

    if args.checkpoints is not None:
        _cps = load_checkpoints(args.checkpoints)
        _mg = checkpoint_margins(raceline[:, :2], _cps)
        _bad = [k for k, v in _mg.items() if v is None or v < 0.0]
        _ok = sorted((v, k) for k, v in _mg.items() if v is not None)
        print(f'checkpoints: min margin {_ok[0][0]:.3f} m at checkpoint '
              f'{_ok[0][1]} (asked {args.checkpoint_margin:.2f}), '
              f'median {np.median([v for v, _ in _ok]):.3f} m'
              + (f'  *** SKIPPED: {_bad} ***' if _bad else ''))
        if args.gate_separation is not None:
            _sep = gate_separations(raceline[:, :2], _cps)
            _w = min(_sep.items(), key=lambda kv: kv[1])
            _short = sorted(k for k, v in _sep.items()
                            if v < args.gate_separation - 1e-9)
            print(f'gate separation: finished line holds '
                  f'{_w[1]:.3f} m at {_w[0][0]}->{_w[0][1]} '
                  f'(asked {args.gate_separation:.2f})'
                  + (f'  *** SHORT: {_short} ***' if _short else ''))

    if args.jerk_lat is not None:
        before = float(np.sum(el_lengths / np.maximum(vx, 0.1)))
        vx = limit_lat_jerk(vx, kappa, el_lengths, args.jerk_lat,
                            args.ax, args.brake)
        after = float(np.sum(el_lengths / np.maximum(vx, 0.1)))
        ay_p = vx ** 2 * kappa
        dt_p = el_lengths / np.maximum(vx, 0.1)
        jerk = np.abs(np.diff(np.append(ay_p, ay_p[0]))) / np.maximum(dt_p, 1e-3)
        print(f'lateral jerk capped at {args.jerk_lat:.0f} m/s3: '
              f'achieved max {jerk.max():.1f}, ideal lap {before:.2f} -> {after:.2f} s')
        vx_cl = np.append(vx, vx[0])
        ax_prof = tph.calc_ax_profile.calc_ax_profile(
            vx_profile=vx_cl, el_lengths=el_lengths, eq_length_output=False)

    lap_t = float(np.sum(el_lengths / np.maximum(vx, 0.1)))
    print(f'velocity: min {vx.min():.2f} max {vx.max():.2f} m/s — '
          f'ideal lap ≈ {lap_t:.1f} s')

    header = 's_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2'
    np.savetxt(out + '.csv',
               np.column_stack([s_race[:-1] if len(s_race) > len(raceline)
                                else s_race, raceline[:, 0], raceline[:, 1],
                                psi, kappa, vx, ax_prof]),
               delimiter=',', fmt='%.4f', header=header, comments='# ')

    plt.figure(figsize=(7, 11))
    plt.plot(reftrack[:, 0], reftrack[:, 1], 'k--', lw=0.7, label='centerline')
    sc = plt.scatter(raceline[:, 0], raceline[:, 1], c=vx, cmap='viridis',
                     s=7, label='raceline')
    plt.colorbar(sc, label='v [m/s]')
    plt.axis('equal')
    plt.legend()
    plt.title(f'min-curvature raceline — ideal lap ≈ {lap_t:.1f} s')
    plt.savefig(out + '_qc.png', dpi=130, bbox_inches='tight')
    print(f'wrote {out}.csv and QC plot')


if __name__ == '__main__':
    main()
