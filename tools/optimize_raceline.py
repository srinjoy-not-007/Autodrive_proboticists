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
    n_pinch = int(np.sum(hi - lo < 1e-9))
    print(f'  min-halfwidth {m:.2f} m: corridor from the map, median width '
          f'{np.median(hi - lo):.3f} m, {n_pinch} of {len(lo)} reference '
          f'points cannot reach it')
    return lo, hi


def corridor_of(reftrack, normvec, work, args):
    """(lo, hi) bounds on the lateral shift alpha, as the solver sees them.

    `work` is the corridor left by the body-sweep passes and already contains
    the body half-width, so with no `--min-halfwidth` this is exactly the
    `(-work[:, 2], work[:, 3])` the code used inline before -- byte-identical.

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
    lo, hi = -work[:, 2], work[:, 3]
    if getattr(args, 'min_halfwidth', None) is None:
        return lo, hi
    if getattr(args, '_wall_bounds', None) is None:
        args._wall_bounds = wall_bounds(reftrack, normvec, args)
    lo2, hi2 = args._wall_bounds
    lo = np.maximum(lo, lo2)
    hi = np.minimum(hi, hi2)
    return np.minimum(lo, hi), hi


def sweep_solve(reftrack, normvec, M, args, dk_bound=None, quiet=False):
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
        if getattr(args, 'min_halfwidth', None) is not None:
            # Restate the wall floor as a corridor width, so the QP's own
            # `reftrack[:, 2] - w_veh/2` bound lands exactly on it.
            inf = np.full((len(reftrack), 2), np.inf)
            lo2, hi2 = corridor_of(reftrack, normvec,
                                   np.column_stack([reftrack[:, :2], inf]),
                                   args)
            keep = np.minimum(keep, np.column_stack([-lo2, hi2])
                              + args.w_veh / 2.0)
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
    ap.add_argument('--min-halfwidth', type=float, default=None, metavar='M',
                    help='OPT-IN (off by default, outputs byte-identical '
                         'without it): keep the PATH at least M metres from '
                         'either wall, enforced in the QP corridor, in the '
                         'alpha smoothing AND in the --dkappa-max refinement. '
                         'Car half-width is 0.135 m, so M = 0.50 leaves '
                         '0.365 m of body clearance on the pinch. Collapses '
                         'to the widest reachable point where the track is '
                         'narrower than 2M. Needs --min-halfwidth-map.')
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
    reftrack = np.loadtxt(args.centerline, delimiter=',')
    print(f'centerline: {len(reftrack)} pts')

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
