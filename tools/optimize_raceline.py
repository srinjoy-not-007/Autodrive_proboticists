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

import numpy as np
import trajectory_planning_helpers as tph
from scipy import ndimage

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

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
            allowed = lim[i] + abs(ay[small]) if same else lim[i] - abs(ay[small])
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
    ap.add_argument('--jerk-lat', type=float, default=None,
                    help='cap d(ay)/dt in m/s3. Off by default. The shipping '
                         'ay-3.0 profile peaks at 21 m/s3 and runs clean, so '
                         'that is the level known to be survivable.')
    ap.add_argument('--stepsize', type=float, default=0.10)
    ap.add_argument('--sweep-iters', type=int, default=2,
                    help='corridor shrink / re-optimize passes for body sweep')
    ap.add_argument('--centerline-only', action='store_true',
                    help='no lateral optimization: follow the centerline with a '
                         'curvature-limited speed profile. Safest line when the '
                         'corridor is too narrow to optimize into.')
    args = ap.parse_args()
    out = args.out or args.centerline.replace('_centerline.csv', '_raceline')

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

    if args.centerline_only:
        alpha = np.zeros(len(reftrack))
        print('centerline-only: no lateral shift')
    else:
        # Iterate: optimize, measure how far the body sweeps at the resulting
        # curvature, shrink the corridor by that, re-optimize. One pass with a
        # constant w_veh cannot express this because the sweep depends on the
        # solution's own curvature.
        work = reftrack.copy()
        alpha = None
        for it in range(args.sweep_iters):
            alpha = tph.opt_min_curv.opt_min_curv(
                reftrack=work, normvectors=normvec, A=M,
                kappa_bound=args.kappa_bound, w_veh=args.w_veh,
                closed=True, print_debug=False)[0]
            k_ref = curvature_of(work, normvec, alpha, args.stepsize)
            sweep = corner_sweep(k_ref)
            # `sweep` already includes the body half-width, so the usable
            # half-corridor for the *path* is the wall distance minus it.
            keep = np.maximum(reftrack[:, 2:4] - sweep[:, None], 0.05)
            shrink = float(np.mean(reftrack[:, 2:4] - keep))
            work = np.column_stack([reftrack[:, :2], keep])
            print(f'  sweep pass {it+1}: corridor shrunk by {shrink:.3f} m '
                  f'(max sweep {sweep.max():.3f} m)')

    # Light circular smoothing of the lateral shift: isolated single-point
    # solutions show up as curvature spikes that crater the velocity profile.
    win = 5
    kern = np.ones(win) / win
    alpha = np.convolve(np.concatenate([alpha[-win:], alpha, alpha[:win]]),
                        kern, mode='same')[win:-win]

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
    ax_machines = np.column_stack([v_steps, np.full_like(v_steps, args.ax)])
    vx = tph.calc_vel_profile.calc_vel_profile(
        ggv=ggv, ax_max_machines=ax_machines, v_max=args.vmax,
        kappa=kappa, el_lengths=el_lengths, closed=True,
        drag_coeff=0.0, m_veh=3.906)
    # Braking-aware acceleration profile
    vx_cl = np.append(vx, vx[0])
    ax_prof = tph.calc_ax_profile.calc_ax_profile(
        vx_profile=vx_cl, el_lengths=el_lengths, eq_length_output=False)

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
