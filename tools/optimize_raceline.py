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
