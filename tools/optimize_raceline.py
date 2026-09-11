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
    alpha_raw = alpha.copy()
    alpha = np.convolve(np.concatenate([alpha[-win:], alpha, alpha[:win]]),
                        kern, mode='same')[win:-win]

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
        viol = np.maximum(np.maximum(-work[:, 2] - alpha,
                                     alpha - work[:, 3]), 0.0)
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
