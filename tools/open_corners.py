"""Open a track's tightest corners: maximise the smallest radius, not the average.

Minimum-curvature optimization minimises the *integral* of kappa^2 and is free to
spike locally to do it. On Porto that is exactly what happens — the "optimized"
line peaks at kappa 1.72 where the plain centerline peaks at 1.02, and it is the
peak that hurts: corner speed is sqrt(ay / kappa_max), and the measured grip
curve says the front tyres saturate past ~0.15 rad of steering, which
tan(0.15)/0.324 = 0.46 1/m of planned curvature already exceeds.

So this optimises the other objective. Curvature-weighted Laplacian relaxation
pulls the path straighter exactly where it is tightest, and every iteration is
projected back inside the corridor, so clearance is a hard constraint rather
than something traded away. It is a greedy approximation to minimax curvature,
chosen over a proper solver because it has no dependency, cannot fail to
converge, and can be inspected step by step.

Output is a centerline-shaped CSV (x, y, w_right, w_left) with the widths
re-referenced to the new path, so it drops straight into the existing pipeline:

  .venv/bin/python tools/open_corners.py data/racelines/<t>_centerline.csv \\
      --out data/racelines/<t>_opened
  .venv/bin/python tools/optimize_raceline.py data/racelines/<t>_opened.csv \\
      --centerline-only --ay 3.5 --jerk-lat 20 --out data/racelines/<t>_openline
"""

import argparse

import numpy as np


def normals(xy):
    d = np.gradient(xy, axis=0)
    t = d / np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
    return np.column_stack([-t[:, 1], t[:, 0]])          # left-hand normal


def curvature(xy):
    dx, dy = np.gradient(xy[:, 0]), np.gradient(xy[:, 1])
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    return (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)


def open_corners(cl, margin, iters, lam, focus, smooth):
    """cl: (n,4) x, y, w_right, w_left. Returns lateral offset per point."""
    c, wr, wl = cl[:, :2], cl[:, 2], cl[:, 3]
    n = normals(c)
    # Usable lateral room, positive towards the left-hand normal.
    hi = np.maximum(wl - margin, 0.0)
    lo = -np.maximum(wr - margin, 0.0)
    alpha = np.zeros(len(c))

    for _ in range(iters):
        p = c + alpha[:, None] * n
        k = np.abs(curvature(p))
        # Weight the relaxation towards the tightest points: that is what makes
        # this minimax-ish rather than just another smoother.
        w = (k / max(k.max(), 1e-9)) ** focus
        lap = np.roll(p, 1, axis=0) + np.roll(p, -1, axis=0) - 2.0 * p
        q = p + lam * w[:, None] * lap
        alpha = np.clip(np.einsum('ij,ij->i', q - c, n), lo, hi)
        # Clipping against the corridor leaves a kink wherever the offset
        # saturates, and a kink in the offset is a curvature spike in the path —
        # which is precisely what this is supposed to remove. Smooth the offset
        # profile itself so the path stays continuous where it hugs the wall.
        alpha = np.convolve(np.r_[alpha[-smooth:], alpha, alpha[:smooth]],
                            np.ones(2 * smooth + 1) / (2 * smooth + 1),
                            mode='same')[smooth:-smooth]
        alpha = np.clip(alpha, lo, hi)
    return alpha, n


def minimax_curvature(cl, margin, n_modes, beta, penalty, iters):
    """Directly minimise the largest curvature, subject to staying in the corridor.

    Laplacian relaxation cannot do this: smoothing a closed path shortens it, so
    it cuts corners and *tightens* the peak it was meant to open — measured at
    -24% to -61% minimum radius on Porto, every configuration worse than the
    centerline it started from. Length is the wrong objective; the peak is what
    sets corner speed.

    So optimise the peak. The lateral offset is written as a truncated Fourier
    series around the lap, which makes it periodic and smooth by construction —
    no clipping kinks, and 2*n_modes+1 variables instead of one per point. The
    objective is a log-sum-exp softmax of kappa^2 (a differentiable stand-in for
    the max) plus a quadratic penalty for leaving the corridor.
    """
    from scipy.optimize import minimize

    c, wr, wl = cl[:, :2], cl[:, 2], cl[:, 3]
    nvec = normals(c)
    hi = np.maximum(wl - margin, 0.0)
    lo = -np.maximum(wr - margin, 0.0)
    n = len(c)
    theta = 2.0 * np.pi * np.arange(n) / n
    # basis: [1, cos t, sin t, cos 2t, sin 2t, ...]
    basis = [np.ones(n)]
    for m in range(1, n_modes + 1):
        basis += [np.cos(m * theta), np.sin(m * theta)]
    B = np.column_stack(basis)

    def alpha_of(z):
        return B @ z

    def cost(z):
        a = alpha_of(z)
        k2 = curvature(c + a[:, None] * nvec) ** 2
        soft = np.log(np.sum(np.exp(beta * (k2 - k2.max())))) / beta + k2.max()
        over = np.maximum(a - hi, 0.0) ** 2 + np.maximum(lo - a, 0.0) ** 2
        return soft + penalty * over.sum()

    z0 = np.zeros(B.shape[1])
    res = minimize(cost, z0, method='L-BFGS-B',
                   options={'maxiter': iters, 'maxfun': iters * 20})
    return np.clip(alpha_of(res.x), lo, hi), nvec, res


def profile_time(k, ds, ay, ax, vmax, passes=2):
    """Lap time under acceleration limits, not the point-mass fiction.

    `v = sqrt(ay/kappa)` assumes the car is already at cornering speed on entry
    and back at full speed the instant the corner ends. It therefore prices a
    wider corner as pure profit and a straight as worth nothing, which is how
    the first version of this objective bought +64% of corner radius with
    0.76 m/s of top speed and came out slower overall.

    Two forward/backward sweeps over the same ax limit the velocity profile
    uses give the corner exits their real cost: a slow apex now has to be
    accelerated out of, so shaving the apex stops being free.
    """
    v = np.minimum(vmax, np.sqrt(ay / np.maximum(k, 1e-3)))
    for _ in range(passes):
        for i in range(len(v)):
            j = (i + 1) % len(v)
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * ax * ds[i]))
        for i in range(len(v) - 1, -1, -1):
            j = (i + 1) % len(v)
            v[i] = min(v[i], np.sqrt(v[j] ** 2 + 2.0 * ax * ds[i]))
    return float(np.sum(ds / np.maximum(v, 0.1)))


def min_laptime(cl, margin, n_modes, ay, vmax, penalty, iters, kappa_cap):
    """Minimise a point-mass lap time directly, over the same smooth basis.

    Neither of the simpler objectives works, and the reasons are worth keeping:

      * minimum *curvature* (what tools/optimize_raceline.py does) minimises the
        integral and is free to spike locally, so Porto's peak goes UP, 1.02 to
        1.72, and the spikes are what saturate the steering.
      * minimum *peak* curvature flattens the peak beautifully — 0.951 m to
        1.772 m of minimum radius — and then drives a 14 s lap, because nothing
        in that objective wants a straight to stay straight. It spends curvature
        everywhere to pull the maximum down, and the top speed collapses from
        5.4 to 3.0 m/s.

    Lap time is the objective that balances them: corner radius only matters
    where the car is slow because of it, and straights are worth keeping because
    that is where the speed is. v = min(vmax, sqrt(ay/kappa)) is a point-mass
    stand-in that ignores how fast the car can actually pick the speed up, so
    treat the number it reports as a shape score, not a lap time — the real
    profile comes from optimize_raceline.py afterwards.
    """
    from scipy.optimize import minimize

    c, wr, wl = cl[:, :2], cl[:, 2], cl[:, 3]
    nvec = normals(c)
    hi = np.maximum(wl - margin, 0.0)
    lo = -np.maximum(wr - margin, 0.0)
    n = len(c)
    theta = 2.0 * np.pi * np.arange(n) / n
    basis = [np.ones(n)]
    for m in range(1, n_modes + 1):
        basis += [np.cos(m * theta), np.sin(m * theta)]
    B = np.column_stack(basis)

    def cost(z):
        a = B @ z
        p = c + a[:, None] * nvec
        k = np.abs(curvature(p))
        ds = np.linalg.norm(np.diff(np.vstack([p, p[:1]]), axis=0), axis=1)
        over = np.maximum(a - hi, 0.0) ** 2 + np.maximum(lo - a, 0.0) ** 2
        # A point mass is happy to take a 0.3 m corner slowly; this car cannot
        # turn tighter than its steering lock, and long before that the front
        # tyres stop delivering what is asked (measured: ratio 1.00 below
        # 0.10 rad, 0.42 at full lock). Without this the optimiser "wins" by
        # planning corners the car physically cannot follow.
        tight = np.maximum(k - kappa_cap, 0.0) ** 2
        return (profile_time(k, ds, ay, ay, vmax)
                + penalty * float(over.sum() + tight.sum()))

    res = minimize(cost, np.zeros(B.shape[1]), method='L-BFGS-B',
                   options={'maxiter': iters, 'maxfun': iters * 20})
    return np.clip(B @ res.x, lo, hi), nvec, res


def report(name, xy):
    k = np.abs(curvature(xy))
    seg = np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1)
    print(f'  {name:22s} length {seg.sum():6.2f} m  kappa_max {k.max():5.3f} '
          f'(R_min {1.0 / max(k.max(), 1e-9):5.3f} m)  kappa_p99 {np.percentile(k, 99):5.3f}')
    return k.max()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('centerline')
    ap.add_argument('--out', required=True)
    ap.add_argument('--margin', type=float, default=0.36,
                    help='corridor to leave unused per side: half the body plus '
                         'the measured lateral localization error')
    ap.add_argument('--iters', type=int, default=400)
    ap.add_argument('--lam', type=float, default=0.35)
    ap.add_argument('--smooth', type=int, default=5,
                    help='half-window used to keep the offset profile smooth')
    ap.add_argument('--method', choices=('laptime', 'minimax', 'relax'),
                    default='laptime',
                    help='laptime: minimise a point-mass lap time (the one that '
                         'works). minimax: optimise the largest curvature directly '
                         '- flattens the peak but kills the straights. '
                         'relax: the Laplacian version, kept only because its '
                         'failure is instructive — it shortens the path and '
                         'tightens the peak.')
    ap.add_argument('--modes', type=int, default=24)
    ap.add_argument('--ay', type=float, default=3.5, help='grip assumed while shaping')
    ap.add_argument('--vmax', type=float, default=6.0)
    ap.add_argument('--kappa-cap', type=float, default=0.46,
                    help='largest planned curvature. 0.46 1/m is 0.15 rad of '
                         'steer, where the measured grip curve says the front '
                         'tyres are still responding; 0.64 is all the car can '
                         'achieve at racing speed even at full lock.')
    ap.add_argument('--beta', type=float, default=60.0)
    ap.add_argument('--penalty', type=float, default=500.0)
    ap.add_argument('--focus', type=float, default=1.5,
                    help='how hard to concentrate on the tightest points')
    args = ap.parse_args()

    cl = np.loadtxt(args.centerline, delimiter=',')
    k0 = report('centerline', cl[:, :2])
    if args.method == 'laptime':
        alpha, n, res = min_laptime(cl, args.margin, args.modes, args.ay,
                                    args.vmax, args.penalty, args.iters,
                                    args.kappa_cap)
        print(f'  laptime: {res.nit} iters, shape score {res.fun:.2f} s')
    elif args.method == 'minimax':
        alpha, n, res = minimax_curvature(cl, args.margin, args.modes,
                                          args.beta, args.penalty, args.iters)
        print(f'  minimax: {res.nit} iterations, {"converged" if res.success else res.message}')
    else:
        alpha, n = open_corners(cl, args.margin, args.iters, args.lam,
                                args.focus, args.smooth)
    p = cl[:, :2] + alpha[:, None] * n
    k1 = report('opened', p)
    print(f'  -> R_min {1/k0:.3f} m to {1/k1:.3f} m '
          f'({(k0/k1 - 1) * 100:+.0f}% radius), max lateral shift {np.abs(alpha).max():.3f} m')

    # Re-reference the corridor to the new path: moving left eats left width.
    wl = np.maximum(cl[:, 3] - alpha, 0.0)
    wr = np.maximum(cl[:, 2] + alpha, 0.0)
    np.savetxt(args.out + '.csv', np.column_stack([p[:, 0], p[:, 1], wr, wl]),
               delimiter=',', fmt='%.4f')
    print(f'  wrote {args.out}.csv')


if __name__ == '__main__':
    main()
