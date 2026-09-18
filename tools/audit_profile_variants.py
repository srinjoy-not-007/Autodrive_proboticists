#!/usr/bin/env python3
"""Build `finals_mesh_audit_*_race.csv`: the shipping geometry, a new profile.

Geometry (s, x, y, psi, kappa) is copied BYTE-FOR-BYTE from
`finals_mesh_hyb675b_race.csv`; only `vx`/`ax` move, and never inside the
protected zones -- the three hairpins plus the start, frozen at the shipping
speeds because every live attempt to raise them collided.

WHY THIS IS NOT A REPROFILE. `finals_mesh_hyb675b_race.csv` is not the output
of a two-pass velocity profile: it is an ay-6.75 profile with the frozen
ay-6.0 speeds blended back over two windows, an explicit 3.0 m/s exit cap at
s 38.3-39.8, and the lateral-jerk limiter's split budget through all of it.
Re-solving it at a new envelope gives a line ~1.0 s/lap of "ideal" faster than
the one that was actually driven, i.e. a different line -- measured: the
two-pass envelope through the same corner speeds is 9.85 s against the base's
10.84 s. So each variant is built by RELAXING EXACTLY ONE CONSTRAINT of the
base profile, in place:

  brake  the backward braking ramps are recomputed at the new deceleration
         and capped at the speed the base itself was carrying when each ramp
         began (the "plateau"). Nothing outside a braking ramp moves, so the
         corner speeds, the jerk limiting and the frozen zones are untouched.
  ax     the forward acceleration ramps are re-limited at the new (lower)
         acceleration. This can only LOWER speeds -- it is the honest-plant
         direction, since the measured achieved a_x is ~4.0 median / 5.1 p90,
         not the planned 6.5.
  vmax   the flat-out plateau (|kappa| < 0.05 and v0 at the cap) is released
         to a higher cap, then re-limited forward at ax and backward at brake.

Then, on every variant: the protected zones are clamped back to the shipping
speed, and `learn_profile.guard` sheds any raise at 3.2 m/s^2 before the next
un-raised bin -- the LMPC lesson that a per-segment raise arrives as entry
speed one corner later (`lmpc_a/b/d/e` all collided at s 37.5 that way).
"""
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from learn_profile import guard                                # noqa: E402

BASE = os.path.join(ROOT, 'data/racelines/finals_mesh_hyb675b_race.csv')
OUT = os.path.join(ROOT, 'data/racelines')

# s windows held at the shipping speeds (FINALS_TUNING sections 3 and 4b).
PROTECTED = [(17.5, 21.5), (36.0, 41.5), (42.5, 44.9), (0.0, 2.0)]

# name -> (ax, brake, vmax); the base's own envelope is ax 6.5 / brake 4.0 /
# vmax 8.0, so a value equal to those is "unchanged".
VARIANTS = {
    'brk46':   (6.5, 4.6, 8.0, 4.6),  # measured held-brake ceiling (4.55)
    'brk50':   (6.5, 5.0, 8.0, 5.0),  # top of measured median band (4.65-5.09)
    'ax50b46': (5.0, 4.6, 8.0, 4.6),  # honest plant: measured a_x p90 ~5.1
    'v84b46':  (6.5, 4.6, 8.4, 3.2),  # the one place the car reaches plan
    'brk50v84': (6.5, 5.0, 8.4, 5.0),  # both raises together
}
BASE_AX, BASE_BRAKE, BASE_VMAX = 6.5, 4.0, 8.0
AY_CAP = 6.75          # the ay base the hybrid was generated at


def brake_envelope(v_end, el, brake, passes=6):
    """Fastest speed at each node that still meets every downstream `v_end`.

    Cyclic backward recursion that never injects the node's OWN base speed, so
    a node on a braking ramp is free to rise; a node whose own speed is the
    binding one is re-clamped by the caller.
    """
    n = len(v_end)
    b = np.full(n, 1e3)
    for _ in range(passes):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            b[i] = np.sqrt(min(v_end[j], b[j]) ** 2 + 2.0 * brake * el[i])
    return b


def accel_envelope(v_start, el, ax, passes=6):
    """Mirror of `brake_envelope`, forward: reachability at acceleration `ax`."""
    n = len(v_start)
    f = np.full(n, 1e3)
    for _ in range(passes):
        for i in range(n):
            j = (i - 1) % n
            f[i] = np.sqrt(min(v_start[j], f[j]) ** 2 + 2.0 * ax * el[j])
    return f


def plateau_cap(v0, at_limit):
    """For each node braking AT THE PLAN LIMIT, the speed where that run began.

    A relaxed brake may carry more speed into a corner, but never more than the
    base was already planning upstream of the braking run -- otherwise the
    change stops being "brake later" and becomes "go faster", which is a
    different experiment with different evidence behind it.
    """
    n = len(v0)
    cap = v0.copy()
    for _ in range(3):
        for i in range(n):
            if at_limit[i]:
                cap[i] = max(cap[i], cap[(i - 1) % n])
    return cap


def main():
    r = np.loadtxt(BASE, delimiter=',')
    s, x, y, psi, kappa, v0, ax_col = (r[:, i] for i in range(7))
    el = np.diff(np.r_[s, s[-1] + (s[-1] - s[-2])])
    prot = np.zeros(len(s), bool)
    for lo, hi in PROTECTED:
        prot |= (s >= lo) & (s <= hi)
    ideal0 = float(np.sum(el / v0))
    ay0 = float(np.max(v0 ** 2 * np.abs(kappa)))
    print(f'base {os.path.basename(BASE)}: ideal {ideal0:.3f} s  '
          f'v {v0.min():.2f}-{v0.max():.2f}  peak ay {ay0:.2f}  '
          f'protected {prot.mean()*100:.0f}% of nodes')

    # control: relaxing nothing must reproduce the base exactly
    ctrl = build(v0, ax_col, kappa, el, prot, BASE_AX, BASE_BRAKE, BASE_VMAX, 3.2)
    print(f'  control (ax {BASE_AX} brake {BASE_BRAKE} vmax {BASE_VMAX}): '
          f'ideal {np.sum(el / ctrl):.3f}  max |dv| {np.abs(ctrl - v0).max():.4f}')

    for name, (ax, brake, vmax, g) in VARIANTS.items():
        v = build(v0, ax_col, kappa, el, prot, ax, brake, vmax, g)
        vv = np.r_[v, v[0]]
        ax_prof = (vv[1:] ** 2 - vv[:-1] ** 2) / (2 * el)
        stem = os.path.join(OUT, f'finals_mesh_audit_{name}_race')
        np.savetxt(stem + '.csv',
                   np.column_stack([s, x, y, psi, kappa, v, ax_prof]),
                   delimiter=',', fmt='%.4f',
                   header='s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2',
                   comments='# ')
        a = np.loadtxt(stem + '.csv', delimiter=',')
        assert np.array_equal(a[:, :5], r[:, :5]), 'geometry moved: ' + name
        assert np.all(v[prot] <= v0[prot] + 1e-9), 'protected zone raised: ' + name
        ideal = float(np.sum(el / v))
        print(f'audit_{name:<8} ax {ax:<4} brake {brake:<4} vmax {vmax:<4} g {g:<4} '
              f'ideal {ideal:6.3f} ({ideal - ideal0:+.3f})  '
              f'v {v.min():.2f}-{v.max():.2f}  '
              f'moved {np.mean(np.abs(v / v0 - 1) > 0.005)*100:4.0f}%  '
              f'raised {np.mean(v > v0 + 1e-3)*100:4.0f}%  '
              f'peak ay {np.max(v ** 2 * np.abs(kappa)):.2f}  '
              f'peak jerk_lat {jerk_lat(v, kappa, el):.1f} (base '
              f'{jerk_lat(v0, kappa, el):.1f})')


def jerk_lat(v, kappa, el):
    """Peak |d(ay)/dt| along the path, the quantity `--jerk-lat` caps."""
    ay = v ** 2 * kappa
    dt = el / np.maximum(v, 1e-3)
    return float(np.max(np.abs(np.diff(np.r_[ay, ay[0]]) / dt)))


def build(v0, ax_col, kappa, el, prot, ax, brake, vmax, guard_brake=3.2):
    """Relax exactly one PROFILE constant of the base, nothing else.

    The base is only re-solved where its own `ax` column says the constant
    being changed was the BINDING constraint: `ax_col <= -3.95` is where
    `--brake 4.0` binds (121 of 449 nodes), `ax_col >= 6.35` is where
    `--ax 6.5` binds (16 nodes), and the flat `v0 == vmax` run is where
    `--vmax 8.0` binds. Everywhere else the base's speed is a hard ceiling, so
    the lateral-jerk limiter's split budget, the ay-6.0 blend windows and the
    s 38.3-39.8 exit cap are all preserved untouched -- which is what makes
    the control (relax nothing) reproduce the base to 1e-4 m/s.
    """
    at_brake = ax_col <= -(BASE_BRAKE - 0.05)
    at_accel = ax_col >= (BASE_AX - 0.15)
    ceil = v0.copy()
    if vmax > BASE_VMAX:
        flat = (np.abs(kappa) < 0.05) & (v0 >= BASE_VMAX - 1e-6)
        ceil = np.where(flat, vmax, ceil)
    # braking runs: free to rise, bounded by the pre-run plateau
    cap = np.where(at_brake, np.maximum(ceil, plateau_cap(ceil, at_brake)), ceil)
    cap = np.minimum(cap, vmax)
    v = np.minimum(cap, brake_envelope(np.where(at_brake, 1e3, ceil), el, brake))
    # acceleration runs: re-limited at the variant's ax
    v = np.minimum(v, accel_envelope(np.where(at_accel, 1e3, v), el,
                                     min(ax, BASE_AX)))
    if ax < BASE_AX:
        v = np.minimum(v, accel_envelope(np.where(at_accel, 1e3, v0), el, ax))
    # the physics ceiling the base itself respects, and the frozen zones
    v = np.minimum(v, np.sqrt(AY_CAP / np.maximum(np.abs(kappa), 1e-4)))
    v = np.minimum(v, np.where(prot, v0, np.inf))
    v = guard(v, np.minimum(v, v0), el, guard_brake)
    v = np.minimum(v, np.where(prot, v0, np.inf))
    return np.maximum(v, 0.5)


if __name__ == '__main__':
    main()
