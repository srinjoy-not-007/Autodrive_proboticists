#!/usr/bin/env python3
"""Cap a candidate raceline's SPEED to a reference line's speed inside s windows.

Why this exists: on the SRL 2026 IROS finals track every harness collision an
`ay` rung produces is at **s 37.0-37.5** (kappa ~1.0, entry to the second-last
corner), at the 0.058 s display period.  Raising planned `ay` everywhere raises
it there too, and that one corner is what fails.  Capping the profile back to
the shipping line's own speed over s 36-40 -- and nowhere else -- keeps the
whole rest of the lap at the new grip rung.  It is the same trick that turned
the ay-6.75 line's Gate C failure into `finals_mesh_hyb675_race.csv`, applied
to a different corner.

Geometry is untouched: only columns vx and ax are rewritten, then
`limit_long_accel` makes the result reachable again.

Usage:
  scripts/win_stack_blend.py <cand.csv> <ref.csv> <out.csv> "36.0:40.0" [taper]

Lives in scripts/, not tools/, deliberately: it is a branch experiment and
`scripts/check_pipeline.sh` guards tools/.
"""
import sys

import numpy as np

sys.path.insert(0, 'tools')
import trajectory_planning_helpers as tph                        # noqa: E402
from optimize_raceline import limit_long_accel                   # noqa: E402

AX, BRAKE = 6.5, 4.6      # the profile the win-stack ladder is built at


def main():
    cand, ref, out = sys.argv[1], sys.argv[2], sys.argv[3]
    wins = [tuple(float(x) for x in w.split(':')) for w in sys.argv[4].split(',')]
    taper = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0

    c = np.loadtxt(cand, delimiter=',')
    r = np.loadtxt(ref, delimiter=',')
    s, v = c[:, 0], c[:, 5].copy()
    vref = np.interp(s, r[:, 0], r[:, 5])
    w = np.zeros_like(s)
    for lo, hi in wins:
        w = np.maximum(w, np.clip((s - (lo - taper)) / taper, 0, 1)
                       * np.clip(((hi + taper) - s) / taper, 0, 1))
    # Outside the window the cap is +inf, so this only ever REDUCES speed.
    v = np.minimum(v, w * vref + (1 - w) * 1e3)
    el = np.diff(np.r_[s, s[-1] + (s[-1] - s[-2])])
    v = limit_long_accel(v, el, AX, BRAKE)
    axp = tph.calc_ax_profile.calc_ax_profile(
        vx_profile=np.append(v, v[0]), el_lengths=el, eq_length_output=False)
    c[:, 5] = v
    c[:, 6] = axp[:len(s)]
    np.savetxt(out, c, delimiter=',', fmt='%.4f')
    print(f'{out}: ideal {np.sum(el / v):.3f} s  v {v.min():.2f}-{v.max():.2f}  '
          f'ay_pk {np.max(v * v * np.abs(c[:, 4])):.2f}')


if __name__ == '__main__':
    main()
