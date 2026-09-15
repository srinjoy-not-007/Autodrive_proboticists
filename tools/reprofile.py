#!/usr/bin/env python3
"""Recompute a raceline's SPEED profile at a new grip envelope, same geometry.

The min-curvature QP decides the PATH; `--ay/--ax/--brake/--jerk-lat` decide
only how fast that path is driven.  Re-solving the QP to change the envelope is
both slow and confounded -- a different `w_veh` or a failed solve gives a
different line, so an `ay` ladder built that way compares two things at once.

This re-profiles an existing raceline in place: identical x/y/psi/kappa, new
vx/ax.  The ladder is then a clean single-variable experiment, and on finals day
it costs seconds per rung instead of minutes.

Usage:
  reprofile.py <raceline.csv> --ay 8.0 [--ax 6.5] [--brake 4.0]
               [--jerk-lat 40] [--vmax 8.0] --out <stem>
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import trajectory_planning_helpers as tph                      # noqa: E402

from optimize_raceline import (VEHICLE_MASS, limit_lat_jerk,   # noqa: E402
                               limit_long_accel)


def reprofile(kappa, el_lengths, ay, ax, brake, jerk_lat, vmax):
    v_steps = np.arange(0.0, vmax + 0.5, 0.5)
    ggv = np.column_stack([v_steps, np.full_like(v_steps, ax),
                           np.full_like(v_steps, ay)])
    ax_machines = np.column_stack([v_steps, np.full_like(v_steps, ax)])
    vx = tph.calc_vel_profile.calc_vel_profile(
        ggv=ggv, ax_max_machines=ax_machines, v_max=vmax, kappa=kappa,
        el_lengths=el_lengths, closed=True, drag_coeff=0.0,
        m_veh=VEHICLE_MASS)
    if jerk_lat > 0.0:
        vx = limit_lat_jerk(vx, kappa, el_lengths, jerk_lat, ax, brake)
        vx = limit_long_accel(vx, el_lengths, ax, brake)
    ax_prof = tph.calc_ax_profile.calc_ax_profile(
        vx_profile=np.append(vx, vx[0]), el_lengths=el_lengths,
        eq_length_output=False)
    return vx, ax_prof


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('raceline')
    ap.add_argument('--ay', type=float, required=True)
    ap.add_argument('--ax', type=float, default=6.5)
    ap.add_argument('--brake', type=float, default=4.0)
    ap.add_argument('--jerk-lat', type=float, default=40.0)
    ap.add_argument('--vmax', type=float, default=8.0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    r = np.loadtxt(a.raceline, delimiter=',')
    s, x, y, psi, kappa = (r[:, i] for i in range(5))
    el = np.diff(np.r_[s, s[-1] + (s[-1] - s[-2])])
    vx, ax_prof = reprofile(kappa, el, a.ay, a.ax, a.brake, a.jerk_lat, a.vmax)

    out = np.column_stack([s, x, y, psi, kappa, vx, ax_prof[:len(s)]])
    np.savetxt(a.out + '.csv', out, delimiter=',', fmt='%.4f')
    ideal = float(np.sum(el / vx))
    print(f'{os.path.basename(a.out)}: ay {a.ay} ax {a.ax} brake {a.brake} '
          f'jerk {a.jerk_lat} vmax {a.vmax}')
    print(f'  ideal lap {ideal:.3f} s   v {vx.min():.2f}-{vx.max():.2f} m/s   '
          f'peak ay {np.max(vx * vx * np.abs(kappa)):.2f} m/s2')


if __name__ == '__main__':
    main()
