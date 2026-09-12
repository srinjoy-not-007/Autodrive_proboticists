#!/usr/bin/env python3
"""Turn a generated line plus its map into the controller arguments to race.

Two of the finals decisions are geometric, and both were being taken by hand
off `docs/STATUS.md` on the day:

  `barrier_margin`   the corridor half-width the MPC's barrier keeps between
                     the BODY CORNERS and the wall.  It is only meaningful up
                     to what the line has room for: a line's FEASIBLE margin is
                     (min half-width along the line) - 0.135 m of car
                     half-width, and above that the barrier pushes across the
                     line instead of biasing it outward -- measured, the fast
                     lines get WORSE at 0.40 (docs/STATUS.md, "Barrier
                     mechanism", 2304 harness laps).
  `thr_brake_lead_s` brings the brake phase forward.  On Porto it is free
                     (-0.018 s/lap); on a tight circuit it buys the approach
                     overspeed back (0 collisions in 396 harness laps on
                     fast3) and costs +0.23..+0.30 s/lap.  0.08 is
                     bit-identical to 0.05 at 0.097 s, so it is never worth
                     paying for.

"Min half-width" here is measured the way the BARRIER measures it, and not the
way the distance transform would: `RefPath.load_corridor` marches along the
path normal, left and right, because the distance field at a point gives the
nearest wall in any direction -- on a hairpin that is the inside wall even when
the car is running wide.  Measured against the numbers in STATUS this is the
definition that reproduces them exactly (Porto `cr_dk0.8` 0.420 -> cap 0.285,
the `fast2_m50` lines 0.480 -> 0.345, the safe lines 0.520 -> 0.385).

Usage:
  .venv/bin/python tools/pick_config.py finals_mesh
  .venv/bin/python tools/pick_config.py porto_mesh \
      --line data/racelines/porto_mesh_cr_dk0.8.csv
"""
import argparse
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, 'tools')
sys.path.insert(0, 'src/pit_crew')
from footprint_analysis import load_dist, clearance, CORNERS   # noqa: E402
from mpc2_core import RefPath                                  # noqa: E402

# Car half-width (0.270 m / 2).  The barrier is applied to the body corners, so
# this is what separates "half-width the PATH has" from "margin the BARRIER can
# ask for".
CAR_HALF_W = 0.135

# Candidate margins, richest first.  Only two values have display evidence:
# 0.34 (32 laps, 0 collisions, OFFICIAL 69.44 s -- the first clean real-display
# run there has ever been) and 0.40 (32/0, 70.13 s, and 0/32 laps below
# -0.30 m at the hairpin against 16/32 at 0.34).  0.40 buys ~0.04 m of real
# hairpin margin for ~0.8 s of race, which is the right trade on a pass/fail
# race -- but only where the line can pay for it.
BARRIER_RICH = 0.40
BARRIER_DEFAULT = 0.34

# How much room the margin must have over and above itself before 0.40 is
# taken.  The feasible cap is measured on a 0.02 m marching grid off a 0.025 m
# map, so a line sitting exactly on the cap is not distinguishable from one
# just under it.
BARRIER_SLACK = 0.02

# What makes a circuit "tight" enough for the brake lead to be worth its
# +0.23..+0.30 s/lap.  Both 2025 circuits trip the kappa test (1.68 and 1.74);
# the width test is what catches a circuit that is merely narrow.  Porto's own
# corridor is 1.12 m, and the lead is free there, so tripping on width costs
# nothing where it fires.
TIGHT_KAPPA = 1.2
TIGHT_WIDTH = 1.3

BRAKE_LEAD_TIGHT = 0.05
BRAKE_LEAD_OPEN = 0.0

# How much of a lap-checkpoint trigger box the line must keep in hand.  A lap
# on which one checkpoint does not fire is not scored at all: measured on
# cdctf2025 fast3_m53 (race_20260912_104735), one lap in 32 read 20.4 s -- the
# SIM'S OWN lap_time ran straight through the 10.43 s crossing without
# resetting -- and the race went 112.1 s instead of ~102, which costs more than
# a collision.
#
# The floor is a judgement call, bracketed by measurement, NOT itself measured:
#   * the narrowest margin any SHIPPED line leaves is 0.518 m (porto_mesh
#     cr_dk0.8 at checkpoint 1); cdctf2025 fast3_m53 leaves 0.715 m;
#   * driving gives some of it away -- over 105 checkpoint-runs on the five
#     CDC-TF logs the driven path loses p50 0.145 m and p95 0.561 m of the
#     margin its own line had;
#   * yet 99 clean Porto laps never came closer than 0.304 m to the edge of a
#     trigger (checkpoint 8), and no log outside 104735 has ever missed one.
# 0.35 m therefore sits above every driven margin seen on a clean lap and well
# below every line we ship, so it fires on a genuinely marginal line only.
CHECKPOINT_MIN_MARGIN = 0.35


def geometry(map_prefix, line_csv):
    """Everything the two rules are decided on, measured off the map."""
    dist, res, ox, oy = load_dist(map_prefix)
    meta = yaml.safe_load(open(map_prefix + '.yaml'))
    ref = RefPath(line_csv)
    ref.load_corridor(dist, meta)
    half = np.minimum(ref.w_left, ref.w_right)
    i = int(np.argmin(half))

    rl = np.loadtxt(line_csv, delimiter=',')
    s, vx = rl[:, 0], rl[:, 5]
    xy = rl[:, 1:3]
    d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    head = np.arctan2(d[:, 1], d[:, 0])
    worst = np.full(len(xy), np.inf)
    for cx, cy in CORNERS:
        px = xy[:, 0] + cx * np.cos(head) - cy * np.sin(head)
        py = xy[:, 1] + cx * np.sin(head) + cy * np.cos(head)
        worst = np.minimum(worst, np.nan_to_num(
            clearance(dist, res, ox, oy, np.column_stack([px, py])),
            nan=np.inf))
    return {
        'min_halfwidth': float(half[i]),
        'min_halfwidth_s': float(ref.s[i]),
        'corridor_min_width': float(np.min(ref.w_left + ref.w_right)),
        'kappa_max': float(np.abs(rl[:, 4]).max()),
        'clearance': float(worst.min()),
        'ideal_lap': float(np.sum(np.diff(s) / np.maximum(vx[:-1], 0.1))),
        'v_max': float(vx.max()),
    }


def checkpoint_geometry(line_csv, cp_csv):
    """Min / median margin the line leaves inside the lap-checkpoint triggers.

    Read straight off the sim scene by `tools/map_from_mesh.py`: each gate is a
    trigger box `width` metres across, and the margin is `width/2` minus how
    far off the box centre the line cuts its plane.  Negative (or a plane the
    line never reaches) means the lap would not be counted at all.
    """
    from optimize_raceline import load_checkpoints, checkpoint_margins
    cps = load_checkpoints(cp_csv)
    xy = np.loadtxt(line_csv, delimiter=',')[:, 1:3]
    mg = checkpoint_margins(xy, cps)
    skipped = sorted(k for k, v in mg.items() if v is None or v < 0.0)
    ok = sorted((v, k) for k, v in mg.items() if v is not None)
    return {
        'checkpoints': cp_csv,
        'n_checkpoints': len(cps),
        'checkpoint_margin_min': ok[0][0] if ok else None,
        'checkpoint_margin_at': ok[0][1] if ok else None,
        'checkpoint_margin_p50': float(np.median([v for v, _ in ok])) if ok
        else None,
        'checkpoints_skipped': skipped,
    }


def decide(geo, slack=BARRIER_SLACK, tight_kappa=TIGHT_KAPPA,
           tight_width=TIGHT_WIDTH):
    """The two rules, and why each went the way it did."""
    cap = geo['min_halfwidth'] - CAR_HALF_W
    rich = cap >= BARRIER_RICH + slack
    barrier = BARRIER_RICH if rich else BARRIER_DEFAULT
    why_barrier = (
        f'feasible margin {cap:.3f} >= {BARRIER_RICH:.2f} + {slack:.2f} slack'
        if rich else
        f'feasible margin {cap:.3f} < {BARRIER_RICH:.2f} + {slack:.2f} slack; '
        f'{BARRIER_RICH:.2f} would push the line across its own corridor')

    by_kappa = geo['kappa_max'] > tight_kappa
    by_width = geo['corridor_min_width'] < tight_width
    tight = by_kappa or by_width
    lead = BRAKE_LEAD_TIGHT if tight else BRAKE_LEAD_OPEN
    reasons = []
    if by_kappa:
        reasons.append(f'tightest kappa {geo["kappa_max"]:.2f} > {tight_kappa}')
    if by_width:
        reasons.append(f'corridor min width {geo["corridor_min_width"]:.2f} m '
                       f'< {tight_width} m')
    why_lead = ('tight circuit (' + '; '.join(reasons) + ')' if tight else
                f'open circuit (kappa {geo["kappa_max"]:.2f} <= {tight_kappa}, '
                f'corridor {geo["corridor_min_width"]:.2f} m >= {tight_width} m)')
    return {
        'feasible_margin': cap,
        'barrier_margin': barrier,
        'barrier_why': why_barrier,
        'tight': tight,
        'thr_brake_lead_s': lead,
        'lead_why': why_lead,
    }


def controller_args(pick):
    """The string to hand to PIT_CREW_CONTROLLER_ARGS.

    Both parameters are spelled out even when one of them is the node default:
    the entrypoint appends this AFTER its own built-in `barrier_margin`, and an
    argument that is present is an argument that can be read back out of the
    process listing on the day.
    """
    return (f'-p barrier_margin:={pick["barrier_margin"]:.2f} '
            f'-p thr_brake_lead_s:={pick["thr_brake_lead_s"]:.2f}')


def default_line(slug):
    for kind in ('race', 'safe'):
        p = f'data/racelines/{slug}_{kind}.csv'
        if os.path.exists(p):
            return p
    sys.exit(f'no {slug}_race.csv or {slug}_safe.csv under data/racelines/ -- '
             f'run scripts/finals_drill_mesh.sh first')


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Pick barrier_margin and thr_brake_lead_s from a line\'s '
                    'own geometry.')
    ap.add_argument('slug', help='map stem under data/maps/, e.g. finals_mesh')
    ap.add_argument('--line', default=None, metavar='CSV',
                    help='line to race (default <slug>_race.csv, else _safe)')
    ap.add_argument('--map', default=None, metavar='PREFIX',
                    help='map prefix (default data/maps/<slug>)')
    ap.add_argument('--out', default=None, metavar='JSON',
                    help='sidecar to write (default '
                         'data/racelines/<slug>_pick.json)')
    ap.add_argument('--no-write', action='store_true',
                    help='print the pick without writing the sidecar')
    ap.add_argument('--checkpoints', default=None, metavar='CSV',
                    help='lap-checkpoint export (default '
                         '<map prefix>_checkpoints.csv when it exists)')
    ap.add_argument('--checkpoint-min-margin', type=float,
                    default=CHECKPOINT_MIN_MARGIN, metavar='M',
                    help=f'fail if the line passes closer than M to the edge '
                         f'of any checkpoint trigger (default '
                         f'{CHECKPOINT_MIN_MARGIN})')
    ap.add_argument('--barrier-slack', type=float, default=BARRIER_SLACK)
    ap.add_argument('--tight-kappa', type=float, default=TIGHT_KAPPA)
    ap.add_argument('--tight-width', type=float, default=TIGHT_WIDTH)
    a = ap.parse_args(argv)

    map_prefix = a.map or f'data/maps/{a.slug}'
    line = a.line or default_line(a.slug)
    geo = geometry(map_prefix, line)
    cp_csv = a.checkpoints or (map_prefix + '_checkpoints.csv')
    cp = (checkpoint_geometry(line, cp_csv) if os.path.exists(cp_csv)
          else None)
    if cp is not None:
        geo.update(cp)
    pick = decide(geo, a.barrier_slack, a.tight_kappa, a.tight_width)
    args = controller_args(pick)

    rec = {'slug': a.slug, 'map': map_prefix, 'line': line,
           'controller_args': args}
    rec.update(geo)
    rec.update(pick)

    print(f'line                {line}')
    print(f'min half-width      {geo["min_halfwidth"]:.3f} m '
          f'(at s {geo["min_halfwidth_s"]:.1f}, barrier normal march)')
    print(f'corridor min width  {geo["corridor_min_width"]:.3f} m')
    print(f'tightest kappa      {geo["kappa_max"]:.2f} 1/m')
    print(f'body clearance      {geo["clearance"]:.3f} m')
    print(f'ideal lap           {geo["ideal_lap"]:.2f} s '
          f'(profile v_max {geo["v_max"]:.2f} m/s)')
    print(f'feasible margin     {pick["feasible_margin"]:.3f} m '
          f'(= min half-width - {CAR_HALF_W:.3f} car half-width)')
    print(f'barrier_margin      {pick["barrier_margin"]:.2f}  '
          f'-- {pick["barrier_why"]}')
    print(f'thr_brake_lead_s    {pick["thr_brake_lead_s"]:.2f}  '
          f'-- {pick["lead_why"]}')
    cp_fail = None
    if cp is None:
        print(f'checkpoint margin    -- no {cp_csv}; NOT CHECKED')
    elif cp['checkpoints_skipped']:
        cp_fail = (f'the line passes OUTSIDE checkpoint(s) '
                   f'{cp["checkpoints_skipped"]}: the sim would not count the '
                   f'lap at all')
        print(f'checkpoint margin   *** SKIPS {cp["checkpoints_skipped"]} ***')
    else:
        lo = cp['checkpoint_margin_min']
        print(f'checkpoint margin   {lo:.3f} m at checkpoint '
              f'{cp["checkpoint_margin_at"]} '
              f'(median {cp["checkpoint_margin_p50"]:.3f}, '
              f'{cp["n_checkpoints"]} gates)')
        if lo < a.checkpoint_min_margin:
            cp_fail = (f'checkpoint margin {lo:.3f} m at checkpoint '
                       f'{cp["checkpoint_margin_at"]} is below the '
                       f'{a.checkpoint_min_margin:.2f} m floor')
    if a.slug == 'porto_mesh':
        print('NOTE: porto_mesh is the QUALIFIER and does not take this pick. '
              'Its shipped config is the documented 0.40 exception on '
              'cr_dk0.8, pending the 100-lap Xvfb run at 0.40 (chain6).')
    print()
    print(f'PIT_CREW_CONTROLLER_ARGS="{args}"')

    if not a.no_write:
        out = a.out or f'data/racelines/{a.slug}_pick.json'
        with open(out, 'w') as fh:
            json.dump(rec, fh, indent=2, sort_keys=True)
            fh.write('\n')
        print(f'wrote {out}')
    if cp_fail:
        print()
        print('*' * 72)
        print(f'FAIL: {cp_fail}.')
        print('A lap with a missed checkpoint is not scored -- measured 20.4 s '
              'against 10.2 (cdctf2025 fast3_m53, 2026-09-12).')
        print('Regenerate the line with make_track_lines.py --checkpoints '
              f'{cp_csv} --checkpoint-margin '
              f'{a.checkpoint_min_margin:.2f}, or pass '
              '--checkpoint-min-margin to overrule this deliberately.')
        print('*' * 72)
        rec['checkpoint_fail'] = cp_fail
        if argv is None:
            sys.exit(2)
    return rec


if __name__ == '__main__':
    main()
