#!/usr/bin/env python3
"""Test whether collisions are explained by the car's footprint, not its centre.

The logged pose is the rear axle, but the car is 0.5 m long. A path that
crosses the corridor at an angle swings its body corners much closer to the
wall than the rear-axle clearance suggests. This compares, for one or more
traces, the clearance of the rear-axle point against the clearance of the
worst body corner (heading taken from the direction of travel).

Usage:
  .venv/bin/python tools/footprint_analysis.py data/maps/track_v2 \
      data/maps/track_v2_traj.csv data/logs/race_XXXX.csv ...
"""

import sys

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

from math import cos, sin

# Body box relative to the rear axle (x forward, y left), from the tech guide:
# length 0.500 m, width 0.270 m, wheelbase 0.324 m. The rear axle sits about
# one rear overhang behind the tail.
REAR_OVERHANG = 0.088
FRONT = 0.500 - REAR_OVERHANG
HALF_W = 0.270 / 2.0
CORNERS = np.array([[FRONT, HALF_W], [FRONT, -HALF_W],
                    [-REAR_OVERHANG, HALF_W], [-REAR_OVERHANG, -HALF_W]])


def load_dist(prefix):
    meta = yaml.safe_load(open(prefix + '.yaml'))
    img = np.flipud(np.array(Image.open(prefix + '.pgm')))
    res = float(meta['resolution'])
    dist = ndimage.distance_transform_edt(img > 50) * res
    return dist, res, meta['origin'][0], meta['origin'][1]


def clearance(dist, res, ox, oy, pts):
    ix = ((pts[:, 0] - ox) / res).astype(int)
    iy = ((pts[:, 1] - oy) / res).astype(int)
    ok = (ix >= 0) & (ix < dist.shape[1]) & (iy >= 0) & (iy < dist.shape[0])
    out = np.full(len(pts), np.nan)
    out[ok] = dist[iy[ok], ix[ok]]
    return out


def analyse(name, xy, dist, res, ox, oy, cols=None):
    # Heading from direction of travel, smoothed; skip teleport jumps
    d = np.diff(xy, axis=0)
    step = np.hypot(d[:, 0], d[:, 1])
    good = (step > 0.01) & (step < 0.5)
    head = np.full(len(xy), np.nan)
    head[:-1][good] = np.arctan2(d[good, 1], d[good, 0])
    head = np.interp(np.arange(len(xy)),
                     np.flatnonzero(~np.isnan(head)),
                     head[~np.isnan(head)]) if np.any(~np.isnan(head)) else head

    axle = clearance(dist, res, ox, oy, xy)
    worst = np.full(len(xy), np.inf)
    for cx, cy in CORNERS:
        px = xy[:, 0] + cx * np.cos(head) - cy * np.sin(head)
        py = xy[:, 1] + cx * np.sin(head) + cy * np.cos(head)
        c = clearance(dist, res, ox, oy, np.column_stack([px, py]))
        worst = np.minimum(worst, np.nan_to_num(c, nan=np.inf))

    valid = np.isfinite(axle) & np.isfinite(worst)
    a, w = axle[valid], worst[valid]
    print(f'\n{name}   ({len(a)} samples)')
    print(f'  rear-axle clearance : min {a.min():.3f}  p5 {np.percentile(a,5):.3f}'
          f'  median {np.median(a):.3f}')
    print(f'  worst-corner        : min {w.min():.3f}  p5 {np.percentile(w,5):.3f}'
          f'  median {np.median(w):.3f}')
    print(f'  corner penalty      : median {np.median(a-w):.3f} m  '
          f'max {np.max(a-w):.3f} m')
    print(f'  samples with corner inside a wall (<=0): '
          f'{100*np.mean(w <= 0.0):.2f}%')
    return a, w


def main():
    prefix = sys.argv[1]
    dist, res, ox, oy = load_dist(prefix)
    for path in sys.argv[2:]:
        if path.endswith('_traj.csv'):
            d = np.genfromtxt(path, delimiter=',', names=True)
            xy = np.column_stack([d['x'], d['y']])
            analyse(f'CLEAN FTG RUN  {path.split("/")[-1]}', xy, dist, res, ox, oy)
        else:
            d = np.genfromtxt(path, delimiter=',', names=True)
            xy = np.column_stack([d['x'], d['y']])
            analyse(f'PURE PURSUIT   {path.split("/")[-1]}', xy, dist, res, ox, oy)

    # What the raceline itself would sweep, using its own heading
    for line, label in [(prefix.replace('maps/', 'racelines/') + '_raceline.csv',
                         'RACELINE (as planned)'),
                        (prefix.replace('maps/', 'racelines/') + '_safe.csv',
                         'SAFE LINE (as planned)')]:
        try:
            rl = np.loadtxt(line, delimiter=',')
        except OSError:
            continue
        xy = rl[:, 1:3]
        # Derive heading from the path itself: TUM's psi column uses a
        # north-zero convention, not standard math angles.
        d = np.diff(np.vstack([xy, xy[:1]]), axis=0)
        head = np.arctan2(d[:, 1], d[:, 0])
        axle = clearance(dist, res, ox, oy, xy)
        worst = np.full(len(xy), np.inf)
        for cx, cy in CORNERS:
            px = xy[:, 0] + cx * np.cos(head) - cy * np.sin(head)
            py = xy[:, 1] + cx * np.sin(head) + cy * np.cos(head)
            worst = np.minimum(worst, np.nan_to_num(
                clearance(dist, res, ox, oy, np.column_stack([px, py])),
                nan=np.inf))
        print(f'\n{label}')
        print(f'  centre clearance min {np.nanmin(axle):.3f}  '
              f'worst-corner min {worst.min():.3f}')


if __name__ == '__main__':
    main()
