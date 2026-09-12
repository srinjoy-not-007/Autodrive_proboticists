"""Validate baked race assets before any actuator node starts.

This is the only gate between the entrypoint's environment variables and a
race.  `PIT_CREW_MAP`, `PIT_CREW_LINE` and `PIT_CREW_CENTERLINE` are three
separate names that used to default to Porto independently, so on an unseen
finals track the likeliest mistake is not a corrupt file but a CONSISTENT set
of files describing two different circuits.  Validating each file on its own
cannot see that, so the geometry is cross-checked against the map here.
"""

import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def _on_map(path, xy, img, res, origin, label):
    """Every point of a track asset must lie inside the map, on free space."""
    ox, oy = float(origin[0]), float(origin[1])
    gx = np.rint((xy[:, 0] - ox) / res).astype(int)
    gy = np.rint((xy[:, 1] - oy) / res).astype(int)
    h, w = img.shape
    inside = (gx >= 0) & (gy >= 0) & (gx < w) & (gy < h)
    if not inside.all():
        raise ValueError(
            f'{label} {path} leaves the map: {int((~inside).sum())} of '
            f'{len(xy)} points are outside the grid — is this the right track?')
    occupied = float((img[gy, gx] <= 50).mean())
    if occupied > 0.02:
        raise ValueError(
            f'{label} {path} runs through walls on this map '
            f'({occupied:.1%} of points occupied) — wrong track, or a map '
            f'built with a different lidar_offset')
    # The trackers treat the line as cyclic (heading and curvature come from
    # np.roll, the index is taken modulo n), so an open line makes its seam a
    # discontinuity the car drives straight into on lap two.
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    spacing = float(np.median(seg)) if len(seg) else 0.0
    gap = float(np.hypot(xy[0, 0] - xy[-1, 0], xy[0, 1] - xy[-1, 1]))
    if gap > max(3.0 * spacing, 0.5):
        raise ValueError(
            f'{label} {path} is not a closed loop: {gap:.2f} m between its '
            f'last point and its first (spacing {spacing:.3f} m)')


def validate(map_prefix, line, center, scale, lidar_offset=None):
    """Reject missing, corrupt, inconsistent or nonsensical race inputs."""
    scale = float(scale)
    if not math.isfinite(scale) or not 0 < scale <= 1.5:
        raise ValueError('speed scale must be finite and in (0, 1.5]')
    meta = yaml.safe_load(Path(map_prefix + '.yaml').read_text())
    res = float(meta['resolution'])
    if not math.isfinite(res) or res <= 0:
        raise ValueError('map resolution must be positive')
    if not np.isfinite(meta['origin']).all():
        raise ValueError('map origin must be finite')
    # Only the map knows which sensor origin it was built with, and the filter
    # and the map MUST agree: a recon map built at offset 0 raced at 0.1933
    # carries a systematic 0.19 m bias against a line with ~0.29 m of
    # clearance. An unstamped map used to be accepted and silently given
    # whatever the entrypoint's default happened to be.
    if lidar_offset is not None:
        want = float(lidar_offset)
        if not math.isfinite(want):
            raise ValueError('LiDAR offset must be finite')
        if 'lidar_offset' not in meta:
            raise ValueError(
                f'{map_prefix}.yaml has no lidar_offset; it cannot be raced at '
                f'{want:.4f} m without knowing what built it (re-run '
                f'tools/map_from_mesh.py, or stamp the key deliberately)')
        if abs(float(meta['lidar_offset']) - want) > 1e-6:
            raise ValueError(
                f'lidar_offset mismatch: map says {float(meta["lidar_offset"]):.4f} '
                f'm, PIT_CREW_LIDAR_OFFSET says {want:.4f} m')
    elif not math.isfinite(float(meta.get('lidar_offset', 0))):
        raise ValueError('LiDAR offset must be finite')
    with Image.open(map_prefix + '.pgm') as image:
        image.verify()
    # flipud to match load_map in localization and pf_core: row 0 is y = origin.
    img = np.flipud(np.array(Image.open(map_prefix + '.pgm')))
    for path, columns, label in ((line, 6, 'raceline'), (center, 2, 'centerline')):
        data = np.loadtxt(path, delimiter=',', ndmin=2)
        if len(data) < 3 or data.shape[1] < columns or not np.isfinite(data).all():
            raise ValueError(f'invalid numeric track asset: {path}')
        if columns == 6 and (np.any(data[:, 5] <= 0) or np.any(np.diff(data[:, 0]) <= 0)):
            raise ValueError('raceline requires positive speeds and increasing arc length')
        xy = data[:, 1:3] if columns == 6 else data[:, 0:2]
        _on_map(path, xy, img, res, meta['origin'], label)
    print(f'Preflight OK: {map_prefix}, {line}, scale={scale}')


if __name__ == '__main__':
    validate(*sys.argv[1:])
