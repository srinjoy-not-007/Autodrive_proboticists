"""Validate baked race assets before any actuator node starts."""

import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def validate(map_prefix, line, center, scale):
    """Reject missing, corrupt or nonsensical race inputs with a clear error."""
    scale = float(scale)
    if not math.isfinite(scale) or not 0 < scale <= 1.5:
        raise ValueError('speed scale must be finite and in (0, 1.5]')
    meta = yaml.safe_load(Path(map_prefix + '.yaml').read_text())
    if not math.isfinite(float(meta['resolution'])) or meta['resolution'] <= 0:
        raise ValueError('map resolution must be positive')
    if not np.isfinite(meta['origin']).all():
        raise ValueError('map origin must be finite')
    if not math.isfinite(float(meta.get('lidar_offset', 0))):
        raise ValueError('LiDAR offset must be finite')
    with Image.open(map_prefix + '.pgm') as image:
        image.verify()
    for path, columns in ((line, 6), (center, 2)):
        data = np.loadtxt(path, delimiter=',', ndmin=2)
        if len(data) < 3 or data.shape[1] < columns or not np.isfinite(data).all():
            raise ValueError(f'invalid numeric track asset: {path}')
        if columns == 6 and (np.any(data[:, 5] <= 0) or np.any(np.diff(data[:, 0]) <= 0)):
            raise ValueError('raceline requires positive speeds and increasing arc length')
    print(f'Preflight OK: {map_prefix}, {line}, scale={scale}')


if __name__ == '__main__':
    validate(*sys.argv[1:])
