#!/usr/bin/env python3
"""Cross-track error of a race log against the line it was driving.

Usage: .venv/bin/python tools/crosstrack.py data/logs/race_X.csv data/racelines/track_v2_safe.csv [s_lo s_hi]

Reports lap-wide |e_lat| p50/p95/max and the same inside an s window (default
14.0-17.5 m, the corner that produces every collision on Porto). Samples within
1 s of a collision are dropped (teleport).

The error is measured from the IPS **ground truth** (`x, y` -- see
`tools/racelog.py`; `px, py` are the localization estimate and are deliberately
not used here), and the speed gate uses the logged encoder speed.
"""
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import racelog  # noqa: E402


def main(log_path, line_path, s_lo=14.0, s_hi=17.5):
    s, xy, *_ = racelog.load_line(line_path)
    log = racelog.load_log(log_path)
    p = log['truth_xy']
    sp = log['enc_speed']
    ok = (~racelog.teleport_mask(log, window=1.0)) & (sp > 0.5)
    j = racelog.project(xy, p)
    e = np.hypot(p[:, 0] - xy[j, 0], p[:, 1] - xy[j, 1])
    sj = s[j]

    def stats(m):
        v = e[m]
        return f'n={m.sum():5d}  p50={np.median(v):.3f}  p95={np.percentile(v,95):.3f}  max={v.max():.3f}'
    print('lap-wide   ', stats(ok))
    w = ok & (sj >= s_lo) & (sj <= s_hi)
    print(f's {s_lo}-{s_hi}  ', stats(w))
    print(f'speed in window mean {sp[w].mean():.2f}')


if __name__ == '__main__':
    a = sys.argv
    main(a[1], a[2], *(float(x) for x in a[3:5]))
