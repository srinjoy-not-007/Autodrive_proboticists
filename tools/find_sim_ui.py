#!/usr/bin/env python3
"""Locate the AutoDRIVE Simulator's UI canvas in a screenshot of a real display.

On Xvfb the canvas is a bare 1920x1080 at (0,0) and the sidebar buttons sit at
fixed pixels (hamburger 75,43; Connection 75,409; ...). On a real desktop the
window is decorated and placed anywhere, and `xdotool getwindowgeometry`
reports the FRAME (title bar + invisible borders), not the canvas, so clicks
computed from it miss by ~(30,100) px. The canvas is found from the sim's own
header bar instead: a full-width band of RGB (50,50,50) with white icons,
84 px tall on a 1080 canvas. Prints "X Y W" = canvas origin and width, or
exits 1.

Usage: find_sim_ui.py screenshot.png
"""
import sys
import numpy as np
from PIL import Image

HEADER_RGB = np.array([50, 50, 50])
TOL = 4
MIN_RUN = 1200          # px of header grey in one row (canvas >= 1280 wide)


def find_canvas(img):
    a = np.asarray(img.convert('RGB')).astype(int)
    grey = np.all(np.abs(a - HEADER_RGB) <= TOL, axis=2)
    for y in range(a.shape[0]):
        row = grey[y]
        if row.sum() < MIN_RUN:
            continue
        # longest run of grey (icons/text punch holes; allow gaps <= 120 px)
        xs = np.flatnonzero(row)
        gaps = np.diff(xs)
        cuts = np.flatnonzero(gaps > 120)
        starts = np.r_[0, cuts + 1]
        ends = np.r_[cuts, len(xs) - 1]
        k = np.argmax(xs[ends] - xs[starts])
        x0, x1 = int(xs[starts[k]]), int(xs[ends[k]])
        if x1 - x0 >= MIN_RUN:
            # The header bar is ~84 px tall on a 1080 canvas; a transient
            # frame mid-resize (or an unrelated grey band) is thinner.
            band = grey[y:y + 120, x0:x1 + 1].mean(axis=1) > 0.6
            height = int(np.argmin(band)) if not band.all() else len(band)
            if height < 40:
                continue
            return x0, y, x1 - x0 + 1
    return None


def main():
    res = find_canvas(Image.open(sys.argv[1]))
    if res is None:
        print('no simulator header bar found', file=sys.stderr)
        sys.exit(1)
    print('%d %d %d' % res)


if __name__ == '__main__':
    main()
