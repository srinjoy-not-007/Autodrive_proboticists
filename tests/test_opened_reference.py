"""The opened reference is a CORRIDOR, and nothing may overwrite it with a line.

`make_track_lines.py` can open a track's corners (`--open-kappa`) and then
sweep min-curvature racelines off the result.  That needs two different files
and they used to share one name:

* the opener writes a CENTERLINE -- four columns, ``x, y, w_tr_right,
  w_tr_left`` -- because the QP's corridor comes from those width columns;
* the ``opened`` CANDIDATE built from it is a RACELINE -- seven columns,
  ``s, x, y, psi, kappa, vx, ax`` -- and was written back to the same stem.

The candidate is solved before the raceline sweep, so by the time the sweep
asked for the opened reference it was reading a raceline as a centerline.  With
no ``w_tr_*`` columns the corridor came out empty and every single ``w_veh``
reported ``no solution (corridor exhausted)`` -- on SRL 2024 IROS that was all
eight width-using candidates off the opened reference, which reads exactly like
a track too tight to solve.  It was a file clobber.

These tests pin both halves: the opener's output format, and the fact that the
two stems are distinct.
"""

import os
import re
import subprocess
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENV = os.path.join(REPO, '.venv', 'bin', 'python')
MTL = os.path.join(REPO, 'tools', 'make_track_lines.py')


def test_reference_and_candidate_stems_are_distinct():
    """`opened_ref` (corridor in) and `opened` (raceline out) must differ.

    Read from the source rather than a run: the failure is silent and a full
    generation pass costs minutes.
    """
    src = open(MTL).read()
    ref = re.search(r"opened_ref = (f'[^']+')", src)
    out = re.search(r"^    opened = (f'[^']+')", src, re.M)
    assert ref and out, 'both stems should be defined in make_track_lines.py'
    assert ref.group(1) != out.group(1), (
        'the opened reference and the opened candidate share a stem; the '
        'candidate will overwrite the corridor it was built from')

    # The opener must WRITE the reference, and the candidate must READ it.
    assert "'--out', opened_ref]" in src, (
        'open_corners.py should write to the reference stem')
    assert re.search(r"cands \+= \[\('opened'.*opened_ref", src), (
        "the 'opened' candidate should be built FROM the reference stem")
    assert "[(opened_ref, '_o')]" in src, (
        'the raceline sweep should use the reference stem, not the candidate')


def test_corridor_opener_emits_four_column_centerline(tmp_path):
    """`--method corridor` must emit x,y,w_tr_right,w_tr_left, not a raceline.

    Built on a synthetic map so the test owns its fixture: a wide ring corridor
    whose centreline is a circle, written as a PGM the opener can read.
    """
    res, w, h = 0.05, 400, 400
    cx, cy = w / 2.0, h / 2.0
    yy, xx = np.mgrid[0:h, 0:w]
    r = np.hypot(xx - cx, yy - cy)
    free = (r > 60) & (r < 120)                      # 3 m wide ring
    img = np.where(free, 254, 0).astype(np.uint8)
    pgm = tmp_path / 'ring.pgm'
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(np.flipud(img).tobytes())
    (tmp_path / 'ring.yaml').write_text(
        'image: ring.pgm\nresolution: %f\norigin: [0.0, 0.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
        'lidar_offset: 0.1933\nsource: mesh\n' % res)

    th = np.linspace(0, 2 * np.pi, 240, endpoint=False)
    rad = 90 * res
    cl = np.column_stack([cx * res + rad * np.cos(th),
                          cy * res + rad * np.sin(th),
                          np.full(len(th), 0.7), np.full(len(th), 0.7)])
    ref = tmp_path / 'cl.csv'
    np.savetxt(ref, cl, delimiter=',', fmt='%.4f')

    out = tmp_path / 'opened'
    p = subprocess.run(
        [VENV, os.path.join(REPO, 'tools', 'open_corners.py'), str(ref),
         '--method', 'corridor', '--map', str(tmp_path / 'ring'),
         '--kappa-target', '1.0', '--wall-floor', '0.45',
         '--out', str(out)], capture_output=True, text=True, cwd=REPO)
    assert p.returncode == 0, p.stderr

    got = np.loadtxt(str(out) + '.csv', delimiter=',')
    assert got.ndim == 2 and got.shape[1] == 4, (
        f'corridor opener emitted {got.shape[1]} columns; the raceline sweep '
        f'reads this as a centerline and needs x,y,w_tr_right,w_tr_left')
    assert np.all(got[:, 2:] >= 0.0), 'corridor widths must be non-negative'
    assert np.all(np.isfinite(got)), 'corridor must be finite'
