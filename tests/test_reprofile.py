"""Re-profiling changes the SPEED, never the PATH.

An `ay` ladder built by re-solving the min-curvature QP at each rung compares
two things at once: a different envelope AND a different line (a different
`w_veh`, or a solve that fails outright, moves the geometry).  `reprofile.py`
exists so the ladder is a clean single-variable experiment -- and so a rung
costs seconds rather than minutes on finals day.

That only holds if the geometry really is untouched, which is what this pins.
"""

import os
import subprocess
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENV = os.path.join(REPO, '.venv', 'bin', 'python')
TOOL = os.path.join(REPO, 'tools', 'reprofile.py')
LINE = os.path.join(REPO, 'data', 'racelines', 'porto_mesh_cr_dk0.8.csv')


def _run(tmp_path, ay, **kw):
    out = str(tmp_path / f'rp{ay}')
    cmd = [VENV, TOOL, LINE, '--ay', str(ay), '--out', out]
    for k, v in kw.items():
        cmd += [f'--{k.replace("_", "-")}', str(v)]
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO)
    assert p.returncode == 0, p.stderr
    return np.loadtxt(out + '.csv', delimiter=',')


def test_geometry_is_untouched(tmp_path):
    src = np.loadtxt(LINE, delimiter=',')
    for ay in (6.75, 9.0):
        got = _run(tmp_path, ay)
        assert got.shape == src.shape
        # s, x, y, psi, kappa must be bit-for-bit what went in: only the
        # velocity profile (vx, ax) may move.
        assert np.allclose(got[:, :5], src[:, :5], atol=1e-4), (
            'reprofile moved the PATH; the ay ladder would be confounded')


def test_more_grip_is_never_slower_in_the_plan(tmp_path):
    """Monotonic by construction: a larger ay can only raise the speed cap."""
    laps = []
    for ay in (6.75, 7.5, 9.0):
        a = _run(tmp_path, ay)
        s, vx = a[:, 0], a[:, 5]
        el = np.diff(np.r_[s, s[-1] + (s[-1] - s[-2])])
        laps.append(float(np.sum(el / vx)))
    assert laps[0] > laps[1] > laps[2], f'ideal lap not monotonic in ay: {laps}'


def test_reprofiling_at_the_shipping_envelope_reproduces_the_shipping_profile(
        tmp_path):
    """The sanity check that makes every other rung believable."""
    src = np.loadtxt(LINE, delimiter=',')
    got = _run(tmp_path, 6.75, ax=6.5, brake=4.0, jerk_lat=40, vmax=8.0)
    # vx within 2%: the shipped file was written by the same code path through
    # optimize_raceline, so this catches a drift in the profile machinery.
    assert np.allclose(got[:, 5], src[:, 5], rtol=0.02), (
        'reprofile no longer reproduces the shipped speed profile')
