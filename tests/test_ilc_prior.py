"""The offline ILC prior must have the sign, the clamp and the format right.

Everything runs on a synthetic circular track where the answer is known by
construction: the car drives a constant 0.05-0.20 m offset round a 1.5 m circle,
so the correct feedforward is a constant with the opposite sign, of known
magnitude.  A sign error here is not a worse lap time, it is a car that steers
*away* from the line everywhere on the track, so it is tested first and tested
with a number rather than a plot.

The third test is the interface test: the npz this tool writes must be the npz
``mpc3_core.ILC.load`` reads -- same key names, same ``nb``, ``bin_m``,
``total_s`` acceptance and a ``laps`` count that makes the table count as
trained.
"""

import csv
import importlib.util
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from pit_crew.mpc3_core import ILC                            # noqa: E402

SPEC = importlib.util.spec_from_file_location(
    'ilc_prior', ROOT / 'tools' / 'ilc_prior.py')
IP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(IP)

R = 1.5                 # line radius, m
N_LINE = 94             # ~0.1 m spacing round the 9.42 m loop
PER_LAP = 64            # log samples per lap
DT = 0.1
BIN = 0.25


def write_line(path):
    """A CCW circle of radius R as a raceline CSV (s,x,y,psi,kappa,v,ax)."""
    th = np.linspace(0.0, 2 * np.pi, N_LINE, endpoint=False)
    s = R * th
    rows = np.column_stack([s, R * np.cos(th), R * np.sin(th),
                            th + math.pi / 2, np.full(N_LINE, 1.0 / R),
                            np.full(N_LINE, 2.0), np.zeros(N_LINE)])
    np.savetxt(path, rows, delimiter=',', fmt='%.6f')
    return rows


def write_log(path, offset, laps=6, collide_lap=None, arc=1.0):
    """A log of a car holding ``offset`` metres LEFT of the line.

    Going counter-clockwise, left is toward the centre, so the driven radius is
    ``R - offset``.  ``collide_lap`` bumps ``collision_count`` in the middle of
    that lap; ``arc`` < 1 drives only that fraction of each lap, which is the
    fragment the coverage / gap gates must reject.
    """
    fields = ['t', 'x', 'y', 'lap_count', 'lap_time', 'collision_count',
              'collision_event']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        t, coll = 0.0, 0
        for lap in range(laps):
            n = max(2, int(round(PER_LAP * arc)))
            for k in range(n):
                th = 2 * math.pi * k / PER_LAP
                if collide_lap == lap and k == PER_LAP // 2:
                    coll += 1
                w.writerow(dict(t=f'{t:.3f}',
                                x=f'{(R - offset) * math.cos(th):.6f}',
                                y=f'{(R - offset) * math.sin(th):.6f}',
                                lap_count=lap, lap_time=f'{k * DT:.2f}',
                                collision_count=coll, collision_event=0))
                t += DT


class Fixture(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.line = os.path.join(self.dir.name, 'circle.csv')
        write_line(self.line)
        self.addCleanup(self.dir.cleanup)

    def build(self, offset, extra=(), laps=6, collide_lap=None, arc=1.0):
        log = os.path.join(self.dir.name, f'log_{offset}_{arc}.csv')
        write_log(log, offset, laps=laps, collide_lap=collide_lap, arc=arc)
        out = os.path.join(self.dir.name, 'table.npz')
        argv = [self.line, log, '--out', out, '--bin', str(BIN)] + list(extra)
        IP.main(argv)
        return out, np.load(out)


class TestSign(Fixture):

    def test_left_lap_steers_right(self):
        """A lap held LEFT of the line must produce a NEGATIVE kappa_ff.

        Positive reference curvature is a left turn (mpc3_core.ILC), so the
        correction for being left is a right-hand curvature offset.
        """
        _out, z = self.build(0.05)
        kff = z['kappa_ff']
        self.assertTrue((kff < 0).all(),
                        f'expected every bin negative, got max {kff.max():+.4f}')
        # ...and the magnitude is the documented inverse plant gain, 2/L^2,
        # times the offset: 0.889 * 0.05.
        self.assertAlmostEqual(float(np.median(kff)), -(2.0 / 1.5 ** 2) * 0.05,
                               delta=0.004)

    def test_right_lap_steers_left(self):
        _out, z = self.build(-0.05)
        self.assertTrue((z['kappa_ff'] > 0).all())

    def test_gain_scales_the_table(self):
        _o1, z1 = self.build(0.05, extra=['--gain', '0.4'])
        _o2, z2 = self.build(0.05, extra=['--gain', '0.8'])
        self.assertAlmostEqual(float(np.median(z2['kappa_ff']))
                               / float(np.median(z1['kappa_ff'])), 2.0,
                               delta=0.02)


class TestClamp(Fixture):

    def test_default_clamp_binds(self):
        """0.30 m of offset asks for 0.267 1/m; the table must stop at 0.0711."""
        _out, z = self.build(0.30)
        kff = z['kappa_ff']
        k_clamp = 2.0 * 0.08 / 1.50 ** 2
        self.assertAlmostEqual(float(np.abs(kff).max()), k_clamp, delta=1e-9)
        self.assertTrue(np.allclose(kff, -k_clamp))

    def test_explicit_clamp(self):
        _out, z = self.build(0.30, extra=['--clamp', '0.02'])
        self.assertLessEqual(float(np.abs(z['kappa_ff']).max()), 0.02 + 1e-12)


class TestGates(Fixture):

    def test_collision_lap_is_dropped(self):
        _out, z = self.build(0.05, laps=6, collide_lap=2)
        # 6 lap groups, the colliding one dropped; the teleport window also
        # eats part of the next lap, so at most 5 survive and at least 4.
        self.assertGreaterEqual(int(z['laps']), 4)
        self.assertLessEqual(int(z['laps']), 5)

    def test_fragments_are_rejected(self):
        """Quarter-laps are not laps: every one must fail the gap gate."""
        with self.assertRaises(ValueError):
            self.build(0.05, arc=0.25)


class TestRoundTrip(Fixture):

    def test_ilc_load_accepts_the_table(self):
        out, z = self.build(0.05)
        total = IP.total_s(np.loadtxt(self.line, delimiter=',')[:, 0])
        ilc = ILC(total, bin_m=BIN, store=out)        # __init__ calls load()
        self.assertEqual(ilc.nb, len(z['kappa_ff']))
        self.assertGreater(ilc.laps, 0, 'table must count as trained')
        self.assertTrue(np.allclose(ilc.kappa_ff, z['kappa_ff']))
        self.assertTrue(np.allclose(ilc.v_scale, 1.0))
        # And the loaded table is used with the sign the solver adds to the
        # reference curvature: left of the line -> a right-hand offset.
        self.assertLess(float(ilc.kappa_offset(np.array([0.0, 3.0, 6.0])).max()),
                        0.0)

    def test_load_rejects_a_mismatched_resolution(self):
        out, _z = self.build(0.05)
        total = IP.total_s(np.loadtxt(self.line, delimiter=',')[:, 0])
        ilc = ILC(total, bin_m=0.5, store=out)
        self.assertEqual(ilc.laps, 0)
        self.assertTrue(np.allclose(ilc.kappa_ff, 0.0))

    def test_keys_are_exactly_what_load_reads(self):
        out, z = self.build(0.05)
        for key in ('kappa_ff', 'v_scale', 'laps', 'total_s', 'bin_m'):
            self.assertIn(key, z.files)
        self.assertAlmostEqual(float(z['bin_m']), BIN)


if __name__ == '__main__':
    unittest.main()
