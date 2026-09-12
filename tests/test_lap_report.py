"""Per-lap reporting must attribute laps, collisions and offsets exactly.

Every check runs on a synthetic circular track where the right answer is known
by construction, so a regression shows up as a wrong number rather than as a
plot that still looks plausible.
"""

import csv
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

PATH = Path(__file__).resolve().parents[1] / 'tools' / 'lap_report.py'
SPEC = importlib.util.spec_from_file_location('lap_report', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

R = 1.5                 # line radius
OFFSET = 0.2            # the car drives this far inside it -- left, going CCW
N_LINE = 94             # ~0.1 m spacing round the 9.42 m loop
PER_LAP = 64            # log samples per lap
DT = 0.1
# Chord speed of the car, which is what differentiating the pose measures.
V_TRUE = 2 * (R - OFFSET) * math.sin(math.pi / PER_LAP) / DT

_TH = np.linspace(0.0, 2 * np.pi, N_LINE, endpoint=False)
LINE = np.column_stack([
    _TH * R,                                        # s
    R * np.cos(_TH), R * np.sin(_TH),               # x, y (counter-clockwise)
    _TH + np.pi / 2, np.full(N_LINE, 1 / R),        # psi, kappa
    np.full(N_LINE, V_TRUE), np.zeros(N_LINE)])     # vx, ax


def LATCHED(lap):
    """The simulator's latched duration for lap `lap`."""
    return round(3.5 + 0.01 * lap, 3)


def write_map(tmp, name, free):
    """Write a map_server map whose free space is given by free(x, y)."""
    res, w, h = 0.05, 160, 160
    gx = (np.arange(w) + 0.5) * res - 4.0
    gy = (np.arange(h) + 0.5) * res - 4.0
    img = np.where(free(gx[None, :], gy[:, None]), 255, 0).astype(np.uint8)
    prefix = str(Path(tmp) / name)
    with open(prefix + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(np.flipud(img).tobytes())
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {name}.pgm\nresolution: {res}\n'
                f'origin: [-4.0, -4.0, 0.0]\nnegate: 0\n'
                f'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return prefix


def corridor_map(tmp):
    """A 2 m wide straight corridor centred on y = 0."""
    return write_map(tmp, 'straight', lambda x, y: np.abs(y) < 1.0 + 0 * x)


def ring_map(tmp):
    """An annulus 1 m either side of the line."""
    return write_map(tmp, 'ring',
                     lambda x, y: np.abs(np.hypot(x, y) - R) < 1.0)


def synth_log(path, laps=3, official=True, collide_lap=None):
    """`laps` circuits held OFFSET m inside the line, with duplicated rows."""
    header = ['t', 'x', 'y', 'speed', 'steering', 'throttle', 'lap_count',
              'lap_time', 'collision_count', 'collision_event']
    if official:
        header += ['last_lap_time', 'best_lap_time']
    rows, t, count, coll = [], 100.0, 0, 0
    r = R - OFFSET
    for lap in range(1, laps + 1):
        for k in range(PER_LAP):
            th = 2 * np.pi * k / PER_LAP
            if lap == collide_lap and k == PER_LAP // 2:
                coll += 1
            row = dict(t=round(t, 3), x=round(r * math.cos(th), 4),
                       y=round(r * math.sin(th), 4), speed=V_TRUE,
                       steering=0.0, throttle=0.1, lap_count=count,
                       lap_time=round(k * DT, 2), collision_count=coll,
                       collision_event=0)
            if official:
                # The latch carries the duration of the lap just completed, so
                # rows with lap_count == L report lap L. Distinct per lap, as
                # the real simulator's are.
                row['last_lap_time'] = LATCHED(count) if count else 0.0
                row['best_lap_time'] = row['last_lap_time']
            rows.append(row)
            rows.append(dict(row, t=round(t + DT / 2, 3)))  # duplicated IPS row
            t += DT
        count += 1
    # The crossing row itself: a fresh pose, or the duplicate filter drops it.
    crossing = dict(rows[-1], t=round(t, 3), x=round(r * 1.001, 4), y=0.0,
                    lap_count=count, lap_time=0.0)
    if official:
        crossing['last_lap_time'] = crossing['best_lap_time'] = LATCHED(count)
    rows.append(crossing)
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        writer.writerows(rows)
    return path


class Geometry(unittest.TestCase):
    def test_offset_is_positive_to_the_left_of_travel(self):
        th = 0.7
        inside = np.array([[(R - 0.3) * math.cos(th), (R - 0.3) * math.sin(th)]])
        outside = np.array([[(R + 0.3) * math.cos(th), (R + 0.3) * math.sin(th)]])
        for pts, want in ((inside, 0.3), (outside, -0.3)):
            idx = MODULE.project(LINE, pts[:, 0], pts[:, 1])
            e = MODULE.signed_offset(LINE, pts[:, 0], pts[:, 1], idx)
            self.assertAlmostEqual(float(e[0]), want, places=2)

    def test_clearance_charges_the_body_corner_not_the_axle(self):
        with tempfile.TemporaryDirectory() as tmp:
            dist, res, ox, oy = MODULE.load_dist(corridor_map(tmp))
            x = np.arange(-2.0, 2.0, 0.2)
            y = np.full(len(x), 0.8)          # axle 0.2 m from the left wall
            worst = MODULE.body_clearance(dist, res, ox, oy, x, y)
            axle = MODULE.clearance(dist, res, ox, oy, np.column_stack([x, y]))
            mid = slice(2, -2)
            self.assertLess(worst[mid].min(), axle[mid].min())
            # Driving straight, the outer corners sit half a body width out.
            self.assertAlmostEqual(worst[mid].min(),
                                   axle[mid].min() - 0.270 / 2.0, places=1)


class Laps(unittest.TestCase):
    def report(self, corner=(4.0, 5.5), **kw):
        with tempfile.TemporaryDirectory() as tmp:
            log = synth_log(str(Path(tmp) / 'race.csv'), **kw)
            args = MODULE.load_dist(ring_map(tmp))
            bins = np.arange(0.0, LINE[-1, 0] + 1.0, 1.0)
            laps, curves, off, sj = MODULE.lap_rows(
                MODULE.load_log(log), LINE, args, [corner], bins)
            return laps, curves, off, sj, bins

    def test_duplicated_ips_rows_do_not_halve_the_sensor_period(self):
        laps, _, _, _, _ = self.report()
        self.assertEqual([r['lap'] for r in laps], [1, 2, 3])
        for row in laps:
            self.assertAlmostEqual(row['dt_p50'], DT, places=3)

    def test_offset_speed_error_and_official_time(self):
        laps, _, _, _, _ = self.report()
        for row in laps:
            self.assertAlmostEqual(row['e_mean'], OFFSET, places=3)
            self.assertAlmostEqual(row['abs_p95'], OFFSET, places=2)
            # delta, not places: the 3-sample speed smoother blurs the lap edge.
            self.assertAlmostEqual(row['dv_mean'], 0.0, delta=0.01)
            self.assertAlmostEqual(row['ovsp1'], 0.0, delta=0.01)
            self.assertAlmostEqual(row['time'], LATCHED(row['lap']), places=3)
            self.assertGreater(row['clr_min'], 0.0)

    def test_sim_timer_estimate_when_the_latch_is_missing(self):
        laps, _, _, _, _ = self.report(official=False)
        for row in laps:
            # The running timer as read one sensor period before the crossing.
            self.assertAlmostEqual(row['time'], (PER_LAP - 1) * DT, places=2)

    def test_collision_is_charged_to_its_lap_and_its_teleport_dropped(self):
        laps, curves, _, _, _ = self.report(collide_lap=2)
        self.assertEqual([r['coll'] for r in laps], [0, 1, 0])
        hurt = [r for r in laps if r['lap'] == 2][0]
        self.assertTrue(math.isnan(hurt['ovsp1']))   # window is inside the drop
        self.assertTrue(np.isnan(curves[2][4]))      # s 4-5 has no samples left
        self.assertFalse(np.isnan(curves[1][4]))

    def test_repeatable_fraction_separates_bias_from_scatter(self):
        bins = np.arange(0.0, 10.0, 1.0)
        sj = np.tile(np.arange(0.05, 10.0, 0.1), 4)
        shape = np.sin(sj)
        curves = {lap: np.array([shape[(sj >= a) & (sj < a + 1)].mean()
                                 for a in bins[:-1]]) for lap in range(4)}
        frac, top = MODULE.repeatable(curves, shape, sj, bins)
        self.assertGreater(frac, 0.95)               # purely a function of s
        self.assertEqual(len(top), 5)
        rng = np.random.default_rng(0)
        flat = {lap: np.zeros(len(bins) - 1) for lap in range(4)}
        self.assertAlmostEqual(
            MODULE.repeatable(flat, rng.normal(size=len(sj)), sj, bins)[0],
            0.0, places=6)


if __name__ == '__main__':
    unittest.main()
