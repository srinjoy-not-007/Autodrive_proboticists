"""The shared race-log loader must name columns unambiguously and attribute
collisions to the PRE-IMPACT sample.

Three conventions here have each produced a wrong conclusion in a real
analysis, so each gets a test that fails loudly if it is reversed again:

* `x, y` are the IPS ground truth and `px, py` the localization estimate;
* the pre-impact sample for a collision counted at row c is row **c-2**,
  because the logger writes one already-teleported row before the counter
  steps (reading c-1 piles every collision onto the checkpoint);
* raceline CSVs are `s, x, y, psi, kappa, vx, ax`, so `line[:, :2]` is
  `(s, x)` and not a position.
"""

import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('racelog', ROOT / 'tools' / 'racelog.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

HEADER = ['t', 'x', 'y', 'speed', 'steering', 'throttle', 'lap_count',
          'lap_time', 'collision_count', 'collision_event', 'px', 'py',
          'pyaw', 'cte', 'last_lap_time', 'best_lap_time', 'pstamp', 'cmd']

DT = 0.1
V = 2.0                 # m/s along +x
OFFSET = 0.25           # the car drives this far LEFT of the line (y = 0)
EST_BIAS = -0.10        # the estimate believes it is this much further right
CHECKPOINT = (0.5, 0.0)  # where the simulator teleports the car back to
HIT_X = 5.0             # pre-impact position of the one synthetic collision

# A straight reference line along +x: s, x, y, psi, kappa, vx, ax.
_S = np.arange(0.0, 10.0, 0.05)
LINE = np.column_stack([_S, _S, np.zeros_like(_S), np.zeros_like(_S),
                        np.zeros_like(_S), np.full_like(_S, V),
                        np.zeros_like(_S)])


def synth_log(path, collide=True):
    """A straight run at `OFFSET` left of the line, with one collision.

    The collision is written exactly the way `race_logger.py` writes one: the
    simulator teleports the car and publishes the new pose (one IPS row), and
    only then does `collision_count` step, which writes a second row repeating
    that same teleported pose.
    """
    rows, t, coll = [], 100.0, 0

    def row(x, y, **kw):
        base = dict(t=round(t, 3), x=round(x, 4), y=round(y, 4), speed=V,
                    steering=0.1309, throttle=0.1, lap_count=0,
                    lap_time=round(t - 100.0, 2), collision_count=coll,
                    collision_event=0, px=round(x, 4),
                    py=round(y + EST_BIAS, 4), pyaw=0.0, cte=round(-y, 4),
                    last_lap_time=0.0, best_lap_time=0.0,
                    pstamp=round(t - DT, 3), cmd=0.25)
        base.update(kw)
        return base

    x = 0.0
    while x < HIT_X - 1e-9:
        rows.append(row(x, OFFSET))
        x, t = x + V * DT, t + DT
    rows.append(row(HIT_X, OFFSET))        # <- the pre-impact sample, row c-2
    t += DT
    if collide:
        rows.append(row(*CHECKPOINT))      # teleported already, counter not yet
        t += DT
        coll = 1
        rows.append(row(*CHECKPOINT, collision_count=1, collision_event=1))
        t += DT
        x = CHECKPOINT[0]
        while x < 2.0:
            x, t = x + V * DT, t + DT
            rows.append(row(x, OFFSET))
    with open(path, 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=HEADER)
        writer.writeheader()
        writer.writerows(rows)
    return path


class Naming(unittest.TestCase):
    def test_x_y_are_truth_and_px_py_are_the_estimate(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = MODULE.load_log(synth_log(str(Path(tmp) / 'race.csv')))
        driving = log['truth_y'] > 0            # exclude the teleported rows
        self.assertTrue(np.allclose(log['truth_y'][driving], OFFSET))
        self.assertTrue(np.allclose(log['est_xy'][driving, 1],
                                    OFFSET + EST_BIAS))
        # ...and the estimate is not silently aliased onto the truth.
        self.assertFalse(np.allclose(log['est_xy'][driving, 1],
                                     log['truth_y'][driving]))
        self.assertTrue(np.allclose(log['truth_xy'][:, 0], log['truth_x']))

    def test_speed_columns_keep_their_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = MODULE.load_log(synth_log(str(Path(tmp) / 'race.csv'),
                                            collide=False))
        self.assertTrue(np.allclose(log['enc_speed'], V))     # the encoder
        self.assertAlmostEqual(float(MODULE.truth_speed(log)[3]), V, places=3)

    def test_cmd_is_normalized_and_steering_feedback_is_radians(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = MODULE.load_log(synth_log(str(Path(tmp) / 'race.csv'),
                                            collide=False))
        self.assertTrue(np.allclose(log['cmd'], 0.25))
        self.assertTrue(np.allclose(log['cmd_rad'], 0.25 * MODULE.MAX_STEER))
        # `steering` is already radians: half of the command's 0.25 * MAX_STEER
        # here, as a lagging rack would be -- never rescaled by the loader.
        self.assertTrue(np.allclose(log['steer_fb_rad'], 0.1309))

    def test_an_old_log_without_the_new_columns_still_loads(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'old.csv'
            rows = csv.DictReader(open(synth_log(str(Path(tmp) / 'race.csv'))))
            keep = [c for c in HEADER if c not in ('px', 'py', 'cmd', 'cte')]
            with open(path, 'w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=keep,
                                        extrasaction='ignore')
                writer.writeheader()
                writer.writerows(rows)
            log = MODULE.load_log(str(path))
        self.assertNotIn('est_xy', log)      # absent, not a column of NaNs
        self.assertNotIn('cmd', log)
        self.assertIn('truth_xy', log)
        self.assertEqual(MODULE.collisions(log, line=(LINE[:, 0], LINE[:, 1:3]))
                         [0]['s'], HIT_X)


class LineColumns(unittest.TestCase):
    def test_xy_comes_from_columns_1_and_2_not_0_and_1(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'line.csv'
            np.savetxt(path, LINE, delimiter=',')
            s, xy, psi, kappa, v = MODULE.load_line(str(path))
        self.assertTrue(np.allclose(s, LINE[:, 0]))
        self.assertTrue(np.allclose(xy, LINE[:, 1:3]))
        self.assertTrue(np.allclose(v, V))
        # The classic bug: line[:, :2] is (s, x), which is NOT a position.
        self.assertFalse(np.allclose(xy, LINE[:, :2]))

    def test_offset_is_positive_to_the_left_of_travel(self):
        xy = LINE[:, 1:3]
        left = MODULE.signed_offset(xy, [[3.0, 0.4]])
        right = MODULE.signed_offset(xy, [[3.0, -0.4]])
        self.assertAlmostEqual(float(left[0]), 0.4, places=6)
        self.assertAlmostEqual(float(right[0]), -0.4, places=6)


class CollisionAttribution(unittest.TestCase):
    def load(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = MODULE.load_log(synth_log(str(Path(tmp) / 'race.csv')))
        return log, MODULE.collisions(log, line=(LINE[:, 0], LINE[:, 1:3]))

    def test_pre_impact_sample_is_row_c_minus_two(self):
        log, hits = self.load()
        self.assertEqual(len(hits), 1)
        c = int(MODULE.collision_rows(log)[0])
        self.assertEqual(hits[0]['row'], c)
        self.assertEqual(hits[0]['pre_row'], c - 2)
        # Rows c and c-1 are BOTH already at the checkpoint; only c-2 is not.
        self.assertAlmostEqual(log['truth_x'][c], CHECKPOINT[0])
        self.assertAlmostEqual(log['truth_x'][c - 1], CHECKPOINT[0])
        self.assertAlmostEqual(log['truth_x'][c - 2], HIT_X)

    def test_attribution_reports_where_the_car_actually_hit(self):
        _, hits = self.load()
        self.assertAlmostEqual(hits[0]['s'], HIT_X, places=6)
        self.assertAlmostEqual(hits[0]['offset'], OFFSET, places=6)
        self.assertAlmostEqual(hits[0]['truth_speed'], V, places=3)
        self.assertAlmostEqual(hits[0]['cte'], -OFFSET, places=6)

    def test_reading_c_minus_one_would_report_the_checkpoint(self):
        """The bug this rule exists to prevent, pinned so it cannot return."""
        log, hits = self.load()
        c = int(MODULE.collision_rows(log)[0])
        wrong = MODULE.signed_offset(LINE[:, 1:3],
                                     log['truth_xy'][c - 1:c])
        self.assertAlmostEqual(float(wrong[0]), CHECKPOINT[1], places=6)
        self.assertNotAlmostEqual(hits[0]['s'], CHECKPOINT[0], places=2)

    def test_deduplicating_ips_rows_deletes_the_collision_row(self):
        """Why attribution runs on the raw log."""
        log, _ = self.load()
        c = int(MODULE.collision_rows(log)[0])
        self.assertFalse(bool(MODULE.dedupe_ips_mask(log)[c]))

    def test_teleport_mask_covers_the_collision(self):
        log, _ = self.load()
        c = int(MODULE.collision_rows(log)[0])
        self.assertTrue(bool(MODULE.teleport_mask(log, window=0.5)[c]))


class RealLog(unittest.TestCase):
    """Regression on a run whose collisions are all the same corner."""

    LOG = ROOT / 'data' / 'logs' / 'race_20260912_070432.csv'
    LINE = ROOT / 'data' / 'racelines' / 'porto_mesh_mc0.63_ay6.75.csv'

    def test_ten_collisions_all_at_the_same_corner(self):
        if not self.LOG.exists():
            self.skipTest('reference log not present')
        log = MODULE.load_log(str(self.LOG))
        hits = MODULE.collisions(log, line_path=str(self.LINE))
        self.assertEqual(len(hits), 10)
        self.assertEqual(int(log['coll'][-1] - log['coll'][0]), 10)
        for hit in hits:
            self.assertGreaterEqual(hit["s"], 10.88)
            self.assertLessEqual(hit["s"], 11.40)
            self.assertGreaterEqual(hit['cte'], -0.371)
            self.assertLessEqual(hit['cte'], -0.338)
            # The geometry computed here and the error the controller reported
            # live agree, which is what says the attribution row is right.
            self.assertAlmostEqual(hit['offset'], hit['cte'], delta=0.02)
            self.assertGreater(hit['truth_speed'], 1.5)
            self.assertLess(hit['truth_speed'], 3.0)
        self.assertEqual([h['lap'] for h in hits],
                         [0, 2, 12, 14, 16, 17, 18, 19, 25, 27])


if __name__ == '__main__':
    unittest.main()
