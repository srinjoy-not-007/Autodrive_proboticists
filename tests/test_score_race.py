"""Tests distinguish actual scored windows from whole-run extrapolation."""

import importlib.util
from pathlib import Path
import unittest

PATH = Path(__file__).resolve().parents[1] / 'tools' / 'score_race.py'
SPEC = importlib.util.spec_from_file_location('score_race', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def fixture():
    rows = [dict(t=0, lap_count=0, collision_count=0)]
    for lap in range(1, 14):
        rows.append(dict(t=lap * 10 + (3 if lap >= 6 else 0),
                         lap_count=lap,
                         collision_count=2 + (1 if lap >= 6 else 0)
                         + (20 if lap >= 12 else 0)))
    return rows


class Scoring(unittest.TestCase):
    def test_exact_window_excludes_warmup_and_cooldown(self):
        result = MODULE.score_rows(fixture())
        self.assertEqual(result['elapsed'], 103)
        self.assertEqual(result['collisions'], 1)
        self.assertEqual(result['race_time'], 113)
        self.assertTrue(result['passed'])

    def test_wrong_direction_and_short_runs_cannot_pass(self):
        with self.assertRaises(ValueError):
            MODULE.score_rows([dict(t=i, lap_count=0, collision_count=0)
                               for i in range(200)])

    def test_midrun_recording_is_not_first_race(self):
        with self.assertRaises(ValueError):
            MODULE.score_rows(fixture()[2:])

    def test_reset_cannot_be_silently_merged(self):
        with self.assertRaises(ValueError):
            MODULE.score_rows(fixture() + [dict(t=140, lap_count=0, collision_count=0)])


def detailed(latch_offset=0, rows_per_lap=4, drop_latch=None, columns=True):
    """A new-format log: several samples per lap, with the simulator's own
    `last_lap_time` latch landing `latch_offset` rows after the `lap_count`
    step (the two ride the same bridge cycle but separate callbacks).

    Lap L lasts `10 + L / 10` seconds; it is driven while `lap_count` reads
    L - 1 and its time is latched when the counter steps to L. The sample clock
    deliberately runs 0.05 s long per lap so the three sources cannot be
    confused with each other.
    """
    duration = lambda lap: 10.0 + lap / 10.0
    rows, t, latched, best = [], 0.0, 0.0, 0.0
    for lap in range(0, 14):
        if lap:
            latched = duration(lap)
            best = latched if not best else min(best, latched)
        step = (duration(lap + 1) + 0.05) / rows_per_lap
        for i in range(rows_per_lap):
            row = dict(t=round(t, 4), lap_count=lap,
                       lap_time=round(i * step, 4),
                       collision_count=2 + (1 if lap >= 6 else 0)
                       + (20 if lap >= 12 else 0))
            if columns:
                shown = latched
                if i < latch_offset or lap == drop_latch:
                    # latch not recorded yet on this row (or never published)
                    shown = duration(lap - 1) if lap > 1 else 0.0
                row['last_lap_time'] = shown
                row['best_lap_time'] = best
            rows.append(row)
            t += step
    # the running timer on the last row before a crossing is that lap's time
    for i in range(len(rows) - 1):
        if rows[i + 1]['lap_count'] != rows[i]['lap_count']:
            rows[i]['lap_time'] = round(duration(rows[i]['lap_count'] + 1), 4)
    return rows


class OfficialLapTimes(unittest.TestCase):
    """The organisers accumulate the simulator's latched per-lap times."""

    def expected(self):
        return [round(10.0 + lap / 10.0, 4) for lap in range(2, 12)]

    def test_official_times_come_from_the_simulator_latch(self):
        result = MODULE.score_rows(detailed())
        self.assertEqual(result['official_laps'], self.expected())
        self.assertAlmostEqual(result['official_race_time'],
                               sum(self.expected()) + 10.0)
        # ...and are not the sample clock, which runs long here.
        self.assertGreater(result['elapsed'], sum(self.expected()))
        # All three sources must name the same ten laps.
        self.assertEqual([round(v, 4) for v in result['sim_laps']],
                         self.expected())

    def test_latch_lagging_the_counter_is_still_attributed_correctly(self):
        for offset in (0, 1, 2):
            with self.subTest(offset=offset):
                result = MODULE.score_rows(detailed(latch_offset=offset))
                self.assertEqual(result['official_laps'], self.expected())

    def test_latch_leading_the_counter_is_still_attributed_correctly(self):
        rows = detailed()
        for i in range(len(rows) - 1):
            if rows[i + 1]['lap_count'] != rows[i]['lap_count']:
                rows[i]['last_lap_time'] = rows[i + 1]['last_lap_time']
        self.assertEqual(MODULE.score_rows(rows)['official_laps'],
                         self.expected())

    def test_a_dropped_latch_falls_back_instead_of_shifting_laps(self):
        result = MODULE.score_rows(detailed(drop_latch=5))
        self.assertIsNone(result['official_laps'])
        self.assertIsNone(result['official_race_time'])
        self.assertEqual(len(result['sim_laps']), 10)

    def test_old_logs_without_the_column_still_score(self):
        for rows in (detailed(columns=False), fixture()):
            with self.subTest():
                result = MODULE.score_rows(rows)
                self.assertIsNone(result['official_laps'])
                self.assertEqual(len(result['laps']), 10)

    def test_nonfinite_latch_is_rejected_not_summed(self):
        rows = detailed()
        rows[len(rows) // 2]['last_lap_time'] = float('nan')
        self.assertIsNone(MODULE.score_rows(rows)['official_laps'])

    def test_collision_window_is_unchanged_by_the_new_source(self):
        result = MODULE.score_rows(detailed())
        self.assertEqual(result['collisions'], 1)
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['race_time'], result['elapsed'] + 10.0)


class RealLogs(unittest.TestCase):
    def test_pre_existing_log_on_disk_still_parses(self):
        import csv
        path = (Path(__file__).resolve().parents[1]
                / 'data' / 'logs' / 'race_20260912_015615.csv')
        if not path.exists():
            self.skipTest('reference log not present')
        with open(path) as stream:
            result = MODULE.score_rows(list(csv.DictReader(stream)))
        self.assertIsNone(result['official_laps'])
        self.assertEqual(len(result['laps']), 10)
        self.assertEqual(len(result['sim_laps']), 10)


if __name__ == '__main__':
    unittest.main()
