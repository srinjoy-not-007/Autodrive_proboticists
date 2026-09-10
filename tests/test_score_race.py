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


if __name__ == '__main__':
    unittest.main()
