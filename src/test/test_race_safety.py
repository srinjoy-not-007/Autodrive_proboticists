"""Regression tests for reset state and nonfinite sensor input."""

import math
import unittest

from pit_crew.vehicle import SpeedEstimator, WHEEL_RADIUS


class SpeedSafety(unittest.TestCase):
    def test_reset_clears_both_wheel_filters(self):
        estimator = SpeedEstimator()
        for wheel in ('L', 'R'):
            estimator.update(wheel, 1.0, 0.0)
            estimator.update(wheel, 1.1, 0.5 / WHEEL_RADIUS)
        self.assertAlmostEqual(estimator.speed, 5.0)
        estimator.jumped = True
        self.assertTrue(estimator.consume_jump())
        estimator.update('L', 2.0, 0.0)
        estimator.update('L', 2.1, 0.0)
        self.assertEqual(estimator.speed, 0.0)
        self.assertFalse(estimator.consume_jump())

    def test_nonfinite_input_cannot_poison_next_sample(self):
        estimator = SpeedEstimator(alpha=1.0)
        estimator.update('L', 1.0, 0.0)
        estimator.update('L', 1.1, math.nan)
        estimator.update('L', math.inf, 100.0)
        estimator.update('L', 1.2, 0.2 / WHEEL_RADIUS)
        self.assertAlmostEqual(estimator.speed, 1.0)


if __name__ == '__main__':
    unittest.main()
