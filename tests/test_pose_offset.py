"""The pose-reference knobs (Sep 19): `pose_along_offset_m`, `lidar_offset_override`.

Both are default-off and the shipping pose stream is unchanged, which
`test_localization_audit.TestShippedPathIsUnchanged` already proves against the
pre-audit commit.  What is proved here is what each knob does when it IS set:

* the shift moves the PUBLISHED point back along its own heading and touches
  nothing else -- same cloud, same weights, same next scan, so the two knobs
  can be measured independently;
* the override beats the map YAML, which deliberately wins over the
  `lidar_offset` parameter and is therefore otherwise unreachable without
  rebuilding the map.

The measurement behind them is in `tests/pose_offset_replay.py`.
"""

import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from test_localization_audit import node, replay                # noqa: E402

sys.path.insert(0, os.environ.get('PIT_CREW_SRC') or os.path.join(ROOT, 'src'))
from pit_crew import vehicle                                    # noqa: E402
from rclpy_stub import Time                                     # noqa: E402

SHIP = dict(max_step_mps=9.0, publish_on_reloc=False)
D = vehicle.IPS_FROM_REAR_AXLE          # 0.0800 m


class TestDefaultsAreInert:

    def test_both_knobs_default_to_zero(self):
        n = node()
        assert n.pose_along_offset == 0.0
        assert n.lidar_offset_override == 0.0

    def test_a_zero_override_leaves_the_maps_own_lever_arm(self):
        n = node(lidar_offset_override=0.0)
        assert n.lidar_offset == pytest.approx(vehicle.LIDAR_FROM_POSE,
                                               abs=1e-9)


class TestTheOutputShift:

    def _pose(self, n):
        p = n.pub_pose.msgs[-1]
        return (p.pose.position.x, p.pose.position.y,
                2.0 * math.asin(max(-1.0, min(1.0, p.pose.orientation.z))))

    def test_the_published_point_moves_back_along_its_own_heading(self):
        n = node(pose_along_offset_m=D)
        n.init_particles_near(1.0, 2.0, 0.0)
        n.imu_yaw = 0.7
        n.pth[:] = 0.0
        n.px[:] = 1.0
        n.py[:] = 2.0
        n.publish(Time())
        x, y, yaw = self._pose(n)
        assert yaw == pytest.approx(0.7, abs=1e-9)
        assert x == pytest.approx(1.0 - D * math.cos(0.7), abs=1e-9)
        assert y == pytest.approx(2.0 - D * math.sin(0.7), abs=1e-9)

    def test_it_is_output_only_the_filter_is_untouched(self):
        """Same cloud, same RNG, same everything -- only the wire differs."""
        base, nb, _ = replay(**SHIP)
        shifted, ns, _ = replay(pose_along_offset_m=D, **SHIP)
        assert np.allclose(nb.px, ns.px) and np.allclose(nb.py, ns.py)
        assert np.allclose(nb.w, ns.w)
        n_checked = 0
        for a, b in zip(base, shifted):
            assert len(a) == len(b)
            for (ax, ay, az), (bx, by, bz) in zip(a, b):
                yaw = 2.0 * math.asin(max(-1.0, min(1.0, az)))
                assert bz == pytest.approx(az, abs=1e-12)
                assert bx == pytest.approx(ax - D * math.cos(yaw), abs=1e-9)
                assert by == pytest.approx(ay - D * math.sin(yaw), abs=1e-9)
                n_checked += 1
        assert n_checked > 50

    def test_a_non_finite_pose_is_still_never_published(self):
        n = node(pose_along_offset_m=D)
        n.init_particles_near(0.0, 0.0, 0.0)
        n.imu_yaw = float('nan')
        before = len(n.pub_pose.msgs)
        n.publish(Time())
        assert len(n.pub_pose.msgs) == before


class TestTheLeverArmOverride:

    def test_it_beats_the_map_yaml(self):
        n = node(lidar_offset_override=vehicle.LIDAR_FROM_REAR_AXLE)
        assert n.lidar_offset == pytest.approx(vehicle.LIDAR_FROM_REAR_AXLE,
                                               abs=1e-9)

    def test_a_non_finite_override_is_ignored(self):
        n = node(lidar_offset_override=float('nan'))
        assert n.lidar_offset == pytest.approx(vehicle.LIDAR_FROM_POSE,
                                               abs=1e-9)

    def test_it_actually_moves_the_ray_origins(self):
        """The knob has to reach `update`, not just the attribute."""
        base, _, _ = replay(**SHIP)
        lever, _, _ = replay(lidar_offset_override=vehicle.LIDAR_FROM_REAR_AXLE,
                             **SHIP)
        moved = [abs(a[0][0] - b[0][0]) + abs(a[0][1] - b[0][1])
                 for a, b in zip(base, lever) if a and b]
        assert len(moved) > 50
        # A different lever arm is a different posterior, not a relabelling:
        # the estimate moves by ~the 0.08 m difference, not by 0.
        assert np.median(moved) > 0.02

    def test_the_two_knobs_are_independent(self):
        """Setting both is the lever arm's stream, shifted -- nothing else."""
        lever, _, _ = replay(lidar_offset_override=vehicle.LIDAR_FROM_REAR_AXLE,
                             **SHIP)
        both, _, _ = replay(lidar_offset_override=vehicle.LIDAR_FROM_REAR_AXLE,
                            pose_along_offset_m=D, **SHIP)
        n_checked = 0
        for a, b in zip(lever, both):
            for (ax, ay, az), (bx, by, bz) in zip(a, b):
                yaw = 2.0 * math.asin(max(-1.0, min(1.0, az)))
                assert bx == pytest.approx(ax - D * math.cos(yaw), abs=1e-9)
                assert by == pytest.approx(ay - D * math.sin(yaw), abs=1e-9)
                n_checked += 1
        assert n_checked > 50
