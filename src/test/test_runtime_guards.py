"""Exercise sensor rejection and watchdogs on real ROS nodes without actuation."""

import math
import unittest
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, LaserScan

from pit_crew.mpc import MPCTracker
from pit_crew.localization import ParticleFilter
from pit_crew.follow_the_gap import FollowTheGap


class RuntimeGuards(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_old_duplicate_and_nan_pose_do_not_refresh_watchdog(self):
        node = MPCTracker()
        try:
            pose = PoseStamped()
            pose.pose.orientation.w = 1.0
            pose.header.stamp = node.get_clock().now().to_msg()
            node.on_pose(pose)
            accepted = node.pose
            node.on_pose(pose)
            self.assertEqual(node.pose, accepted)
            pose.header.stamp.sec -= 2
            node.on_pose(pose)
            self.assertEqual(node.pose, accepted)
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x = math.nan
            node.on_pose(pose)
            self.assertEqual(node.pose, accepted)
            node.pose = (*accepted[:3], node.now() - 1.0)
            node.publish = Mock()
            node.control_step()
            node.publish.assert_called_with(node.steer, 0.0)
        finally:
            node.destroy_node()

    def test_empty_scan_does_not_publish_dead_reckoning_as_valid_pose(self):
        node = ParticleFilter()
        try:
            imu = Imu()
            imu.orientation.w = 1.0
            node.on_imu(imu)
            node.publish = Mock()
            scan = LaserScan()
            scan.header.stamp = node.get_clock().now().to_msg()
            node.on_scan(scan)
            node.publish.assert_not_called()
        finally:
            node.destroy_node()

    def test_reactive_fallback_clears_throttle_without_valid_scan(self):
        node = FollowTheGap()
        try:
            node._publish_raw = Mock()
            node.check_scan_freshness()
            self.assertEqual(node._publish_raw.call_args.args[1], 0.0)
            node.on_scan(LaserScan())
            self.assertIsNone(node._valid_scan_at)
        finally:
            node.destroy_node()
