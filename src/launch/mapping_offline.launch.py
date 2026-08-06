"""Offline SLAM mapping: replay a sensor bag alongside this launch.

Brings up wheel+IMU odometry, the base_link->lidar static TF, and
slam_toolbox in mapping mode with sim time (bag must be played with --clock).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('pit_crew'),
                          'config', 'slam_mapping.yaml')
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar'],
        ),
        Node(
            package='pit_crew',
            executable='odometry',
            name='wheel_imu_odometry',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[config, {'use_sim_time': True}],
            output='screen',
        ),
    ])
