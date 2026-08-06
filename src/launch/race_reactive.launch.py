"""Bring up the reactive racing stack (Follow-the-Gap only)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('pit_crew'),
                          'config', 'ftg.yaml')
    return LaunchDescription([
        Node(
            package='pit_crew',
            executable='follow_the_gap',
            name='follow_the_gap',
            parameters=[config],
            output='screen',
        ),
    ])
