# SPDX-License-Identifier: Apache-2.0
"""Start the Kestrel SLAM node with a configuration file.

    ros2 launch kestrel_ros kestrel.launch.py config:=/path/to/config.yaml

M0: the node only loads and validates the configuration. Sensor inputs arrive in M9.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory('kestrel_ros'), 'config', 'gemini_335lg.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Kestrel YAML configuration file.'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='kestrel_ros',
            executable='kestrel_node',
            name='kestrel',
            output='screen',
            parameters=[{'config_file': LaunchConfiguration('config')}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
