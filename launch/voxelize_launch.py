#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('pascal_full')
    params_file = os.path.join(pkg_share, 'config', 'cache_config.yaml')

    voxelize_node = Node(
        package='pascal_full',
        executable='voxelize',
        name='voxelize',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        voxelize_node,
    ])
