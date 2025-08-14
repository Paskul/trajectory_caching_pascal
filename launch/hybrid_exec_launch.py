#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pascal_full')
    params_file = os.path.join(pkg_share, 'config', 'cache_config.yaml')

    hybrid_exec = Node(
        package='pascal_full',
        executable='hybrid_exec',
        name='hybrid_exec',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        hybrid_exec,
    ])
