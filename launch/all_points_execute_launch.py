#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('pascal_full')
    params_file = os.path.join(pkg_share, 'config', 'cache_config.yaml')

    all_exec = Node(
        package='pascal_full',
        executable='all_points_execute',
        name='all_points_execute',
        output='screen',
        parameters=[params_file]
    )

    single_exec = Node(
        package='pascal_full',
        executable='trajectory_execute',
        name='trajectory_execute',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        all_exec,
        single_exec,
    ])