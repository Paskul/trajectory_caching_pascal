#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    apple_pred = Node(
        package='pascal_full',
        executable='apple_pred',
        name='apple_pred',
        output='screen'
    )

    apple_pred_vis = Node(
        package='pascal_full',
        executable='visualize_apple_pred',
        name='visualize_apple_pred',
        output='screen'
    )

    return LaunchDescription([
        apple_pred,
        apple_pred_vis,
    ])