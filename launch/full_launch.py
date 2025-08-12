#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    #tree_config = os.path.join(
    #    get_package_share_directory('tree_template'),
    #    'config',
    #    'tree_template_config.yaml'
    #)


    # 1) pascal_odom visual_z_estimate
    visual_z = Node(
        package='pascal_odom',
        executable='visual_z_estimate',
        name='visual_z_estimate',
        output='screen'
    )

    # 2) tree_template tree_template
    tree_tmpl = Node(
        package='tree_template',
        executable='tree_template',
        name='tree_template_base',
        output='screen'
    )

    # 3) tree_template tree_template_from_ros2
    tree_tmpl_ros2 = Node(
        package='tree_template',
        executable='tree_template_from_ros2',
        name='tree_template_from_ros2',
        output='screen'
    )

    return LaunchDescription([
        visual_z,
        tree_tmpl,
        tree_tmpl_ros2,
    ])