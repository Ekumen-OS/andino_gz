#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import ReplaceString


def generate_launch_description():
    pkg_andino_gz = get_package_share_directory('andino_gz')
    bridge_config_file_path = os.path.join(pkg_andino_gz, 'config', 'bridge_config.yaml')

    entity_arg = DeclareLaunchArgument(
        'entity', default_value='andino', description='Name of the entity to bridge with Gazebo.')

    # A <entity> placeholder is used in the bridge config file to be replaced by the entity name.
    bridge_config = ReplaceString(
        source_file=bridge_config_file_path,
        replacements={'<entity>': LaunchConfiguration('entity')},
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config
        }],
    )

    return LaunchDescription([
        entity_arg,
        bridge_node,
    ])
