#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


pkg_andino_description = get_package_share_directory('andino_description')
pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
pkg_andino_gz = get_package_share_directory('andino_gz')


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Start RViz.')
    rsp_arg = DeclareLaunchArgument('rsp', default_value='false', description='Run robot state publisher node.')
    jsp_gui_arg = DeclareLaunchArgument('jsp_gui', default_value='false', description='Run joint state publisher gui node.')

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_andino_gz, 'urdf', 'andino_gz.urdf.xacro')
    mappings = {'use_fixed_caster': 'false'}
    robot_description_config = xacro.process_file(robot_description_file, mappings=mappings)
    robot_desc = robot_description_config.toprettyxml(indent='  ')
    # Passing absolute path to the robot description due to Gazebo issues finding andino_description pkg path.
    robot_desc = robot_desc.replace(
        'package://andino_description/', f'file://{pkg_andino_description}/'
    )

    # TODO Pass world as argument to the launchfile
    world_name = 'world.sdf'
    world_path = os.path.join(pkg_andino_gz, 'worlds', world_name)
    
    # Uncomment the following line to use the empty world
    # world_path = '-r empty.sdf'
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {
                'robot_description': robot_desc,
            }
        ],
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_path}.items(),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_andino_gz, 'rviz', 'andino_gz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Joint state publisher
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='andino',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'andino',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
        ],
        output='screen',
    )

    ld = LaunchDescription(
        [
            # Arguments and Nodes
            jsp_gui_arg,
            rsp_arg,
            rviz_arg,
            gazebo,
            jsp_gui,
            robot_state_publisher,
            rviz,
            spawn,
        ]
    )

    return ld
