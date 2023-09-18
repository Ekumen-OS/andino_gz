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


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Start RViz.')
    rsp_arg = DeclareLaunchArgument('rsp', default_value='false', description='Run robot state publisher node.')
    jsp_gui_arg = DeclareLaunchArgument('jsp_gui', default_value='false', description='Run joint state publisher gui node.')

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_andino_description, 'urdf', 'andino.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_desc = robot_description_config.toprettyxml(indent='  ')
    # Passing absolute path to the robot description due to Gazebo issues finding andino_description pkg path.
    robot_desc = robot_desc.replace(
        'package://andino_description/', f'file://{pkg_andino_description}/'
    )

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
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_andino_description, 'config', 'andino_vis.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='andino',
                output='both',
                parameters=[{'robot_description': robot_desc}],
                condition=IfCondition(LaunchConfiguration('rsp'))
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
        ],
        output='screen',
    )

    # Load Diffdrive plugin
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
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
            rsp,
            rviz,
            spawn,
            load_diff_drive_controller
        ]
    )

    return ld
