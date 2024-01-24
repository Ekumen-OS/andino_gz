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


def generate_launch_description():
    pkg_andino_gz = get_package_share_directory('andino_gz')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Start RViz.')
    jsp_gui_arg = DeclareLaunchArgument('jsp_gui', default_value='false', description='Run joint state publisher gui node.')

    # TODO Pass world as argument to the launchfile
    world_name = 'world.sdf'
    world_path = os.path.join(pkg_andino_gz, 'worlds', world_name)
    
    # Uncomment the following line to use the empty world
    # world_path = '-r empty.sdf'

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_path}.items(),
    )

    # Spawn the robot and the Robot State Publisher node.
    spawn_robot_and_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_andino_gz, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'entity': 'andino',
            'initial_pose_x': '0',
            'initial_pose_y': '0',
            'initial_pose_z': '0.1',
            'initial_pose_yaw': '0',
            'robot_description_topic': '/robot_description',
            'use_sim_time': 'true',
        }.items(),
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

    return LaunchDescription(
        [
            # Arguments and Nodes
            jsp_gui_arg,
            rviz_arg,
            gazebo,
            spawn_robot_and_rsp,
            jsp_gui,
            rviz,
        ]
    )
