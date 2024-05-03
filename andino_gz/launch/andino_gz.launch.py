#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace

from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    pkg_andino_gz = get_package_share_directory('andino_gz')

    ros_bridge_arg = DeclareLaunchArgument(
        'ros_bridge', default_value='true', description='Run ROS bridge node.')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.')
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='depot.sdf', description='Name of the world to load.')
    robots_arg = DeclareLaunchArgument(
        'robots', default_value="andino={x: 0., y: 0., z: 0.1, yaw: 0.};",
        description='Robots to spawn, multiple robots can be stated separated by a ; ')

    # Variables of launch file.
    rviz = LaunchConfiguration('rviz')
    ros_bridge = LaunchConfiguration('ros_bridge')
    world_name = LaunchConfiguration('world_name')

    # Obtains world path.
    world_path = PathJoinSubstitution([pkg_andino_gz, 'worlds', world_name])

    base_group = GroupAction(
        scoped=True, forwarding=False,
        launch_configurations={
            'ros_bridge': ros_bridge,
            'world_name': world_name
        },
        actions=[
            # Gazebo Sim
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': world_path}.items(),
            ),
            # ROS Bridge for generic Gazebo stuff
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                output='screen',
                namespace='andino_gz_sim',
                condition=IfCondition(ros_bridge),
            ),
        ]
    )

    robots_list = ParseMultiRobotPose('robots').value()
    # When no robots are specified, spawn a single robot at the origin.
    # The default value isn't getting parsed correctly, so we need to check for an empty dictionary.
    if (robots_list == {}):
        robots_list = {"andino": {"x": 0., "y": 0., "z": 0.1, "yaw": 0.}}
    log_number_robots = LogInfo(msg="Robots to spawn: " + str(robots_list))
    spawn_robots_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        # As it is scoped and not forwarding, the launch configuration in this context gets cleared.
        group = GroupAction(
            scoped=True, forwarding=False,
            launch_configurations={
                'rviz': rviz,
                'ros_bridge': ros_bridge
            },
            actions=[
                LogInfo(msg="Group for robot: " + robot_name),
                PushRosNamespace(
                    condition=IfCondition(
                        PythonExpression([TextSubstitution(text=str(len(robots_list.keys()))), ' > 1'])),
                    namespace=robot_name),
                # Spawn the robot and the Robot State Publisher node.
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_andino_gz, 'launch', 'include', 'spawn_robot.launch.py')
                    ),
                    launch_arguments={
                        'entity': robot_name,
                        'initial_pose_x': str(init_pose['x']),
                        'initial_pose_y': str(init_pose['y']),
                        'initial_pose_z': str(init_pose['z']),
                        'initial_pose_yaw': str(init_pose['yaw']),
                        'robot_description_topic': 'robot_description',
                        'use_sim_time': 'true',
                    }.items(),
                ),
                # RViz
                Node(
                    condition=IfCondition(rviz),
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', os.path.join(pkg_andino_gz, 'rviz', 'andino_gz.rviz')],
                    parameters=[{'use_sim_time': True}],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                    ],
                ),
                # Run ros_gz bridge
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_andino_gz, 'launch', 'include', 'gz_ros_bridge.launch.py')
                    ),
                    launch_arguments={
                        'entity': robot_name,
                    }.items(),
                    condition=IfCondition(LaunchConfiguration('ros_bridge')),
                )
            ]
        )
        spawn_robots_group.append(group)

    ld = LaunchDescription()
    ld.add_action(log_number_robots)
    ld.add_action(ros_bridge_arg)
    ld.add_action(rviz_arg)
    ld.add_action(world_name_arg)
    ld.add_action(robots_arg)
    ld.add_action(base_group)
    for group in spawn_robots_group:
        ld.add_action(group)
    return ld
