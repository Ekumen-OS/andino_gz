#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap

from nav2_common.launch import ParseMultiRobotPose

from andino_gz.launch_tools.substitutions import TextJoin


def generate_launch_description():
    pkg_andino_gz = get_package_share_directory('andino_gz')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    ros_bridge_arg = DeclareLaunchArgument(
        'ros_bridge', default_value='True', description='Run ROS bridge node.')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='True', description='Start RViz.')
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='depot.sdf', description='Name of the world to load. Match with map if using Nav2.')
    robots_arg = DeclareLaunchArgument(
        'robots', default_value="andino={x: 0., y: 0., z: 0.1, yaw: 0.};",
        description='Robots to spawn, multiple robots can be stated separated by a ; ')
    gui_config_arg = DeclareLaunchArgument(
        'gui_config',
        default_value='default.config',
        description='Name of the gui configuration file to load.')
    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='False',
        description='Enable Nav2 Bringup.')
    map_name_arg = DeclareLaunchArgument(
      'map', default_value="depot", description='Name of the map to load. It should match the world_name.'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_andino_gz, 'config', 'nav2_params.yaml']),
        description='Nav2 configuration. Full path to the ROS2 parameters file to use for all launched nodes')

    # Variables of launch file.
    rviz = LaunchConfiguration('rviz')
    ros_bridge = LaunchConfiguration('ros_bridge')
    world_name = LaunchConfiguration('world_name')
    map_name = LaunchConfiguration('map')
    gui_config = LaunchConfiguration('gui_config')
    gui_config_path = PathJoinSubstitution([pkg_andino_gz, 'config_gui', gui_config])
    nav2_flag = LaunchConfiguration('nav2')
    params_file = LaunchConfiguration('params_file')

    # Obtains world path.
    world_path = PathJoinSubstitution([pkg_andino_gz, 'worlds', world_name])
    log_world_path = LogInfo(msg=TextJoin(substitutions=["World path: ", world_path]))
    # Obtains the map path.
    map_path = PathJoinSubstitution([pkg_andino_gz, 'maps', map_name, TextJoin(substitutions=[map_name ,'.yaml'])])
    log_map_path = LogInfo(msg=TextJoin(substitutions=["Map path: ", map_path]))
    # Gazebo arguments.
    gz_args = TextJoin(
        substitutions=[
            world_path,
            TextJoin(substitutions=["--gui-config", gui_config_path], separator=' '),
        ],
        separator=' ',
    )
    # Launches the base group: Gazebo sim and ROS bridge for generic Gazebo stuff.
    base_group = GroupAction(
        scoped=True, forwarding=False,
        launch_configurations={
            'ros_bridge': ros_bridge,
            'world_name': world_name,
            'gui_config': gui_config,
        },
        actions=[
            # Gazebo Sim
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': gz_args}.items(),
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
    # The default value isn't getting parsed correctly because ParseMultiRobotPose checks sys.args
    # instead of using launch argument.
    # TODO: Implement our ParseMultiRobotPose substitution for getting robot's pose correctly.
    log_robots_by_user = LogInfo(msg="Robots provided by user.")
    if (robots_list == {}):
        log_robots_by_user = LogInfo(msg="No robots provided, using default:")
        robots_list = {"andino": {"x": 0., "y": 0., "z": 0.1, "yaw": 0.}}
    log_number_robots = LogInfo(msg="Robots to spawn: " + str(robots_list))
    spawn_robots_group = []
    more_than_one_robot = PythonExpression([TextSubstitution(text=str(len(robots_list.keys()))), ' > 1'])
    one_robot = PythonExpression([TextSubstitution(text=str(len(robots_list.keys()))), ' == 1'])
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        # As it is scoped and not forwarding, the launch configuration in this context gets cleared.
        robots_group = GroupAction(
            scoped=True, forwarding=False,
            launch_configurations={
                'rviz': rviz,
                'ros_bridge': ros_bridge,
                'nav2': nav2_flag,
            },
            actions=[
                LogInfo(msg="Group for robot: " + robot_name),
                PushRosNamespace(
                    condition=IfCondition(more_than_one_robot),
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
                # RViz with nav2
                Node(
                    condition=IfCondition(PythonExpression([rviz, ' and ', LaunchConfiguration('nav2')])),
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', os.path.join(pkg_andino_gz, 'rviz', 'andino_gz_nav2.rviz')],
                    parameters=[{'use_sim_time': True}],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                    ],
                ),
                # RViz without nav2
                Node(
                    condition=IfCondition(PythonExpression([rviz, ' and not ', LaunchConfiguration('nav2')])),
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
                ),
            ]
        )
        nav_group = GroupAction(
          scoped=True, forwarding=False,
          launch_configurations={
              'rviz': rviz,
              'ros_bridge': ros_bridge,
              'map': map_path,
              'params_file': params_file,
              'nav2': nav2_flag,
          },
          actions=[
              # Remapping scan topics for Nav2 local and global costmap.
              # As we use relative values in the param file for supporting multiple robots,
              # the scan topic needs to be remapped otherwise goes under global-costmap/scan topic.
              SetRemap(src='/' + robot_name + '/global_costmap/scan', dst='/' + robot_name + '/scan', condition=IfCondition(PythonExpression([more_than_one_robot, ' and ', LaunchConfiguration('nav2')]))),
              SetRemap(src='/' + robot_name + '/local_costmap/scan', dst='/' + robot_name + '/scan', condition=IfCondition(PythonExpression([more_than_one_robot, ' and ', LaunchConfiguration('nav2')]))),
              # Nav2 Bringup for multiple robots
              IncludeLaunchDescription(
                  PythonLaunchDescriptionSource(
                      os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
                  ),
                  launch_arguments={
                    'namespace': robot_name,
                    'use_namespace': 'True',
                    'map': LaunchConfiguration('map'),
                    'autostart': 'True',
                    'use_sim_time': 'True',
                    'params_file': LaunchConfiguration('params_file'),
                  }.items(),
                  condition=IfCondition(PythonExpression([more_than_one_robot, ' and ', LaunchConfiguration('nav2')])),
              ),
              SetRemap(src='/global_costmap/scan', dst='/scan', condition=IfCondition(PythonExpression([one_robot, ' and ', LaunchConfiguration('nav2')]))),
              SetRemap(src='/local_costmap/scan', dst='/scan', condition=IfCondition(PythonExpression([one_robot, ' and ', LaunchConfiguration('nav2')]))),
              # Nav2 Bringup for single robot
              IncludeLaunchDescription(
                  PythonLaunchDescriptionSource(
                      os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
                  ),
                  launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'autostart': 'True',
                    'use_sim_time': 'True',
                    'params_file': LaunchConfiguration('params_file'),
                  }.items(),
                  condition=IfCondition(PythonExpression([one_robot, ' and ', LaunchConfiguration('nav2')])),
              ),
            ]
          )
        spawn_robots_group.append(robots_group)
        spawn_robots_group.append(nav_group)

    ld = LaunchDescription()
    ld.add_action(log_robots_by_user)
    ld.add_action(log_number_robots)
    ld.add_action(ros_bridge_arg)
    ld.add_action(rviz_arg)
    ld.add_action(world_name_arg)
    ld.add_action(robots_arg)
    ld.add_action(gui_config_arg)
    ld.add_action(nav2_arg)
    ld.add_action(map_name_arg)
    ld.add_action(params_file_arg)
    ld.add_action(log_world_path)
    ld.add_action(log_map_path)
    ld.add_action(base_group)
    for group in spawn_robots_group:
        ld.add_action(group)
    return ld
