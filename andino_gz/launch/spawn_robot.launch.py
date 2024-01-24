"""Spawn an Andino robot in new Gazebo, also launch the robot_state_publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from xacro import process_file

PKG_ANDINO_DESCRIPTION = get_package_share_directory('andino_description')
PKG_ANDINO_GZ = get_package_share_directory('andino_gz')

def get_robot_description() -> str:
    """
    Obtain the URDF from the xacro file.

    This replace package tag by file tag to works with gazebo
    # See  https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1284

    Considerations: the URDF model of the robot will be set with certain
    presets:
    - use_fixed_caster: false

    Returns
    -------
        URDF of the robot with gazebo data

    """
    # Parse robot description from xacro
    robot_description_file_path = os.path.join(PKG_ANDINO_GZ, 'urdf', 'andino_gz.urdf.xacro')
    mappings = {'use_fixed_caster': 'false'}
    robot_description_config = process_file(robot_description_file_path, mappings=mappings)
    robot_desc = robot_description_config.toprettyxml(indent='  ')
    # Passing absolute path to the robot description due to Gazebo issues finding andino_description pkg path.
    robot_desc = robot_desc.replace(
        'package://andino_description/', f'file://{PKG_ANDINO_DESCRIPTION}/'
    )
    return robot_desc


def generate_launch_description():
    # Arguments
    entity = LaunchConfiguration('entity')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    robot_description_topic = LaunchConfiguration('robot_description_topic')
    rsp_frequency = LaunchConfiguration('rsp_frequency')
    use_sim_time = LaunchConfiguration('use_sim_time')

    entity_argument = DeclareLaunchArgument(
        'entity', default_value='andino', description='Name of the robot.'
    )
    x_argument = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial x pose of andino in the simulation.',
    )
    y_argument = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial y pose of andino in the simulation',
    )
    z_argument = DeclareLaunchArgument(
        'initial_pose_z',
        default_value='0.1',
        description='Initial z pose of andino in the simulation.',
    )
    yaw_argument = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial yaw pose of andino in the simulation.',
    )
    robot_desc_argument = DeclareLaunchArgument(
        'robot_description_topic',
        default_value='/robot_description',
        description='Robot description topic.',
    )
    rsp_frequency_argument = DeclareLaunchArgument(
        'rsp_frequency',
        default_value='30.0',
        description='Robot State Publisher frequency.',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    # Robot state publisher.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'publish_frequency': rsp_frequency,
                'robot_description': get_robot_description(),
            }
        ],
    )

    # Spawn the robot model.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', entity,
            '-topic', robot_description_topic,
            '-x', initial_pose_x,
            '-y', initial_pose_y,
            '-z', initial_pose_z,
            '-R', '0',
            '-P', '0',
            '-Y', initial_pose_yaw,
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            entity_argument,
            x_argument,
            y_argument,
            z_argument,
            yaw_argument,
            robot_desc_argument,
            rsp_frequency_argument,
            use_sim_time_argument,
            rsp_node,
            spawn_node,
        ]
    )
