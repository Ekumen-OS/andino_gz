#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

###############
#    Docs     #
###############
# This launch file is used to visualize the tf tree of the robot.
# It is convenient when having multiple robots to visualize the tf tree of each robot.


def generate_launch_description():
    robot_ns_arg = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Namespace to used, typically is the name of the robot when more than one Andino is launched.",
    )
    return LaunchDescription(
        [
            robot_ns_arg,
            Node(
                package="tf2_tools",
                executable="view_frames",
                name="view_frames",
                arguments=[
                    "--ros-args",
                    "-r",
                    "/tf:=tf",
                    "-r",
                    "/tf_static:=tf_static",
                ],
                namespace=LaunchConfiguration("robot_name"),
                output="screen",
            ),
        ]
    )
