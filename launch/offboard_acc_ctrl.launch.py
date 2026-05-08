#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_mode = LaunchConfiguration("control_mode")
    flight_path = LaunchConfiguration("flight_path")

    return LaunchDescription([
        DeclareLaunchArgument(
            "control_mode",
            default_value="position",
            description="Choose 'position' or 'acceleration'"
        ),
        Node(
            package="px4_ros_com",
            executable="offboard_control_srv",
            #name="offboard_control_srv",
            output="screen",
            parameters=[{
                "control_mode": control_mode,
            }],
        ),
        Node(
            package="px4_ros_com",
            executable="trajectory_publisher",
            name="trajectory_publisher",
            output="screen",
            parameters=[{
                "flight_path": flight_path,
            }]
        )
    ])