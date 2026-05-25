#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    control_mode = LaunchConfiguration("control_mode")

    return LaunchDescription([
        DeclareLaunchArgument("control_mode", default_value="position"),

        # 1. Pilot
        Node(package="px4_ros_com", executable="offboard_control_srv", output="screen",
             parameters=[{"control_mode": control_mode}]),

        # 2. Navigator
        Node(package="trajectory_publisher", executable="trajectory_publisher_node", output="screen"),

        # 3. Static Transform for RViz (Map to Odom/BaseLink)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'])
    ])