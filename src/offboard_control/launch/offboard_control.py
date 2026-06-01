from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    control_mode = LaunchConfiguration("control_mode")
    flight_path = LaunchConfiguration("flight_path")
    kp = LaunchConfiguration("Kp")
    kv = LaunchConfiguration("Kv")

    control_mode_arg = DeclareLaunchArgument(
        "control_mode",
        default_value="position",
        description="Choose 'position' or 'acceleration'"
    )
    
    flight_path_arg = DeclareLaunchArgument(
        "flight_path",
        default_value="circle",
        description="Choose the trajectory path (e.g., 'hover', 'step', 'circle', 'figure8')"
    )

    kp_arg = DeclareLaunchArgument(
        "Kp",
        default_value="2.0",
        description="Proportional gain for the acceleration controller"
    )
    
    kv_arg = DeclareLaunchArgument(
        "Kv",
        default_value="1.5",
        description="Velocity gain for the acceleration controller"
    )

    offboard_control_node = Node(
        package="offboard_control",
        executable="offboard_control_srv",
        output="screen",
        parameters=[{
            "control_mode": control_mode,
            "Kp": kp,
            "Kv": kv
        }],
    )
    
    trajectory_publisher_node = Node(
        package="offboard_control",
        executable="trajectory_publisher",
        name="trajectory_publisher",
        output="screen",
        parameters=[{
            "flight_path": flight_path
        }]
    )

    return LaunchDescription([
        control_mode_arg,
        flight_path_arg,
        kp_arg,
        kv_arg,
        offboard_control_node,
        trajectory_publisher_node
    ])