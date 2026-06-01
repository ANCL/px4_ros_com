cmake_minimum_required(VERSION 3.8)
project(offboard_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros_com REQUIRED) # Standard upstream package

# =========================================================================
# Build Executables
# =========================================================================
add_executable(offboard_control_srv src/offboard_control_srv.cpp)
# Added px4_ros_com here so this node can use the frame transforms if needed
ament_target_dependencies(offboard_control_srv rclcpp px4_msgs nav_msgs geometry_msgs px4_ros_com)
target_link_libraries(offboard_control_srv Eigen3::Eigen)

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher rclcpp px4_msgs px4_ros_com)
target_link_libraries(trajectory_publisher Eigen3::Eigen)

add_executable(path_visualizer src/path_visualizer.cpp)
ament_target_dependencies(path_visualizer rclcpp px4_msgs nav_msgs geometry_msgs px4_ros_com)
target_link_libraries(path_visualizer Eigen3::Eigen)

# =========================================================================
# Installation
# =========================================================================
install(TARGETS offboard_control_srv trajectory_publisher path_visualizer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()