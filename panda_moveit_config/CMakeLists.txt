cmake_minimum_required(VERSION 3.5)
project(panda_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(panda_description REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Install directories
install(DIRECTORY config launch rviz srdf DESTINATION share/${PROJECT_NAME})
add_executable(waypoint_follower src/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs moveit_ros_planning_interface control_msgs)
install(TARGETS waypoint_follower DESTINATION lib/${PROJECT_NAME})
# Setup the project
ament_package()
