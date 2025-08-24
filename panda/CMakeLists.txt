cmake_minimum_required(VERSION 3.5)
project(panda)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(${PROJECT_NAME}_description REQUIRED)
find_package(${PROJECT_NAME}_moveit_config REQUIRED)

# Convert xacros of description and moveit_config packages to URDF, SDF and SRDF
get_filename_component(${PROJECT_NAME}_REPO_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(${PROJECT_NAME}_DESCRIPTION_SCRIPTS_DIR ${${PROJECT_NAME}_REPO_DIR}/${PROJECT_NAME}_description/scripts)
set(${PROJECT_NAME}_MOVEIT_CONFIG_SCRIPTS_DIR ${${PROJECT_NAME}_REPO_DIR}/${PROJECT_NAME}_moveit_config/scripts)
execute_process(
  COMMAND ${${PROJECT_NAME}_DESCRIPTION_SCRIPTS_DIR}/xacro2urdf.bash
)
execute_process(
  COMMAND ${${PROJECT_NAME}_DESCRIPTION_SCRIPTS_DIR}/xacro2sdf.bash
)
execute_process(
  COMMAND ${${PROJECT_NAME}_MOVEIT_CONFIG_SCRIPTS_DIR}/xacro2srdf.bash
)

add_executable(mtc_spray_clean 
  src/mtc_spray_clean.cpp
  src/toilet_scene.cpp
)

add_executable(simple_swipe src/simple_swipe.cpp)
ament_target_dependencies(simple_swipe 
  rclcpp 
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
  shape_msgs
)
install(TARGETS simple_swipe DESTINATION lib/${PROJECT_NAME})

add_executable(toilet_surface_clean src/toilet_surface_clean.cpp)
ament_target_dependencies(toilet_surface_clean 
  rclcpp 
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
  shape_msgs
)
install(TARGETS toilet_surface_clean DESTINATION lib/${PROJECT_NAME})


target_include_directories(mtc_spray_clean PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)



ament_target_dependencies(mtc_spray_clean
  rclcpp
  moveit_core
  moveit_task_constructor_core
  moveit_ros_planning_interface
  geometry_msgs
  moveit_visual_tools
  tf2_geometry_msgs
)

# Install directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
