cmake_minimum_required(VERSION 3.8)

project(swerve_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

add_executable(
  swerve_controller_node

  src/swerve_controller_node.cpp

  src/swerve_controller.cpp

  src/kinematic.cpp

  src/pid.cpp
)

ament_target_dependencies(
  swerve_controller_node

  rclcpp

  geometry_msgs

  sensor_msgs

  std_msgs

  nav_msgs

  tf2

  tf2_ros
)

install(
  TARGETS
    swerve_controller_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION
    include/
)

ament_package()