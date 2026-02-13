cmake_minimum_required(VERSION 3.5)
project(rft_ros2_interface)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:/opt/ros/foxy/lib")


install(
  PROGRAMS
  script/interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
