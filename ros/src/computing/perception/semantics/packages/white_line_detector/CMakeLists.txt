cmake_minimum_required(VERSION 2.8.3)
project(white_line_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  autoware_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)