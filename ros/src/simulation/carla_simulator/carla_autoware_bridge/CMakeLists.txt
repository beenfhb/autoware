cmake_minimum_required(VERSION 2.8.3)
project(carla_autoware_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES carla_autoware_bridge
#  CATKIN_DEPENDS autoware_msgs roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
