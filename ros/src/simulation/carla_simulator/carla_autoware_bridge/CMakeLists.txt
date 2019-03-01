cmake_minimum_required(VERSION 2.8.3)
project(carla_autoware_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  rospy
)

catkin_package(
#  LIBRARIES carla_autoware_bridge
#  CATKIN_DEPENDS autoware_msgs roscpp rospy
#  DEPENDS system_lib
)

foreach (dir launch)
    install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach (dir)

install(PROGRAMS scripts/launch_bridge.sh scripts/launch_carla.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )