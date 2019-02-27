cmake_minimum_required(VERSION 2.8.3)
project(emergency_stop)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        tf
        std_msgs
        sensor_msgs
        geometry_msgs
        diagnostic_msgs
        jsk_recognition_msgs
        autoware_msgs
        autoware_config_msgs
        pcl_ros
        pcl_conversions
        )

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#        INCLUDE_DIRS include
        LIBRARIES libwaypoint_follower
        CATKIN_DEPENDS roscpp
        tf
        std_msgs
        sensor_msgs
        geometry_msgs
        diagnostic_msgs
        jsk_recognition_msgs
        autoware_msgs
        autoware_config_msgs
        pcl_ros
        pcl_conversions
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(stop_area
        nodes/stop_area/stop_area_node.cpp
        nodes/stop_area/stop_area_core.cpp
        nodes/stop_area/stop_area_util.cpp
        )
target_link_libraries(stop_area
        ${catkin_LIBRARIES})
add_dependencies(stop_area
        ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(TARGETS stop_area
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
