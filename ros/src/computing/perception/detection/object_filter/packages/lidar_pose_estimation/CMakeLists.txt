cmake_minimum_required(VERSION 2.8.3)
project(lidar_pose_estimation)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
  autoware_msgs
)

catkin_package(
  CATKIN_DEPENDS autoware_build_flags roscpp tf2 tf2_geometry_msgs tf2_ros geometry_msgs autoware_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_lane_adjust
  src/lane_adjust.cpp
  src/vector_map.cpp
)
set_target_properties(${PROJECT_NAME}_lane_adjust PROPERTIES OUTPUT_NAME lane_adjust PREFIX "")
target_link_libraries(${PROJECT_NAME}_lane_adjust ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_lane_adjust
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_detected_object_adjuster.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
