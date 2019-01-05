cmake_minimum_required(VERSION 2.8.3)
project(white_line_estimator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  autoware_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  dynamic_reconfigure
  pcl_ros
  tf2_ros
  tf2_geometry_msgs
)

generate_dynamic_reconfigure_options(cfg/white_line_estimator.cfg)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(white_line_estimator_node 
  src/white_line_estimator_node.cpp src/white_line_estimator.cpp 
  src/ransac_approximator.cpp src/image_projector.cpp)
target_link_libraries(white_line_estimator_node ${catkin_LIBRARIES})
add_dependencies(white_line_estimator_node ${catkin_EXPORTED_TARGETS})

install(TARGETS white_line_estimator_node 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir cfg)
  install(DIRECTORY ${dir}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)