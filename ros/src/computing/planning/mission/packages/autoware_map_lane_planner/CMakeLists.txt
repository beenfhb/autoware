cmake_minimum_required(VERSION 2.8.3)
project(autoware_map_lane_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_map
  autoware_map_msgs
  autoware_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(autoware_map_lane_planner_node src/autoware_map_lane_planner_node.cpp src/autoware_map_lane_planner.cpp)
target_link_libraries(autoware_map_lane_planner_node ${catkin_LIBRARIES})
add_dependencies(autoware_map_lane_planner_node ${catkin_EXPORTED_TARGETS})

# include header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# CPP Execution programs
set(CPP_EXEC_NAMES autoware_map_lane_planner_node)
foreach(cpp_exec_names ${CPP_EXEC_NAMES})
  install(TARGETS ${cpp_exec_names}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endforeach(cpp_exec_names)