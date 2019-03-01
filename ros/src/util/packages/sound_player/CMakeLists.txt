cmake_minimum_required(VERSION 2.8.3)
project(sound_player)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sound_play
  autoware_build_flags
  autoware_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sound_play autoware_msgs

)

include_directories(${catkin_INCLUDE_DIRS}
 ${autoware_config_msgs_INCLUDE_DIRS}
 ${autoware_msgs_INCLUDE_DIRS})


add_executable(sound_play_node
        nodes/sound_play.cpp
        )
target_link_libraries(sound_play_node
        ${catkin_LIBRARIES}
        )
add_dependencies(sound_play_node ${catkin_EXPORTED_TARGETS})


catkin_install_python(PROGRAMS scripts/sound_player.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES scripts/sound_player.yaml
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#TODO INSTALL ALL THE WAVs, MODIFY SCRIPT
