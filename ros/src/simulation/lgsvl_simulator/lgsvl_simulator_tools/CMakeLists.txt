cmake_minimum_required(VERSION 2.8.3)
project(lgsvl_simulator_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  lgsvl_msgs
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

# Python scripts
set(PYTHON_SCRIPTS scripts/lgsvl_darknet_labeler.py)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)