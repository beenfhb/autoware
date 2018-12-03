cmake_minimum_required(VERSION 2.8.3)
project(offline_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	rviz
	pcl_ros
	vector_map
	rosbag_storage)

find_package(Qt5 REQUIRED Core Widgets)
find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  	rviz_offline_display
  	random_access_bag
  CATKIN_DEPENDS roscpp rviz vector_map rosbag_storage
)


include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	common
	include
)

link_directories(
	${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

add_library(
	rviz_offline_display
		rviz_plugins/PCDFileDisplay.cpp
		rviz_plugins/CSVTrajectory.cpp
		rviz_plugins/FileProperty.cpp
		rviz_plugins/VectorMapDisplay.cpp
		rviz_plugins/VectorMapLoader.cpp
)

target_link_libraries(
	rviz_offline_display
		Qt5::Core
		Qt5::Widgets
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
)

add_library(
	random_access_bag
		random_access_bag/RandomAccessBag.cpp
)

target_link_libraries(
	random_access_bag
		${catkin_LIBRARIES}
)

add_executable(test_bag
	random_access_bag
		random_access_bag/test.cpp
)

target_link_libraries(
	test_bag
		random_access_bag
		${catkin_LIBRARIES}
)