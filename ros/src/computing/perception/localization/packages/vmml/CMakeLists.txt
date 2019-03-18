cmake_minimum_required(VERSION 3.1.0)
project(vmml)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -frtti)

set (CMAKE_MODULE_PATH 
		${CMAKE_MODULE_PATH} 
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set (CMAKE_INCLUDE_CURRENT_DIR		ON)
set (CMAKE_AUTOMOC					ON)
set (CMAKE_AUTOUIC					ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
#	autoware_build_flags
	roscpp
	std_msgs
	tf
	cv_bridge
	rosbag_storage
	
	# For RandomAccessBag
	offline_tools
	
	# Test offline GNSS localization
	nmea_msgs
	gnss
	
	# Test offline NDT localization
	ndt_tku
	pcl_omp_registration
	velodyne_pointcloud
)

find_package (Boost REQUIRED COMPONENTS serialization python graph filesystem)
find_package (OpenCV 3 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (PCL REQUIRED)
find_package (OpenMP REQUIRED)
find_package (PythonLibs REQUIRED)
find_package (G2O REQUIRED)

# Shell-like interface for testing image-based recognition and localization
find_package (Libedit REQUIRED)

find_package (Qt5Widgets CONFIG REQUIRED)

add_subdirectory (DBoW2)

if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


catkin_package(
	LIBRARIES vmml
	CATKIN_DEPENDS roscpp libg2o offline_tools
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${CMAKE_CURRENT_SOURCE_DIR}
	DBoW2
	
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${G2O_INCLUDE_DIR}	
	${LIBEDIT_INCLUDE_DIRS}
)


set(REQUIRED_LIBS
	DBoW2
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	Boost::serialization
	${G2O_LIBRARIES}
)


add_library(
	vmml STATIC
		src/VMap.cpp
		src/MapBuilder2.cpp
		src/KeyFrame.cpp
		src/MapPoint.cpp
		src/Viewer.cpp
		src/ImageDatabase.cpp
		src/BaseFrame.cpp
		src/Matcher.cpp
		src/Frame.cpp
		src/Localizer.cpp
		src/triangulation.cpp
		src/Optimizer.cpp
		src/utilities.cpp
		src/SequenceSLAM.cpp
		src/Trajectory.cpp
		src/Tracklet.cpp
		src/ImagePreprocessor.cpp
		src/datasets/GenericDataset.cpp
		src/datasets/OxfordDataset.cpp
#		src/datasets/CustomDataset.cpp
		src/datasets/MeidaiBagDataset.cpp
		src/datasets/MeidaiLocalizerGNSS.cpp
		src/datasets/MeidaiLocalizerNDT.cpp
		src/datasets/LidarScanBag.cpp
		src/datasets/NdtLocalizer.cpp
		src/datasets/PclNdtLocalizer.cpp
)


# Dataset Browser
add_executable(dataset_browser
	src/bin/dataset_browser/main.cpp
	src/bin/dataset_browser/DatasetBrowser.cpp	
	src/bin/dataset_browser/ratio_layouted_frame.cpp
	src/bin/dataset_browser/dataset_browser.ui
# this header must be included in order to be processed by MOC & UIC
	src/bin/dataset_browser/ratio_layouted_frame.h
)

target_include_directories(dataset_browser
	PUBLIC src/bin/dataset_browser 
)

target_link_libraries(
	dataset_browser
		Qt5::Widgets
		vmml
		${REQUIRED_LIBS}
		${catkin_LIBRARIES}
)


#add_executable(test_bag
#	src/bin/test_bag.cpp
#)
#
#target_link_libraries(
#	test_bag
#		vmml
#		${REQUIRED_LIBS}
#		${catkin_LIBRARIES}
#)


add_executable(vmml_cli
	src/bin/vmml_cli.cpp
)

target_link_libraries(vmml_cli
	vmml
	${REQUIRED_LIBS}
	${LIBEDIT_LIBRARIES}
	${catkin_LIBRARIES}
)


add_executable(test_tf
	src/bin/temporaries/test_tf.cpp
)

target_link_libraries(
	test_tf
		vmml
		${REQUIRED_LIBS}
		${catkin_LIBRARIES}
)


add_executable(test_priv
	src/bin/temporaries/test_priv.cpp
)

target_link_libraries(
	test_priv
		vmml
		${REQUIRED_LIBS}
		${catkin_LIBRARIES}
)

