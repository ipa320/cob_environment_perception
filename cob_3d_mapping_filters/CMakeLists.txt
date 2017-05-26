cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_filters)

set(catkin_RUN_PACKAGES
	cob_3d_mapping_common
	nodelet
	pcl_ros
	roscpp
	sensor_msgs
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS
		common/include
	LIBRARIES
		cob_3d_mapping_filters
		cob_3d_mapping_filter_nodelets
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	common/include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_mapping_filters
	common/src/jump_edge_filter.cpp
	common/src/speckle_filter.cpp
)
target_link_libraries(cob_3d_mapping_filters
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(cob_3d_mapping_filters ${catkin_EXPORTED_TARGETS})

add_library(cob_3d_mapping_filter_nodelets
	ros/src/amplitude_filter.cpp
	ros/src/intensity_filter.cpp
	ros/src/smoothing_filter.cpp
	ros/src/jump_edge_filter.cpp
	ros/src/speckle_filter.cpp
)
target_link_libraries(cob_3d_mapping_filter_nodelets
	cob_3d_mapping_filters
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(cob_3d_mapping_filter_nodelets ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(test_jump_edge_filter common/test/test_jump_edge_filter.cpp)
target_link_libraries(test_jump_edge_filter
	cob_3d_mapping_filters
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(test_jump_edge_filter ${catkin_EXPORTED_TARGETS})

add_executable(test_speckle_filter common/test/test_speckle_filter.cpp)
target_link_libraries(test_speckle_filter
	cob_3d_mapping_filters
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(test_speckle_filter ${catkin_EXPORTED_TARGETS})

add_executable(profile_filters common/test/profile_filters.cpp)
target_link_libraries(profile_filters
	cob_3d_mapping_filters
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(profile_filters ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_3d_mapping_filters cob_3d_mapping_filter_nodelets test_jump_edge_filter test_speckle_filter profile_filters
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)