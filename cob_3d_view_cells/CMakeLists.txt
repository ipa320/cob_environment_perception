cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_view_cells)

set(catkin_RUN_PACKAGES 
	cob_3d_mapping_common
	cob_3d_mapping_msgs
	cob_3d_visualization
	dynamic_reconfigure
	nodelet
	pcl_ros
	roscpp
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

include(../cob_3d_segmentation/cmake/OptimizeForArchitecture.cmake)
OptimizeForArchitecture()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
	INCLUDE_DIRS
		common/include
	LIBRARIES
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		Boost
#		PCL
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	common/include
	/usr/include
	${catkin_INCLUDE_DIRS}
)

add_executable(feature2view ros/src/feature2view.cpp)
add_dependencies(feature2view ${catkin_EXPORTED_TARGETS})
target_link_libraries(feature2view
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS feature2view
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