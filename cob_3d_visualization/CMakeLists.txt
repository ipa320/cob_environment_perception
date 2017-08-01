cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_visualization)

set(catkin_RUN_PACKAGES 
	cob_3d_mapping_common
	cob_3d_mapping_msgs
	eigen_conversions
	interactive_markers
	pcl_conversions
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
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
		include
	LIBRARIES
#		cob_3d_mapping_semantics
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
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(shape_visualization
	src/shape_visualization.cpp
	src/shape_marker.cpp
)
add_dependencies(shape_visualization ${catkin_EXPORTED_TARGETS})
target_link_libraries(shape_visualization
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(test_shape_array
	test/test_shape_array.cpp
)
add_dependencies(test_shape_array ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_shape_array
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS shape_visualization test_shape_array
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)