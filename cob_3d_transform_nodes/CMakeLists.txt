cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_transform_nodes)

set(catkin_RUN_PACKAGES 
	cob_3d_mapping_common
	cob_3d_mapping_msgs
	pcl_ros
	roscpp
	tf
	tf_conversions
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

#find_package(PCL REQUIRED COMPONENTS common)

catkin_package(
	INCLUDE_DIRS
	LIBRARIES
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		#PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	${catkin_INCLUDE_DIRS}
	#${PCL_INCLUDE_DIRS}
)

add_executable(transform_shape_array src/transform_shape_array.cpp)
target_link_libraries(transform_shape_array 
	${catkin_LIBRARIES}
)
add_dependencies(transform_shape_array ${catkin_EXPORTED_TARGETS})

add_executable(transform_pointcloud src/transform_pointcloud.cpp)
target_link_libraries(transform_pointcloud 
	${catkin_LIBRARIES}
	#${PCL_LIBRARIES}
)
add_dependencies(transform_pointcloud ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS transform_shape_array transform_pointcloud
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)