cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_point_map)

set(catkin_RUN_PACKAGES 
	actionlib
	actionlib_msgs
	cob_3d_mapping_common
	cob_3d_mapping_msgs
	dynamic_reconfigure
	nodelet
	pcl_ros
	roscpp
	std_srvs
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

#find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(cfg/point_map_nodelet.cfg)

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
	LIBRARIES
		cob_3d_mapping_point_map_nodelets
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
	${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_mapping_point_map_nodelets
	ros/src/point_map_nodelet.cpp
	ros/src/difference_segmentation.cpp
)
add_dependencies(cob_3d_mapping_point_map_nodelets ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_3d_mapping_point_map_nodelets
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

## Declare a cpp executable
add_executable(set_point_map ros/src/set_point_map.cpp)
add_dependencies(set_point_map ${catkin_EXPORTED_TARGETS})
target_link_libraries(set_point_map
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(get_point_map ros/src/get_point_map.cpp)
add_dependencies(get_point_map ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_point_map
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

#rosbuild_check_for_sse()

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_3d_mapping_point_map_nodelets set_point_map get_point_map
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)