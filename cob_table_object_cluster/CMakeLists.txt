cmake_minimum_required(VERSION 2.8.3)
project(cob_table_object_cluster)

set(catkin_RUN_PACKAGES 
	actionlib
	cob_3d_mapping_common
	cob_3d_mapping_msgs
	cob_object_detection_msgs
	cob_perception_msgs
	cob_vision_utils
	dynamic_reconfigure
	eigen_conversions
	nodelet
	pcl_ros
	roscpp
	std_srvs
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

find_package(Boost REQUIRED COMPONENTS signals)

generate_dynamic_reconfigure_options(cfg/table_object_cluster_nodelet.cfg)


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
		cob_table_object_cluster
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		Boost
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	common/include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_table_object_cluster
	common/src/table_object_cluster.cpp
	ros/src/table_region_crop_nodelet.cpp
)
add_dependencies(cob_table_object_cluster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_table_object_cluster
	${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(table_object_cluster_node	ros/src/table_object_cluster_node.cpp)
add_dependencies(table_object_cluster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(table_object_cluster_node
	cob_table_object_cluster
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(table_object_cluster_action_client_test ros/src/table_object_cluster_action_client_test.cpp)
add_dependencies(table_object_cluster_action_client_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(table_object_cluster_action_client_test
	${catkin_LIBRARIES}
)

add_executable(table_object_cluster_action_server_node ros/src/table_object_cluster_action_server_node.cpp)
add_dependencies(table_object_cluster_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(table_object_cluster_action_server_node
	${catkin_LIBRARIES}
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_table_object_cluster table_object_cluster_node table_object_cluster_action_client_test table_object_cluster_action_server_node
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