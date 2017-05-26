cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_semantics)

set(catkin_RUN_PACKAGES 
	cob_3d_mapping_common
	cob_3d_mapping_msgs
	cob_3d_visualization
	dynamic_reconfigure
	pcl_ros
	rosbag
	roscpp
	visualization_msgs
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

generate_dynamic_reconfigure_options(cfg/structure_extraction_node.cfg
	cfg/supporting_plane_extraction_node.cfg
	cfg/table_extraction_node.cfg
)

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
		cob_3d_mapping_semantics
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
	common/include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_mapping_semantics
		common/src/table_extraction.cpp
		common/src/supporting_plane_extraction.cpp
		common/src/structure_extraction.cpp
)
add_dependencies(cob_3d_mapping_semantics
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cob_3d_mapping_semantics
	${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(table_extraction_node
		ros/src/table_extraction_node.cpp
)
add_dependencies(table_extraction_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(table_extraction_node
	cob_3d_mapping_semantics
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(environment_monitoring_node
		ros/src/environment_monitoring.cpp
)
add_dependencies(environment_monitoring_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(environment_monitoring_node
	cob_3d_mapping_semantics
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(supporting_plane_extraction_node
		ros/src/supporting_plane_extraction_node.cpp
)
add_dependencies(supporting_plane_extraction_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(supporting_plane_extraction_node
	cob_3d_mapping_semantics
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(structure_extraction_node
		ros/src/structure_extraction_node.cpp
)
add_dependencies(structure_extraction_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(structure_extraction_node
	cob_3d_mapping_semantics
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(get_tables_client
		ros/src/get_tables_client.cpp
)
add_dependencies(get_tables_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_tables_client
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_3d_mapping_semantics table_extraction_node environment_monitoring_node supporting_plane_extraction_node structure_extraction_node get_tables_client
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