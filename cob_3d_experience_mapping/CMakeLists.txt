cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_experience_mapping)

set(catkin_RUN_PACKAGES 
	cob_3d_mapping_common
	cob_3d_visualization
	dynamic_reconfigure
	liblemon
	pcl_ros
	roscpp
	rospy
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

find_package(Eigen REQUIRED)


generate_dynamic_reconfigure_options(
	cfg/settings.cfg
	cfg/visualization.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS
		include
		common/include
	LIBRARIES
		cob_3d_experience_mapping
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
#	DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
	include
	common/include
	${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(cob_3d_experience_mapping_lemon_node src/exp_mapping_lemon.cpp)
add_dependencies(cob_3d_experience_mapping_lemon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_3d_experience_mapping_lemon_node
	${catkin_LIBRARIES}
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_3d_experience_mapping_lemon_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/ include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
	PATTERN ".svn" EXCLUDE
)