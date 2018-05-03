cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_slam)

set(catkin_RUN_PACKAGES 
	cob_3d_mapping_msgs
	#libcgal_addons   # todo: reactivate this line: for now deactivated because it causes Travis to fail for kinetic
	nav_msgs
	pcl_ros
	roscpp
	tf
	tf_conversions
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

#find_package(CGAL REQUIRED)  # todo: reactivate this line: for now deactivated because it causes Travis to fail for kinetic
#find_package(nurbs++ REQUIRED)
##TODO: check for SSE

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
	# Fallback to cmake_modules
	find_package(cmake_modules REQUIRED)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
	set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
	set(EIGEN3_DEFINITIONS ${EIGEN_DEFINITIONS})
	set(EIGEN3_DEPENDS Eigen)
	# Possibly map additional variables to the EIGEN3_ prefix.
else()
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
	set(EIGEN3_DEPENDS Eigen3)
endif()
add_definitions(${EIGEN3_DEFINITIONS})
# do not forget: include_directories(${EIGEN3_INCLUDE_DIRS})


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
		ros/include
	LIBRARIES
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		#CGAL   # todo: reactivate this line: for now deactivated because it causes Travis to fail for kinetic
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
## include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
#include_directories(
#	common/include
#	ros/include
#	${CGAL_INCLUDE_DIR}
#	${EIGEN3_INCLUDE_DIRS}
#)


## Declare a cpp executable
#add_executable(slam_node
#	ros/src/slam_node.cpp
#	common/src/curved_polygons/polygon_merger.cpp
#	common/src/lib/polypartition.cpp
#)
#add_executable(test_atoms ros/test/atoms.cpp)
#add_executable(test_merging ros/test/merging.cpp)
#add_executable(test_slam ros/test/slam.cpp 
#				common/src/curved_polygons/polygon_merger.cpp 
#				common/src/lib/polypartition.cpp
#)

#target_link_libraries(slam_node 
#	${catkin_LIBRARIES}
#	nurbsf 
#	matrix 
#	matrixN 
#	matrixI 
#	${CGAL_LIBRARIES}
#)

#target_link_libraries(test_atoms 
#	${catkin_LIBRARIES}
#	gtest
#	${CGAL_LIBRARIES}
#)

#target_link_libraries(test_merging
#	${catkin_LIBRARIES}
#	gtest 
#	nurbsf 
#	matrix 
#	matrixN 
#	matrixI 
#	${CGAL_LIBRARIES}
#)

#target_link_libraries(test_slam 
#	${catkin_LIBRARIES}
#	gtest 
#	nurbsf 
#	matrix 
#	matrixN 
#	matrixI 
#	${CGAL_LIBRARIES}
#)


#set_target_properties(slam_node PROPERTIES COMPILE_FLAGS -frounding-math) #${SSE_FLAGS}
#set_target_properties(test_atoms PROPERTIES COMPILE_FLAGS -frounding-math) #${SSE_FLAGS}
#set_target_properties(test_merging PROPERTIES COMPILE_FLAGS -frounding-math) #${SSE_FLAGS}
#set_target_properties(test_slam PROPERTIES COMPILE_FLAGS -frounding-math) #${SSE_FLAGS}

#############
## Install ##
#############
## Mark executables and/or libraries for installation
#install(TARGETS slam_node test_atoms test_merging test_slam
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY common/include/${PROJECT_NAME}/ ros/include/${PROJECT_NAME}/
#	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#	PATTERN ".svn" EXCLUDE
#)
