cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_geometry_map_v2)

set(catkin_RUN_PACKAGES
	cob_3d_mapping_common
	cob_3d_mapping_msgs
	cob_3d_mapping_submodules
	cob_3d_visualization
	dynamic_reconfigure
	eigen_conversions
	libnuklei
	pcl_ros
	rosbag
	roscpp
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

#find_package(Boost REQUIRED)
#find_package(Boost 1.59.0)
#if(Boost_FOUND)
#	include_directories(libs/boost_1_59)
#endif() 


generate_dynamic_reconfigure_options(cfg/geometry_map_node.cfg)

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
		#geometry_map2
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
)

###########
## Build ##
###########

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
		message(MESSAGE "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	common/include
	ros/include
	libs/boost_1_59
	libs/geometry/include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
#	${Boost_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(geometry_map2_node ros/src/geometry_map_node.cpp
	common/src/object.cpp common/src/context.cpp common/src/script.cpp common/src/vis_obj.cpp common/src/plane.cpp common/src/classifier.cpp
)
add_dependencies(geometry_map2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geometry_map2_node
	${catkin_LIBRARIES}
	CGAL
)
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	set_target_properties(geometry_map2_node PROPERTIES COMPILE_DEFINITIONS "DEBUG_")
endif()

catkin_add_gtest(utest ros/src/test.cpp common/src/object.cpp common/src/vis_obj.cpp)
add_dependencies(utest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(utest
	${catkin_LIBRARIES}
	CGAL
)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS geometry_map2_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/ ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
	PATTERN ".svn" EXCLUDE
)