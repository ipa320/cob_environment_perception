cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_common)

set(catkin_RUN_PACKAGES
	cob_3d_mapping_msgs
	eigen_conversions
	libgpc
	pcl_conversions
	libpolypartition
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
    ros/include
    common/include
  LIBRARIES
    cob_3d_mapping_common
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
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_mapping_common
	common/src/shape.cpp
	common/src/polygon.cpp
	common/src/cylinder.cpp
	common/src/dominant_color.cpp
    common/src/io.cpp
)

target_link_libraries(cob_3d_mapping_common
  ${catkin_LIBRARIES}
  ${PCL}
  pcl_sample_consensus pcl_segmentation
)

## Declare a cpp executable
add_executable(test_minimal_rectangle common/test/test_minimal_rectangle.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(test_minimal_rectangle
  ${catkin_LIBRARIES}
)

#Unit tests
add_executable(test_polygon common/test/test_polygon.cpp)
add_executable(test_cylinder common/test/test_cylinder.cpp)

target_link_libraries(test_polygon
  cob_3d_mapping_common
  gtest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(test_cylinder
  cob_3d_mapping_common
  gtest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(test_polygon ${catkin_EXPORTED_TARGETS})
add_dependencies(test_cylinder ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_3d_mapping_common
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/ ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)