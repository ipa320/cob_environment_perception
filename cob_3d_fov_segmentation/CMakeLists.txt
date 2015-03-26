cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_fov_segmentation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cob_3d_mapping_common
  dynamic_reconfigure
  tf_conversions
)

#find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(cfg/field_of_view.cfg cfg/fov_segmentation.cfg)

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
    fov
  CATKIN_DEPENDS
    cob_3d_mapping_common
    dynamic_reconfigure
    tf_conversions
  #DEPENDS
  #  Eigen
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
  #${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(fov
    common/src/field_of_view.cpp
    common/src/fov_segmentation.cpp)

## Declare a cpp executable
add_executable(fov_node ros/src/field_of_view_node.cpp)
add_dependencies(fov_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_executable(fov_segmentation_node ros/src/fov_segmentation_node.cpp)
add_dependencies(fov_segmentation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(fov_node
  fov
  ${catkin_LIBRARIES}
)

target_link_libraries(fov_segmentation_node
  fov
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS fov_node fov_segmentation_node fov
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/ ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)
