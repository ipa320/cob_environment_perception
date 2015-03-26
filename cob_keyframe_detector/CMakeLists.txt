cmake_minimum_required(VERSION 2.8.3)
project(cob_keyframe_detector)

set(catkin_RUN_PACKAGES 
  cob_srvs
  dynamic_reconfigure
  roscpp
  sensor_msgs
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

generate_dynamic_reconfigure_options(cfg/keyframe_detector.cfg)

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
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(keyframe_detector_node src/keyframe_detector.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(keyframe_detector_node 
  ${catkin_LIBRARIES}
)

add_dependencies(keyframe_detector_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
