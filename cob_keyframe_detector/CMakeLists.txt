cmake_minimum_required(VERSION 2.8.3)
project(cob_keyframe_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cob_srvs
  dynamic_reconfigure
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

generate_dynamic_reconfigure_options(cfg/keyframe_detector.cfg)


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
