cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_point_map)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cob_3d_mapping_common
  cob_3d_mapping_msgs
  cob_srvs
  dynamic_reconfigure
  nodelet
  pcl
  pcl_ros
  roscpp
)

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
  INCLUDE_DIRS
    common/include
  LIBRARIES
    cob_3d_mapping_point_map_nodelets
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    cob_3d_mapping_common
    cob_3d_mapping_msgs
    cob_srvs
    dynamic_reconfigure
    nodelet
    pcl
    pcl_ros
    roscpp
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
)

## Declare a cpp library
add_library(cob_3d_mapping_point_map_nodelets 	
  ros/src/point_map_nodelet.cpp
  ros/src/difference_segmentation.cpp
)
add_dependencies(cob_3d_mapping_point_map_nodelets ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(cob_3d_mapping_point_map_nodelets
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(set_point_map ros/src/set_point_map.cpp)
add_executable(get_point_map ros/src/get_point_map.cpp)

target_link_libraries(set_point_map
  ${catkin_LIBRARIES}
)

target_link_libraries(get_point_map
  ${catkin_LIBRARIES}
)

#rosbuild_check_for_sse()

