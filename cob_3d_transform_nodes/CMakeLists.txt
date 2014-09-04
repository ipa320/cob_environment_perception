cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_transform_nodes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cob_3d_mapping_common
  cob_3d_mapping_msgs
  pcl_ros
  roscpp
  tf
  tf_conversions
)

find_package(PCL REQUIRED COMPONENTS common)

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
    cob_3d_mapping_common
    cob_3d_mapping_msgs
    pcl_ros
    tf
    roscpp
    tf_conversions
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
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(transform_shape_array src/transform_shape_array.cpp)
add_executable(transform_pointcloud src/transform_pointcloud.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(transform_shape_array 
  ${catkin_LIBRARIES}
)

target_link_libraries(transform_pointcloud 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(transform_shape_array ${catkin_EXPORTED_TARGETS})

