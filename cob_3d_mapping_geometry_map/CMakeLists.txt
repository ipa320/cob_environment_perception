cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_geometry_map)

set(catkin_RUN_PACKAGES 
  cob_3d_mapping_common
  cob_3d_mapping_msgs
  tf_conversions
  dynamic_reconfigure
  pcl_ros
  rosbag
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

#find_package(PCL REQUIRED)

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
    geometry_map
  CATKIN_DEPENDS
    ${catkin_RUN_PACKAGES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(geometry_map common/src/geometry_map.cpp common/src/geometry_map_entry.cpp)
add_dependencies(geometry_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(geometry_map_node ros/src/geometry_map_node.cpp)
add_dependencies(geometry_map_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(set_geometry_map ros/src/set_geometry_map.cpp)
add_dependencies(set_geometry_map ${catkin_EXPORTED_TARGETS})

add_executable(get_geometry_map ros/src/get_geometry_map.cpp)
add_dependencies(get_geometry_map ${catkin_EXPORTED_TARGETS})

target_link_libraries(geometry_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(geometry_map_node
  geometry_map
  ${catkin_LIBRARIES}
)

target_link_libraries(set_geometry_map
  ${catkin_LIBRARIES}
)

target_link_libraries(get_geometry_map
  ${catkin_LIBRARIES}
)

