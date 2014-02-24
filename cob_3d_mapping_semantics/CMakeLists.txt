cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_semantics)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cob_3d_mapping_common
  cob_3d_mapping_msgs
  dynamic_reconfigure
  pcl_ros
  rosbag
  roscpp
  visualization_msgs
)

find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(cfg/structure_extraction_node.cfg
  cfg/supporting_plane_extraction_node.cfg
  cfg/table_extraction_node.cfg
)

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
    cob_3d_mapping_semantics
  CATKIN_DEPENDS
    cob_3d_mapping_common
    cob_3d_mapping_msgs
    dynamic_reconfigure
    pcl_ros
    rosbag
    roscpp
    visualization_msgs
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
  common/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_mapping_semantics 
    common/src/table_extraction.cpp
    common/src/supporting_plane_extraction.cpp
    common/src/structure_extraction.cpp
)

## Declare a cpp executable
add_executable(table_extraction_node 
    ros/src/table_extraction_node.cpp
)
add_dependencies(table_extraction_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(supporting_plane_extraction_node 
    ros/src/supporting_plane_extraction_node.cpp
)
add_dependencies(supporting_plane_extraction_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(structure_extraction_node 
    ros/src/structure_extraction_node.cpp
)
add_dependencies(structure_extraction_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(get_tables_client 
    ros/src/get_tables_client.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(table_extraction_node 
  cob_3d_mapping_semantics
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(supporting_plane_extraction_node 
  cob_3d_mapping_semantics 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(structure_extraction_node 
  cob_3d_mapping_semantics 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(get_tables_client 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

