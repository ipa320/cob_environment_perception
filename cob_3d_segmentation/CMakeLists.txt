cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_segmentation)

set(catkin_RUN_PACKAGES 
  actionlib
  actionlib_msgs
  cob_3d_features
  cob_3d_mapping_common
  cob_3d_mapping_filters
  cob_3d_mapping_msgs
  dynamic_reconfigure
  geometry_msgs
  nodelet
  pcl_ros
  libpolypartition
  roscpp
  rosbag
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
	cmake_modules
	message_generation
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${catkin_BUILD_PACKAGES}
)

#find_package(TinyXML REQUIRED)
find_package(VTK REQUIRED COMPONENTS vtkCommon vtkFiltering vtkRendering)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
include(cmake/OptimizeForArchitecture.cmake)
OptimizeForArchitecture()

generate_dynamic_reconfigure_options(cfg/plane_extraction_nodelet.cfg
  cfg/segmentation_nodelet.cfg)

#######################################
## Declare ROS messages and services ##
#######################################


## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
    action
  FILES
    ObjectWatch.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
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
    ros/include
  LIBRARIES
    cob_3d_segmentation
    cob_3d_segmentation_nodelets
  CATKIN_DEPENDS
    ${catkin_RUN_PACKAGES}
    message_runtime
  DEPENDS
#    TinyXML
    VTK
    Boost
    OpenCV
#    PCL
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
#  ${TinyXML_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_segmentation
	common/src/polygon_extraction.cpp
	common/src/cluster_handler.cpp
	common/src/edge_handler.cpp
	common/src/cluster_graph_structure.cpp
	common/src/depth_segmentation.cpp
	common/src/cluster_classifier.cpp
	common/src/plane_extraction.cpp
)
add_dependencies(cob_3d_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(cob_3d_segmentation_nodelets
	ros/src/segmentation_all_in_one_nodelet.cpp
	ros/src/plane_extraction_nodelet.cpp
	ros/src/simple_segmentation_nodelet.cpp
)

add_dependencies(cob_3d_segmentation_nodelets ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(segmentation_node ros/src/segmentation_node.cpp)
add_executable(qppf_node ros/src/qppf_node.cpp)
add_dependencies(qppf_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_executable(qppf_node1 ros/src/qppf_node.cpp)
add_dependencies(qppf_node1 ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_executable(plane_extraction_action_client ros/src/plane_extraction_action_client.cpp)
add_dependencies(plane_extraction_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_executable(test_depth_segmentation common/apps/test_depth_segmentation.cpp)
add_executable(segmentation_visualizer ros/src/segmentation_visualizer.cpp)
add_executable(profile_depth_seg common/apps/profile_depth_seg.cpp)
add_executable(profile_plane_extraction common/apps/profile_plane_extraction.cpp)
add_executable(profile_multi_plane_seg common/apps/profile_multi_plane_seg.cpp)
#add_executable(test_segmentation ros/test/test_segmentation.cpp common/src/polypartition.cpp)
add_executable(test_polygon_extraction ros/test/test_polygon_extraction.cpp)
add_executable(test_plane_extraction common/apps/test_plane_extraction.cpp)
add_executable(test_pcl_segmentation common/apps/test_pcl_segmentation.cpp)
add_executable(tutorial_fast_segmentation common/apps/tutorial_fast_segmentation.cpp)
# plane segmentation evaluation tool:
#add_executable(evaluate_plane_segmentation common/apps/evaluate_plane_segmentation.cpp)

set_target_properties(qppf_node1 PROPERTIES COMPILE_DEFINITIONS "ONLY_PLANES_DEPTH")

target_link_libraries(cob_3d_segmentation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

target_link_libraries(cob_3d_segmentation_nodelets
  cob_3d_segmentation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(segmentation_node
  cob_3d_segmentation
  ${catkin_LIBRARIES}
)

target_link_libraries(qppf_node
  ${catkin_LIBRARIES}
)

target_link_libraries(qppf_node1
  ${catkin_LIBRARIES}
)

target_link_libraries(plane_extraction_action_client
  ${catkin_LIBRARIES}
)

target_link_libraries(test_depth_segmentation
  cob_3d_segmentation
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(segmentation_visualizer
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

target_link_libraries(profile_depth_seg
  cob_3d_segmentation
  ${catkin_LIBRARIES}
)

target_link_libraries(profile_plane_extraction
  cob_3d_segmentation
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

#target_link_libraries(test_segmentation
#  ${catkin_LIBRARIES}
#  gtest
#)

target_link_libraries(test_polygon_extraction
  cob_3d_segmentation
  ${catkin_LIBRARIES}
  gtest
)

target_link_libraries(test_plane_extraction
  cob_3d_segmentation
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(tutorial_fast_segmentation
  cob_3d_segmentation
  ${catkin_LIBRARIES}
)

#target_link_libraries(evaluate_plane_segmentation
#  cob_3d_mapping_tools
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

set_target_properties(cob_3d_segmentation PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DCOMPILE_NODELET")
#set_target_properties(test_segmentation ${SSE_FLAGS})



