cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(cob_3d_mapping_point_map	
	common/src/point_map.cpp)
	
rosbuild_add_library(cob_3d_mapping_point_map_nodelets
	ros/src/point_map_nodelet.cpp
	ros/src/difference_segmentation.cpp)
	
rosbuild_add_executable(trigger_mapping_action_client ros/src/trigger_mapping_action_client.cpp)
rosbuild_add_executable(set_reference_map_client ros/src/set_reference_map.cpp)
rosbuild_add_executable(get_map_client ros/src/get_map_client.cpp)
	
target_link_libraries(cob_3d_mapping_point_map_nodelets cob_3d_mapping_point_map pcl_registration pcl_segmentation)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

INSTALL(TARGETS cob_3d_mapping_point_map cob_3d_mapping_point_map_nodelets EXPORT cob_3d_mapping_point_map 
   LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_point_map/lib"
)
