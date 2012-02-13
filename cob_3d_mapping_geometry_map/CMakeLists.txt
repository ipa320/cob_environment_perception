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

rosbuild_add_library(gpc common/src/gpc.c)
rosbuild_add_library(geometry_map common/src/feature_map.cpp)
rosbuild_add_executable(feature_map common/src/feature_map.cpp)
rosbuild_add_executable(feature_map_node ros/src/feature_map_node.cpp)
rosbuild_add_executable(eval_feature_map common/test/eval_mapping.cpp)
rosbuild_add_executable(get_map_client ros/src/get_map_client.cpp)

#rosbuild_add_executable(geometry_map_visualisation common/src/vis/geometry_map_visualisation.cpp)
rosbuild_add_executable(geometry_map common/src/geometry_map.cpp)
rosbuild_add_executable(dynamic_geometry_map_node ros/src/dynamic_geometry_map_node.cpp)
rosbuild_add_executable(geometry_map_node ros/src/geometry_map_node.cpp)
rosbuild_add_executable(eval_feature_map common/test/eval_mapping.cpp)
rosbuild_add_executable(geometry_test_map common/test/geometry_map_test.cpp)
rosbuild_add_executable(get_map_client ros/src/get_map_client.cpp)

target_link_libraries(dynamic_geometry_map_node gpc geometry_map_lib geometry_map_vis_lib)
target_link_libraries(geometry_map_node gpc geometry_map_lib geometry_map_vis_lib)
target_link_libraries(geometry_map gpc geometry_map_vis_lib )
target_link_libraries(geometry_test_map  geometry_map_vis_lib geometry_map_lib test_planes gpc)

#target_link_libraries(geometry_map_visualisation gpc geometry_map_lib)



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
