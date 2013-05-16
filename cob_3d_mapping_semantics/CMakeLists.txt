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
rosbuild_gensrv()
rosbuild_add_library(cob_3d_mapping_semantics common/src/table_extraction.cpp
    common/src/supporting_plane_extraction.cpp
    common/src/structure_extraction.cpp)

#rosbuild_add_executable(semantic_extraction common/src/semantic_extraction.cpp)
rosbuild_add_executable(table_extraction_node ros/src/table_extraction_node.cpp)
rosbuild_add_executable(supporting_plane_extraction_node ros/src/supporting_plane_extraction_node.cpp)
rosbuild_add_executable(structure_extraction_node ros/src/structure_extraction_node.cpp)
rosbuild_add_executable(get_tables_client ros/src/get_tables_client.cpp)
#rosbuild_add_executable(move_to_table_node ros/src/move_to_table_node.cpp)

target_link_libraries(table_extraction_node cob_3d_mapping_semantics)
target_link_libraries(supporting_plane_extraction_node cob_3d_mapping_semantics)
target_link_libraries(structure_extraction_node cob_3d_mapping_semantics)
#target_link_libraries(map_client cob_3d_mapping_semantics)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_roslaunch_check(ros/launch)
