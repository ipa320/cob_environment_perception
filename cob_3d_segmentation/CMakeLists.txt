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
rosbuild_check_for_sse()

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include ${PROJECT_SOURCE_DIR}/ros/include)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(cob_3d_segmentation  ros/src/segmentation_node.cpp)
rosbuild_add_executable(segmentation_node ros/src/segmentation_node.cpp)

rosbuild_add_compile_flags(cob_3d_segmentation -D__LINUX__ -DCOMPILE_NODELET)

rosbuild_add_roslaunch_check(ros/launch)

#######################  TESTING  #######################
rosbuild_add_executable(test_segmentation ros/test/test_segmentation.cpp)
rosbuild_add_compile_flags(test_segmentation ${SSE_FLAGS})
target_link_libraries(test_segmentation gtest)

rosbuild_add_executable(test_polygon_extraction ros/test/test_polygon_extraction.cpp)
target_link_libraries(test_polygon_extraction gtest)
