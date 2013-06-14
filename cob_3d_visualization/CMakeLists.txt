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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

#rosbuild_add_library(cob_3d_visualization src/table_visualization.cpp)

rosbuild_add_executable(shape_visualization src/shape_visualization.cpp
    src/shape_marker.cpp)
#rosbuild_add_executable(table_visualization src/table_visualization.cpp
#						src/table_marker.cpp)
rosbuild_add_executable(test_shape_array test/test_shape_array.cpp)
#target_link_libraries(table_visualization cob_3d_visualization)

#rosbuild_add_library(cob_3d_visualization  src/table_visualization.cpp)
#rosbuild_add_executable(uv_angle_test test/uv_angle_test.cpp)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_roslaunch_check(launch)
