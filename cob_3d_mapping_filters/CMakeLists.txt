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
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

#find_package(PCL 1.5.1 REQUIRED COMPONENTS
#common io kdtree search surface features filters)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})



rosbuild_add_library(cob_3d_mapping_filters
	common/src/amplitude_filter.cpp
	common/src/intensity_filter.cpp
	common/src/jump_edge_filter.cpp
	common/src/speckle_filter.cpp)

rosbuild_add_library(cob_3d_mapping_filter_nodelets
	ros/src/amplitude_filter.cpp
	ros/src/intensity_filter.cpp
	ros/src/smoothing_filter.cpp
	ros/src/jump_edge_filter.cpp
	ros/src/speckle_filter.cpp)

#rosbuild_add_executable(test_amplitude_filter common/test/test_amplitude_filter.cpp)
#target_link_libraries(test_amplitude_filter cob_3d_mapping_filters)
rosbuild_add_executable(test_intensity_filter common/test/test_intensity_filter.cpp)
target_link_libraries(test_intensity_filter cob_3d_mapping_filters)
#rosbuild_add_executable(test_jump_edge_filter common/test/test_jump_edge_filter.cpp)
#target_link_libraries(test_jump_edge_filter cob_3d_mapping_filters)
#rosbuild_add_executable(test_speckle_filter common/test/test_speckle_filter.cpp)
#target_link_libraries(test_speckle_filter cob_3d_mapping_filters)
#rosbuild_add_executable(profile_filters common/test/profile_filters.cpp)
#target_link_libraries(profile_filters cob_3d_mapping_filters)

rosbuild_add_executable(test_statistical_outlier_removal_filter common/test/test_statistical_outlier_removal_filter.cpp)
target_link_libraries(test_statistical_outlier_removal_filter ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_KDTREE_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_SURFACE_LIBRARY} ${PCL_FEATURES_LIBRARY}  ${PCL_FILTERS_LIBRARY})
#rosbuild_link_boost(test_statistical_outlier_removal_filter program_options)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_roslaunch_check(ros/launch)

