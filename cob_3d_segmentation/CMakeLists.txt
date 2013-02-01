cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
rosbuild_check_for_sse()

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include ${PROJECT_SOURCE_DIR}/ros/include)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

rosbuild_add_library(cob_3d_segmentation
	common/src/polygon_extraction.cpp
	common/src/segmentation.cpp # Old segmentation
	common/src/cluster_handler.cpp
	common/src/edge_handler.cpp
	common/src/cluster_graph_structure.cpp
	common/src/depth_segmentation.cpp
	common/src/cluster_classifier.cpp
	common/src/plane_extraction.cpp)

rosbuild_add_library(cob_3d_segmentation_nodelets
	ros/src/segmentation_all_in_one_nodelet.cpp
	ros/src/plane_extraction_nodelet.cpp
	ros/src/simple_segmentation_nodelet.cpp)

target_link_libraries(cob_3d_segmentation cob_3d_mapping_features cob_3d_mapping_features_omp)
target_link_libraries(cob_3d_segmentation_nodelets cob_3d_segmentation cob_3d_mapping_common)


rosbuild_add_executable(segmentation_node ros/src/segmentation_node.cpp)
target_link_libraries(segmentation_node cob_3d_segmentation)
rosbuild_add_executable(qppf_node ros/src/qppf_node.cpp)

rosbuild_add_executable(plane_extraction_action_client ros/src/plane_extraction_action_client.cpp)
target_link_libraries(plane_extraction_action_client ${OpenCV_LIBS})


# segmentation app:
rosbuild_add_executable(test_depth_segmentation common/apps/test_depth_segmentation.cpp)
target_link_libraries(test_depth_segmentation cob_3d_segmentation vtkCommon vtkRendering)
rosbuild_link_boost(test_depth_segmentation program_options)

# node for segmentation visualization:
rosbuild_add_executable(segmentation_visualizer ros/src/segmentation_visualizer.cpp)
target_link_libraries(segmentation_visualizer vtkCommon vtkRendering)

# Executables for profiling:
rosbuild_add_executable(profile_depth_seg common/apps/profile_depth_seg.cpp)
target_link_libraries(profile_depth_seg cob_3d_mapping_features cob_3d_mapping_features_omp cob_3d_segmentation)

rosbuild_add_executable(profile_plane_extraction common/apps/profile_plane_extraction.cpp)
target_link_libraries(profile_plane_extraction cob_3d_segmentation vtkCommon vtkFiltering ${OpenCV_LIBS})

rosbuild_add_compile_flags(cob_3d_segmentation -D__LINUX__ -DCOMPILE_NODELET)
rosbuild_add_roslaunch_check(ros/launch)

#######################  TESTING  #######################
rosbuild_add_executable(test_segmentation ros/test/test_segmentation.cpp)
rosbuild_add_compile_flags(test_segmentation ${SSE_FLAGS})
target_link_libraries(test_segmentation gtest)

rosbuild_add_executable(test_polygon_extraction ros/test/test_polygon_extraction.cpp)
target_link_libraries(test_polygon_extraction cob_3d_segmentation gtest)

rosbuild_add_executable(test_plane_extraction common/apps/test_plane_extraction.cpp)
target_link_libraries(test_plane_extraction cob_3d_segmentation vtkCommon vtkFiltering)
rosbuild_link_boost(test_plane_extraction program_options)

#rosbuild_add_executable(test_pcl_segmentation common/src/test_pcl_segmentation.cpp)
