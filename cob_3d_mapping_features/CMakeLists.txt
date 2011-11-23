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

rosbuild_add_library(cob_3d_mapping_features 
	common/src/range_image_border_extractor.cpp
	common/src/edge_estimation_3d.cpp
	common/src/edge_estimation_2d.cpp
	common/src/plane_extraction.cpp
	common/src/knn_classifier.cpp)

rosbuild_add_library(cob_3d_mapping_features_nodelets
	ros/src/plane_extraction_nodelet.cpp)

rosbuild_add_executable(test_edge_extraction 
	common/src/segmentation.cpp
	common/test/test_edge_extraction.cpp)
rosbuild_add_executable(test_segmentation 
	common/src/segmentation.cpp
	common/test/test_segmentation.cpp)
rosbuild_add_executable(test_plane_extraction common/src/plane_extraction.cpp)
rosbuild_add_executable(plane_extraction_action_client ros/src/plane_extraction_action_client.cpp)
rosbuild_add_executable(fpfh_svm_trainer common/test/fpfh_svm_trainer.cpp)
rosbuild_add_executable(feature_evaluation common/test/feature_evaluation.cpp)
rosbuild_add_executable(test_edge_estimation common/test/test_edge_estimation.cpp)

rosbuild_add_compile_flags(test_edge_extraction -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
rosbuild_add_compile_flags(test_edge_estimation -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
rosbuild_add_compile_flags(feature_evaluation -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

target_link_libraries(test_edge_extraction cob_3d_mapping_features)
target_link_libraries(test_edge_estimation cob_3d_mapping_features)
target_link_libraries(feature_evaluation cob_3d_mapping_features)
target_link_libraries(cob_3d_mapping_features_nodelets cob_3d_mapping_features)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
