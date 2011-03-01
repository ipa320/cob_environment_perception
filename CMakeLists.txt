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
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include
${PROJECT_SOURCE_DIR}/ros/include)

# add project executable
#rosbuild_add_executable(env_model common/src/EnvReconstructionMain.cpp
#	common/src/EnvReconstructionControlFlow.cpp
#	common/src/CuiEnvReconstruction.cpp)

#rosbuild_add_executable(env_model_node ros/src/cob_env_model.cpp)
#rosbuild_add_executable(data_acquisition_node ros/src/data_acquisition.cpp)
#rosbuild_add_executable(icp_registration_node ros/src/icp_registration.cpp)
#rosbuild_add_executable(point_cloud_filter_node ros/src/point_cloud_filter.cpp)
#rosbuild_add_executable(extract_features_node ros/src/extract_features.cpp)
rosbuild_add_executable(test_icp ros/src/test_icp.cpp)
rosbuild_add_executable(bag_to_pcd ros/src/bag_to_pcd.cpp)
rosbuild_add_library(cob_env_model ros/src/detect_tables.cpp
	ros/src/aggregate_point_map.cpp
	ros/src/point_cloud_filter.cpp
	ros/src/segment_field_of_view.cpp
	ros/src/cpc_point.cpp)
rosbuild_add_executable(test_fov common/src/test_fov.cpp)
#rosbuild_add_library(nodelet_cpc_publisher ros/src/cpc_publisher.cpp
#	ros/src/cpc_visualizer.cpp)
#rosbuild_add_executable(test_cpc_visualizer ros/src/test_cpc_visualizer.cpp)
#rosbuild_add_executable(cpc_publisher ros/src/cpc_publisher_main.cpp)
#rosbuild_add_executable(cpc_visualizer ros/src/cpc_visualizer_main.cpp)



# add compile flag
#rosbuild_add_compile_flags(env_model -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_compile_flags(env_model_node -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_compile_flags(data_acquisition_node -D__LINUX__)
#rosbuild_add_compile_flags(icp_registration_node -D__LINUX__ -D__COB_ROS__)
rosbuild_add_compile_flags(cob_env_model -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_compile_flags(extract_features_node -D__LINUX__ -D__COB_ROS__)


# add compile flag
target_link_libraries(test_fov cob_env_model)
#target_link_libraries(env_model ann glut mesasr)
#target_link_libraries(env_model_node glut)
#target_link_libraries(cpc_publisher nodelet_cpc_publisher)
#target_link_libraries(test_cpc_visualizer nodelet_cpc_publisher)
#target_link_libraries(cpc_visualizer nodelet_cpc_publisher)

