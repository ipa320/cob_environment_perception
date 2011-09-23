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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


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
rosbuild_add_executable(extract_features ros/src/extract_features.cpp)
#rosbuild_add_executable(test_range ros/src/test_range_edges.cpp)
rosbuild_add_executable(norm_time ros/src/norm_time.cpp)


#rosbuild_add_executable(test_icp ros/src/test_icp.cpp)
rosbuild_add_executable(bag_to_pcd ros/src/tools/bag_to_pcd.cpp)
rosbuild_add_executable(pointcloud_to_pcd ros/src/tools/pointcloud_to_pcd.cpp)

rosbuild_add_library(cob_env_model_filters
	common/src/filters/amplitude_filter.cpp
	common/src/filters/intensity_filter.cpp
	common/src/filters/jump_edge_filter.cpp)
	
rosbuild_add_library(cob_env_model_map	
	common/src/map/gpc.c
	common/src/map/feature_map.cpp
	common/src/map/point_map.cpp)
	
rosbuild_add_library(cob_env_model 
	common/src/features/range_image_border_extractor.cpp
	common/src/features/boundary.cpp
	common/src/features/plane_extraction.cpp
	common/src/table_object_cluster.cpp)

rosbuild_add_library(cob_env_model_nodelets
    ros/src/table_object_cluster_nodelet.cpp
	ros/src/plane_extraction_nodelet.cpp
	ros/src/aggregate_point_map.cpp
	ros/src/filters/amplitude_filter.cpp
	ros/src/filters/intensity_filter.cpp
	ros/src/filters/jump_edge_filter.cpp
	ros/src/filters/speckle_filter.cpp)

#rosbuild_add_executable(aggregate_point_map
#	ros/src/aggregate_point_map.cpp
#	common/src/point_map.cpp)
rosbuild_add_executable(field_of_view_node ros/src/field_of_view.cpp)
rosbuild_add_executable(concat_fields_node ros/src/tools/concat_fields.cpp)
rosbuild_add_executable(feature_map_node ros/src/feature_map_node.cpp)
rosbuild_add_executable(feature_map common/src/map/feature_map.cpp)
rosbuild_add_executable(create_point_cloud common/src/tools/create_point_cloud.cpp)
rosbuild_add_executable(create_kitchen_point_cloud common/src/tools/create_kitchen_point_cloud.cpp)
rosbuild_add_executable(plane_extraction_action_client ros/src/plane_extraction_action_client.cpp)
rosbuild_add_executable(table_object_cluster_action_client ros/src/table_object_cluster_action_client.cpp)
rosbuild_add_executable(trigger_mapping_action_client ros/src/trigger_mapping_action_client.cpp)
rosbuild_add_executable(eval_mapping common/src/tools/eval_mapping.cpp)
#rosbuild_add_library(gpc common/src/gpc.c)
#rosbuild_add_executable(test_fov common/src/test_fov.cpp)
#rosbuild_add_library(nodelet_cpc_publisher ros/src/cpc_publisher.cpp
#ros/src/cpc_visualizer.cpp)
#rosbuild_add_executable(cpc_publisher ros/src/cpc_publisher_main.cpp)
#rosbuild_add_executable(cpc_visualizer ros/src/cpc_visualizer_main.cpp)


# add compile flag
rosbuild_add_compile_flags(extract_features -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)


# add compile flag
#target_link_libraries(cob_env_model gpc)
target_link_libraries(extract_features cob_env_model)
target_link_libraries(feature_map cob_env_model_map)
target_link_libraries(feature_map_node cob_env_model_map)
target_link_libraries(cob_env_model_nodelets cob_env_model cob_env_model_filters cob_env_model_map)
#target_link_libraries (planar_segmentation ${PCL_IO_LIBRARY} ${PCL_FILTERS_LIBRARY} ${PCL_SEGMENTATION_LIBRARY})
#target_link_libraries(test_fov cob_env_model)
#target_link_libraries(env_model ann glut mesasr)
#target_link_libraries(env_model_node glut)
#target_link_libraries(cpc_publisher nodelet_cpc_publisher)
#target_link_libraries(test_cpc_visualizer nodelet_cpc_publisher)
#target_link_libraries(cpc_visualizer nodelet_cpc_publisher)
target_link_libraries(norm_time cob_env_model)

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
