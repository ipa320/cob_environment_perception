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

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/common/include
	${PROJECT_SOURCE_DIR}/ros/include)


find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
rosbuild_add_boost_directories()

rosbuild_add_library(cob_3d_mapping_tools
	common/src/cob_3d_mapping_tools/io.cpp
	common/src/cob_3d_mapping_tools/point_generator.cpp)

rosbuild_add_executable(transform_pointcloud ros/src/transform_pointcloud.cpp)
rosbuild_add_executable(concat_fields_node ros/src/concat_fields.cpp)
rosbuild_link_boost(concat_fields_node program_options signals)
rosbuild_add_executable(bag_to_pcd ros/src/bag_to_pcd.cpp)
rosbuild_link_boost(bag_to_pcd system filesystem)
rosbuild_add_executable(pointcloud_to_pcd ros/src/pointcloud_to_pcd.cpp)
rosbuild_add_executable(pointcloud_bag_delayer ros/src/pointcloud_bag_delayer.cpp)
rosbuild_link_boost(pointcloud_bag_delayer program_options)
rosbuild_add_executable(shape_bag_delayer ros/src/shape_bag_delayer.cpp)
rosbuild_link_boost(shape_bag_delayer program_options)

rosbuild_add_executable(create_point_cloud common/src/create_point_cloud.cpp)
rosbuild_add_executable(create_kitchen_point_cloud common/src/create_kitchen_point_cloud.cpp)

# extracts fpfh, pc and rsd feature values from a scene into separated pcd files:
rosbuild_add_executable(extract_feature_values common/src/extract_feature_values.cpp)
rosbuild_link_boost(extract_feature_values program_options)

# extracts pc from organized cloud and saves as ppm:
rosbuild_add_executable(extract_organized_curvatures common/src/extract_organized_curvatures.cpp)
target_link_libraries(extract_organized_curvatures cob_3d_mapping_tools cob_3d_mapping_features)
rosbuild_link_boost(extract_organized_curvatures program_options)

# some tools for fpfh training:
rosbuild_add_executable(merge_pcds common/src/merge_pcds.cpp)
rosbuild_link_boost(merge_pcds program_options)
rosbuild_add_executable(merge_knn_training_data common/src/merge_knn_training_data.cpp)
rosbuild_link_boost(merge_knn_training_data program_options)
rosbuild_add_executable(merge_fpfh_training_data common/src/merge_fpfh_training_data.cpp)
rosbuild_link_boost(merge_fpfh_training_data program_options)
rosbuild_add_executable(reduce_fpfh_training_data common/src/reduce_fpfh_training_data.cpp)
rosbuild_link_boost(reduce_fpfh_training_data program_options)
target_link_libraries(reduce_fpfh_training_data cob_3d_mapping_features)
rosbuild_add_executable(fpfh_primitives common/src/fpfh_primitives.cpp)
target_link_libraries(fpfh_primitives cob_3d_mapping_tools vtkCommon vtkFiltering vtkRendering)
rosbuild_link_boost(fpfh_primitives program_options)

# some tools for synthetic shape generation:
rosbuild_add_executable(create_synthetic_scene common/src/create_synthetic_scene.cpp)
target_link_libraries(create_synthetic_scene cob_3d_mapping_tools vtkCommon vtkFiltering)
rosbuild_link_boost(create_synthetic_scene program_options)
rosbuild_add_executable(create_synthetic_radii_test common/src/create_synthetic_radii_test.cpp)
target_link_libraries(create_synthetic_radii_test cob_3d_mapping_tools vtkCommon vtkFiltering)
rosbuild_link_boost(create_synthetic_radii_test program_options)

# some tools to convert point clouds:
rosbuild_add_executable(ppm_to_pcd common/src/ppm_to_pcd.cpp)
target_link_libraries(ppm_to_pcd cob_3d_mapping_tools)
rosbuild_link_boost(ppm_to_pcd program_options)
rosbuild_add_executable(pcd_to_ppm common/src/pcd_to_ppm.cpp)
target_link_libraries(pcd_to_ppm cob_3d_mapping_tools)
rosbuild_link_boost(pcd_to_ppm program_options)
rosbuild_add_executable(rgb_to_rgba common/src/rgb_to_rgba.cpp)
rosbuild_link_boost(rgb_to_rgba program_options)
rosbuild_add_executable(rgba_to_rgb common/src/rgba_to_rgb.cpp)
rosbuild_link_boost(rgba_to_rgb program_options)

# a test program for a new smoothing filter for kinect data :
rosbuild_add_executable(sav_gol common/src/sav_gol_smoothing_filter.cpp)
target_link_libraries(sav_gol vtkCommon vtkFiltering)
rosbuild_link_boost(sav_gol program_options)

# some tools to generate fpfh, pc and rsd test data and evaluate various parameters
rosbuild_add_executable(evaluate_test_data common/src/evaluate_test_data.cpp)
rosbuild_link_boost(evaluate_test_data program_options)
rosbuild_add_executable(generate_test_data common/src/generate_test_data.cpp)
rosbuild_add_compile_flags(generate_test_data -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
rosbuild_link_boost(generate_test_data program_options)

# plane segmentation evaluation tool:
rosbuild_add_executable(evaluate_plane_segmentation common/src/evaluate_plane_segmentation.cpp)
target_link_libraries(evaluate_plane_segmentation cob_3d_mapping_tools)
rosbuild_link_boost(evaluate_plane_segmentation program_options)

# Gui:
rosbuild_add_executable(test_gui common/src/test_wx_gui.cpp)
target_link_libraries(test_gui ${wxWidgets_LIBRARIES} cob_3d_mapping_tools)
rosbuild_link_boost(test_gui program_options signals)

rosbuild_add_executable(test_hsv_tool common/src/test_hsv_tool.cpp)
target_link_libraries(test_hsv_tool ${wxWidgets_LIBRARIES} cob_3d_mapping_tools)
rosbuild_link_boost(test_hsv_tool program_options signals)

rosbuild_add_executable(bag_gui ros/src/bag_gui.cpp)
target_link_libraries(bag_gui ${wxWidgets_LIBRARIES} cob_3d_mapping_tools)

rosbuild_add_executable(gui_normals common/src/gui_normals.cpp)
target_link_libraries(gui_normals ${wxWidgets_LIBRARIES} cob_3d_mapping_tools)
rosbuild_link_boost(gui_normals signals)

rosbuild_add_executable(noise_gen ros/src/noise_gen.cpp)
rosbuild_link_boost(noise_gen signals)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_roslaunch_check(ros/launch)
