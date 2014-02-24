cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cob_3d_features
  cob_3d_fov_segmentation
  cob_3d_mapping_common
  cob_3d_segmentation
  pcl_conversions
  pcl_ros
  rosbag
)

#find_package(TinyXML REQUIRED)
find_package(VTK REQUIRED COMPONENTS vtkCommon vtkFiltering vtkRendering)
find_package(Boost REQUIRED COMPONENTS filesystem program_options signals system)
find_package(OpenCV REQUIRED)
find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    common/include
    ros/include
  LIBRARIES
    cob_3d_mapping_tools
  CATKIN_DEPENDS
    cob_3d_features
    cob_3d_fov_segmentation
    cob_3d_mapping_common
    cob_3d_segmentation
    pcl_conversions
    pcl_ros
    rosbag
  DEPENDS
    VTK
    Boost
    OpenCV
    wxWidgets
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${wxWidgets_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_mapping_tools
	common/src/cob_3d_mapping_tools/io.cpp
	common/src/cob_3d_mapping_tools/point_generator.cpp
)

add_executable(concat_fields_node ros/src/concat_fields.cpp)
add_executable(bag_to_pcd ros/src/bag_to_pcd.cpp)
add_executable(pointcloud_to_pcd ros/src/pointcloud_to_pcd.cpp)
add_executable(pointcloud_bag_delayer ros/src/pointcloud_bag_delayer.cpp)
add_executable(shape_bag_delayer ros/src/shape_bag_delayer.cpp)
#add_executable(voxel_grid common/src/voxel_grid.cpp)
add_executable(create_point_cloud common/src/create_point_cloud.cpp)
add_executable(create_kitchen_point_cloud common/src/create_kitchen_point_cloud.cpp)
# extracts fpfh, pc and rsd feature values from a scene into separated pcd files:
add_executable(extract_feature_values common/src/extract_feature_values.cpp)
# extracts pc from organized cloud and saves as ppm:
add_executable(extract_organized_curvatures common/src/extract_organized_curvatures.cpp)
# some tools for fpfh training:
add_executable(merge_pcds common/src/merge_pcds.cpp)
add_executable(merge_knn_training_data common/src/merge_knn_training_data.cpp)
add_executable(merge_fpfh_training_data common/src/merge_fpfh_training_data.cpp)
add_executable(reduce_fpfh_training_data common/src/reduce_fpfh_training_data.cpp)
add_executable(fpfh_primitives common/src/fpfh_primitives.cpp)
# some tools for synthetic shape generation:
add_executable(create_synthetic_scene common/src/create_synthetic_scene.cpp)
add_executable(create_synthetic_radii_test common/src/create_synthetic_radii_test.cpp)
# some tools to convert point clouds:
add_executable(ppm_to_pcd common/src/ppm_to_pcd.cpp)
add_executable(pcd_to_ppm common/src/pcd_to_ppm.cpp)
add_executable(rgb_to_rgba common/src/rgb_to_rgba.cpp)
add_executable(rgba_to_rgb common/src/rgba_to_rgb.cpp)
# a test program for a new smoothing filter for kinect data :
add_executable(sav_gol common/src/sav_gol_smoothing_filter.cpp)
# some tools to generate fpfh, pc and rsd test data and evaluate various parameters
add_executable(evaluate_test_data common/src/evaluate_test_data.cpp)
add_executable(generate_test_data common/src/generate_test_data.cpp)
# plane segmentation evaluation tool:
#add_executable(evaluate_plane_segmentation common/src/evaluate_plane_segmentation.cpp)
# Gui:
#add_executable(test_gui common/src/test_wx_gui.cpp)
add_executable(test_hsv_tool common/src/test_hsv_tool.cpp)
add_executable(bag_gui ros/src/bag_gui.cpp)
add_executable(gui_normals common/src/gui_normals.cpp)
add_executable(noise_gen ros/src/noise_gen.cpp)
add_executable(test_perspective_projection common/src/test_perspective_projection.cpp)

target_link_libraries(concat_fields_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(bag_to_pcd
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(pointcloud_to_pcd
  ${catkin_LIBRARIES}
)

target_link_libraries(pointcloud_bag_delayer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(shape_bag_delayer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(create_point_cloud
  ${catkin_LIBRARIES}
)

target_link_libraries(create_kitchen_point_cloud
  ${catkin_LIBRARIES}
)

target_link_libraries(extract_feature_values
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(extract_organized_curvatures
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(merge_pcds
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(merge_knn_training_data
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(merge_fpfh_training_data
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(reduce_fpfh_training_data
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(fpfh_primitives
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${VTK_LIBRARIES}
)

target_link_libraries(create_synthetic_scene
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${VTK_LIBRARIES}
)

target_link_libraries(create_synthetic_radii_test
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${VTK_LIBRARIES}
)

target_link_libraries(ppm_to_pcd
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(pcd_to_ppm
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(rgb_to_rgba
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(rgba_to_rgb
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(sav_gol
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${VTK_LIBRARIES}
)

target_link_libraries(evaluate_test_data
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(generate_test_data
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


#target_link_libraries(evaluate_plane_segmentation
#  cob_3d_mapping_tools
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#target_link_libraries(test_gui
#  cob_3d_mapping_tools
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  ${wxWidgets_LIBRARIES}
#)

target_link_libraries(test_hsv_tool
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${wxWidgets_LIBRARIES}
)

target_link_libraries(bag_gui
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${wxWidgets_LIBRARIES}
)

target_link_libraries(gui_normals
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${wxWidgets_LIBRARIES}
)

target_link_libraries(noise_gen
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(test_perspective_projection
  ${catkin_LIBRARIES}
)

set_target_properties(evaluate_test_data PROPERTIES COMPILE_FLAGS -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
set_target_properties(generate_test_data PROPERTIES COMPILE_FLAGS -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

