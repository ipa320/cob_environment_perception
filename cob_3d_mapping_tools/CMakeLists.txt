cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cob_3d_features
  cob_3d_fov_segmentation
  cob_3d_mapping_common
  cob_3d_segmentation
  pcl_conversions
  pcl_ros
  rosbag
  sensor_msgs
)

#find_package(TinyXML REQUIRED)
find_package(VTK REQUIRED COMPONENTS vtkCommon vtkFiltering vtkRendering)
find_package(Boost REQUIRED COMPONENTS filesystem program_options signals system)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
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
  LIBRARIES
    cob_3d_mapping_tools
  CATKIN_DEPENDS
    sensor_msgs
  DEPENDS
    Boost
    Eigen
    PCL
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
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${wxWidgets_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_mapping_tools
#	common/src/cob_3d_mapping_tools/io.cpp
	common/src/cob_3d_mapping_tools/point_generator.cpp
)

add_executable(concat_fields_node ros/src/concat_fields.cpp)
add_executable(bag_to_pcd ros/src/bag_to_pcd.cpp)
add_executable(pointcloud_bag_delayer ros/src/pointcloud_bag_delayer.cpp)
add_executable(shape_bag_delayer ros/src/shape_bag_delayer.cpp)

# some tools for fpfh training:
add_executable(merge_pcds common/src/merge_pcds.cpp)

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

# Gui:
#add_executable(test_gui common/src/test_wx_gui.cpp)
add_executable(bag_gui ros/src/bag_gui.cpp)
add_executable(gaussian_noise_generator_node ros/src/noise_gen.cpp)


target_link_libraries(concat_fields_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(bag_to_pcd
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(pointcloud_bag_delayer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(shape_bag_delayer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(merge_pcds
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
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

#target_link_libraries(test_gui
#  cob_3d_mapping_tools
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  ${wxWidgets_LIBRARIES}
#)

target_link_libraries(bag_gui
  cob_3d_mapping_tools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${wxWidgets_LIBRARIES}
)

target_link_libraries(gaussian_noise_generator_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)



