cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type. Options are:
# Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
# Debug : w/ debug symbols, w/o optimization
# Release : w/o debug symbols, w/ optimization
# RelWithDebInfo : w/ debug symbols, w/ optimization
# MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)
    #/usr/include/vtk-5.8)
rosbuild_add_boost_directories()


## include vtk
#include_directories("/usr/include/vtk-5.6" )
## include pcl directly
#include(/opt/ros/fuerte/include/pcl-1.5)
#include(/opt/ros/fuerte/include/pcl-1.5)
#include(/opt/ros/fuerte/stacks/geometry/tf/include)

#find_package(Eigen REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})
find_package(OpenCV REQUIRED)
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(cob_3d_features
  common/src/fast_edge_estimation_3d.cpp
  common/src/edge_estimation_3d.cpp
  common/src/edge_estimation_2d.cpp
  common/src/edge_extraction.cpp
  common/src/knn_classifier.cpp
  common/src/most_discriminating_data_points.cpp
  common/src/organized_curvature_estimation.cpp
  common/src/organized_normal_estimation.cpp)

target_link_libraries(cob_3d_features ${OpenCV_LIBS})

rosbuild_add_library(cob_3d_features_omp
  common/src/organized_curvature_estimation_omp.cpp
  common/src/organized_normal_estimation_omp.cpp
  common/src/fast_edge_estimation_3d_omp.cpp)


rosbuild_add_openmp_flags(cob_3d_features_omp)

rosbuild_add_boost_directories()


rosbuild_add_executable(test_organized_curvature_estimation common/test/test_organized_curvature_estimation.cpp)
target_link_libraries(test_organized_curvature_estimation cob_3d_features cob_3d_features_omp vtkCommon vtkFiltering ${OpenCV_LIBS})
rosbuild_link_boost(test_organized_curvature_estimation program_options)


# Georg's old test program for edge estimation and segmentation:
#rosbuild_add_executable(test_edge_extraction common/apps/test_edge_extraction.cpp)
#rosbuild_add_compile_flags(test_edge_extraction -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
#target_link_libraries(test_edge_extraction cob_3d_mapping_features)

# Test Surface Classification using organized principal curvatures
rosbuild_add_executable(test_surface_class_estimation common/test/test_surface_class_estimation.cpp)
target_link_libraries(test_surface_class_estimation cob_3d_features cob_3d_features_omp vtkCommon vtkFiltering vtkRendering ${OpenCV_LIBS})
rosbuild_link_boost(test_surface_class_estimation program_options)

# Test program for 2D (on RGB or depth) and 3D EdgeEstimation + visualization
rosbuild_add_executable(test_edge_estimation common/test/test_edge_estimation.cpp)
rosbuild_add_compile_flags(test_edge_estimation -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
target_link_libraries(test_edge_estimation cob_3d_features vtkCommon vtkFiltering ${OpenCV_LIBS})
rosbuild_link_boost(test_edge_estimation program_options)
rosbuild_add_executable(test_edge_sharpening common/test/test_edge_sharpening.cpp)
target_link_libraries(test_edge_sharpening cob_3d_features vtkCommon vtkRendering)
rosbuild_link_boost(test_edge_sharpening program_options)
rosbuild_add_executable(test_invariant_feature common/test/test_invariant_feature.cpp)

# Profiling old EdgeEstimation and FastEdgeEstimation
# Only computation, no visualization or output.
rosbuild_add_executable(profile_oce common/test/profile_oce.cpp)
rosbuild_add_executable(profile_fee_omp common/test/profile_fee_omp.cpp)
rosbuild_add_executable(profile_fee common/test/profile_fee.cpp)
rosbuild_add_executable(profile_ee common/test/profile_ee.cpp)
target_link_libraries(profile_oce cob_3d_features cob_3d_features_omp vtkCommon vtkFiltering ${OpenCV_LIBS})
target_link_libraries(profile_fee_omp cob_3d_features cob_3d_features_omp vtkCommon vtkFiltering ${OpenCV_LIBS})
target_link_libraries(profile_fee cob_3d_features vtkCommon vtkFiltering ${OpenCV_LIBS})
target_link_libraries(profile_ee cob_3d_features vtkCommon vtkFiltering ${OpenCV_LIBS})

rosbuild_add_executable(profile_ne common/test/profile_ne.cpp)
target_link_libraries(profile_ne cob_3d_features_omp cob_3d_features)


#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# rostest
#rosbuild_add_roslaunch_check(ros/launch)
