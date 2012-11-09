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
rosbuild_check_for_sse()

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include ${PROJECT_SOURCE_DIR}/ros/include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#find_package(Eigen REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})
find_package(OpenCV REQUIRED)
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})



#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

file(GLOB registration_SRC1
    "common/src/fastslam/*/*.cpp"
)
#rosbuild_add_executable(registration ros/src/registration_node.cpp common/src/fastslam/3DEnvReconstruction/EnvReconstructionControlFlow.cpp)
#rosbuild_add_executable(registration ros/src/registration_node.cpp common/src/fastslam/Filters/FastSLAM3DObjectModel.cpp)

#rosbuild_add_executable(registration ros/src/registration_node.cpp common/src/rgbdslam.cpp)
#rosbuild_add_executable(registration ros/src/registration_node.cpp ${registration_SRC1})
#rosbuild_add_executable(registration ros/src/registration_node.cpp)

rosbuild_add_library(cob_3d_registration
	ros/src/registration_nodelet.cpp)
target_link_libraries(cob_3d_registration vtkCommon )


rosbuild_add_executable(capture ros/src/capture.cpp)
rosbuild_add_executable(filter ros/src/filter.cpp)
rosbuild_add_executable(png2img ros/src/png_to_img.cpp)
target_link_libraries(png2img ${OpenCV_LIBS})
rosbuild_add_executable(ply2pcd ros/src/ply_to_pcd.cpp)
#rosbuild_add_executable(testing ros/src/test.cpp)
rosbuild_add_executable(eval_svd ros/src/test_svd.cpp)

rosbuild_add_gtest(test_frir ros/test/test_frir.cpp)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

rosbuild_add_executable(example ros/src/example2.cpp)
#rosbuild_add_executable(frir_freehand ros/src/frir_freehand.cpp)

rosbuild_add_compile_flags(cob_3d_registration -D__LINUX__ ${SSE_FLAGS})
#rosbuild_add_compile_flags(frir_freehand -D__LINUX__ ${SSE_FLAGS})

#target_link_libraries(registration ann  cob_vision_slam cob_sensor_fusion cob_camera_sensors cob_3d_mapping_features_omp)
target_link_libraries(cob_3d_registration  cob_3d_mapping_features_omp)

rosbuild_add_roslaunch_check(ros/launch)
