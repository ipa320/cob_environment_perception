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
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/common/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add project executable
rosbuild_add_executable(cob_env_model common/src/EnvReconstructionMain.cpp
	common/src/EnvReconstructionControlFlow.cpp
	common/src/CuiEnvReconstruction.cpp
	common/src/EKF2DLandmark.cpp
	common/src/AbstractEKF.cpp
	common/src/FastSLAMParticle.cpp
	common/src/FastSLAM.cpp
	common/src/MeasPDF2DLandmark.cpp
	common/src/MeasModel2DLandmark.cpp
	common/src/SysPDF2DOdometry.cpp
	common/src/KdTreeDataAssociation.cpp
	common/src/kdTree.cpp
	common/src/RobotSim.cpp
	common/src/CameraSensorsControlFlow.cpp
	common/src/PointCloudRenderer.cpp
	common/src/zpr.cpp
	common/src/SURFDetector.cpp
	common/src/AbstractFeatureVector.cpp
	common/src/BlobFeatureVector.cpp
	common/src/AbstractFeatureList.cpp
	common/src/BlobFeatureList.cpp
	common/src/AbstractFeature.cpp
	common/src/BlobFeature.cpp)

rosbuild_add_executable(cob_env_model_ros ros/src/cob_env_model.cpp
	common/src/EnvReconstructionControlFlow.cpp
	common/src/CuiEnvReconstruction.cpp
	common/src/EKF2DLandmark.cpp
	common/src/AbstractEKF.cpp
	common/src/FastSLAMParticle.cpp
	common/src/FastSLAM.cpp
	common/src/MeasPDF2DLandmark.cpp
	common/src/MeasModel2DLandmark.cpp
	common/src/SysPDF2DOdometry.cpp
	common/src/KdTreeDataAssociation.cpp
	common/src/kdTree.cpp
	common/src/RobotSim.cpp
	common/src/CameraSensorsControlFlow.cpp
	common/src/PointCloudRenderer.cpp
	common/src/zpr.cpp
	common/src/SURFDetector.cpp
	common/src/AbstractFeatureVector.cpp
	common/src/BlobFeatureVector.cpp
	common/src/AbstractFeatureList.cpp
	common/src/BlobFeatureList.cpp
	common/src/AbstractFeature.cpp
	common/src/BlobFeature.cpp)


# add compile flag
rosbuild_add_compile_flags(cob_env_model -D__LINUX__ -D__COB_ROS__)
rosbuild_add_compile_flags(cob_env_model_ros -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_executable(swiss_ranger_show ros/src/swiss_ranger_show.cpp)
#rosbuild_add_executable(color_camera ros/src/color_camera.cpp)
#rosbuild_add_executable(color_camera_show ros/src/color_camera_show.cpp)

# add compile flag
#rosbuild_add_compile_flags(swiss_ranger -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_compile_flags(SensorFusion -D__LINUX__ -D__COB_ROS__)
target_link_libraries(cob_env_model ann glut mesasr)
target_link_libraries(cob_env_model_ros ann glut mesasr)
