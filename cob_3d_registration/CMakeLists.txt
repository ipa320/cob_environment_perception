cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_registration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cob_3d_features
  cob_3d_mapping_common
  cob_srvs
  cv_bridge
  message_generation
  nav_msgs
  nodelet
  pcl
  pcl_ros
  tf
  tf_conversions
)

#find_package(TinyXML REQUIRED)
find_package(VTK REQUIRED COMPONENTS vtkCommon)
#find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
#TODO: check for SSE

#generate_dynamic_reconfigure_options(cfg/plane_extraction_nodelet.cfg
#  cfg/segmentation_nodelet.cfg)

#######################################
## Declare ROS messages and services ##
#######################################


## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    EvaluationAssumptions.srv
    EvaluationResult.srv
    Parameterlist.srv
    RegistrationPCD.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages()

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
    cob_3d_registration
    cob_3d_registration_lib
  CATKIN_DEPENDS
    cob_3d_features
    cob_3d_mapping_common
    cob_srvs
    cv_bridge
    message_generation
    nav_msgs
    nodelet
    pcl
    pcl_ros
    tf
    tf_conversions
  DEPENDS
    VTK
    OpenCV
    Eigen
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_registration_lib
	common/src/registration_icp.cpp
	common/src/registration_icp_moments.cpp
	common/src/registration_icp_fpfh.cpp
	common/src/registration_icp_narf.cpp
	common/src/registration_icp_edges.cpp
#	common/src/registration_correspondence.cpp
	common/src/registration_info.cpp
)

add_library(cob_3d_registration
        ros/src/registration_nodelet.cpp
)

## Declare a cpp executable
#file(GLOB registration_SRC1
#    "common/src/fastslam/*/*.cpp"
#)
#add_executable(registration ros/src/registration_node.cpp common/src/fastslam/3DEnvReconstruction/EnvReconstructionControlFlow.cpp)
#add_executable(registration ros/src/registration_node.cpp common/src/fastslam/Filters/FastSLAM3DObjectModel.cpp)
#add_executable(registration ros/src/registration_node.cpp common/src/rgbdslam.cpp)
#add_executable(registration ros/src/registration_node.cpp ${registration_SRC1})
#add_executable(cob_3d_registration ros/src/registration_nodelet.cpp)
#rosbuild_add_executable(capture ros/src/capture.cpp)
#rosbuild_add_executable(filter ros/src/filter.cpp)
#rosbuild_add_executable(png2img ros/src/png_to_img.cpp)
#rosbuild_add_executable(ply2pcd ros/src/ply_to_pcd.cpp)
#rosbuild_add_executable(testing ros/src/test.cpp)
#rosbuild_add_executable(eval_svd ros/src/test_svd.cpp)


target_link_libraries(cob_3d_registration_lib 
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

target_link_libraries(cob_3d_registration 
  cob_3d_registration_lib
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

#target_link_libraries(png2img 
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#)

#target_link_libraries(registration ann  cob_vision_slam cob_sensor_fusion cob_camera_sensors cob_3d_mapping_features_omp)

set_target_properties(cob_3d_registration PROPERTIES COMPILE_FLAGS "-D__LINUX__") #${SSE_FLAGS}
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

catkin_add_gtest(test_frir ros/test/test_frir.cpp)




