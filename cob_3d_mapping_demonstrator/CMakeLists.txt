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
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include
    ${PROJECT_SOURCE_DIR}/ros/include)

#add project executbles(example examples/example.cpp)
rosbuild_add_library(cob_3d_mapping_demonstrator_rviz ros/src/rviz_buttons.cpp
    ros/src/rviz_buttons_panel.cpp
    ros/src/rviz_title.cpp
    ros/src/rviz_logo.cpp
    ros/src/rviz_logo_panel.cpp
    ros/src/init.cpp)
rosbuild_add_executable(${PROJECT_NAME} ros/src/MapDemonDriver.cpp common/src/MapDemonCtrl.cpp common/src/SerialDevice.cpp)
rosbuild_add_executable(testClient ros/test/test_client.cpp)
rosbuild_add_executable(testClientPOS ros/test/test_clientPOS.cpp)

# Add wx widgets
find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

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
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_roslaunch_check(ros/launch)



