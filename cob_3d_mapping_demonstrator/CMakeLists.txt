cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include
    ${PROJECT_SOURCE_DIR}/ros/include)

exec_program("rosversion -d; echo" OUTPUT_VARIABLE ROS_VERSION)
message("${ROS_VERSION}")

if(ROS_VERSION MATCHES "fuerte")
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})

    ## I prefer the Qt signals and slots to avoid defining "emit", "slots",
    ## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
    add_definitions(-DQT_NO_KEYWORDS)

    ## Here we specify which header files need to be run through "moc",
    ## Qt's meta-object compiler.
    qt4_wrap_cpp(MOC_FILES
    ros/include/cob_3d_mapping_demonstrator/rviz_title.h
    ros/include/cob_3d_mapping_demonstrator/rviz_buttons.h)

    #add project executables
    rosbuild_add_library(cob_3d_mapping_demonstrator_rviz ros/src/rviz_buttons.cpp
        ros/src/rviz_title.cpp
        ${MOC_FILES})
    target_link_libraries(cob_3d_mapping_demonstrator_rviz  ${QT_LIBRARIES})
elseif(ROS_VERSION MATCHES "electric")
    # Add wx widgets
    find_package(wxWidgets REQUIRED)
    include(${wxWidgets_USE_FILE})
    include_directories( ${wxWidgets_INCLUDE_DIRS} )

    rosbuild_add_library(cob_3d_mapping_demonstrator_rviz ros/src/wx_rviz_buttons.cpp
        ros/src/wx_rviz_buttons_panel.cpp
        ros/src/wx_rviz_title.cpp
        ros/src/wx_rviz_logo.cpp
        ros/src/wx_rviz_logo_panel.cpp
        ros/src/wx_init.cpp)
else()
    message("Unsupported ROS version for visualization")
endif()


rosbuild_add_executable(${PROJECT_NAME} ros/src/MapDemonDriver.cpp common/src/MapDemonCtrl.cpp common/src/MapDemonCtrl_Maestro.cpp common/src/SerialDevice.cpp)
rosbuild_add_executable(testClient ros/test/test_client.cpp)
rosbuild_add_executable(testClientPOS ros/test/test_clientPOS.cpp)


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

#reactivate after switch to fuerte
#rosbuild_add_roslaunch_check(ros/launch)



