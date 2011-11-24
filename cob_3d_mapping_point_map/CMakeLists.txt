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

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

#IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
set(CMAKE_INSTALL_PREFIX /opt/ros/electric/stacks/cob_binary/cob_3d_mapping_point_map)
#ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

include(InstallRequiredSystemLibraries)

set(CPACK_GENERATOR DEB)
SET(CPACK_SET_DESTDIR "ON")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "cob_3d_mapping_point_map")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER goa@ipa.fhg.de)
#set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Readme.txt")
#set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
#set(CPACK_PACKAGE_VERSION_PATCH "0")

set(CPACK_PACKAGE_FILE_NAME cob_3d_mapping_point_map-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}-ubuntu-11.04)
SET(CPACK_PACKAGE_NAME cob_3d_mapping_point_map-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR})

set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "ros-electric-common (>= 1.6.1-s132),
         ros-electric-perception-pcl (>= 1.0.2-s132)"
)
# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#message(STATUS ${CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES})
# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")
#message(STATUS ${CMAKE_INSTALL_RPATH})
#message(STATUS ${CMAKE_INSTALL_PREFIX})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(cob_3d_mapping_point_map	
	common/src/point_map.cpp)
	
rosbuild_add_library(cob_3d_mapping_point_map_nodelets
	ros/src/point_map_nodelet.cpp)
	
rosbuild_add_executable(trigger_mapping_action_client ros/src/trigger_mapping_action_client.cpp)
rosbuild_add_executable(set_reference_map_client ros/src/set_reference_map.cpp)
rosbuild_add_executable(get_map_client ros/src/get_map_client.cpp)
	
target_link_libraries(cob_3d_mapping_point_map_nodelets cob_3d_mapping_point_map)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# link libraries
INSTALL(TARGETS cob_3d_mapping_point_map cob_3d_mapping_point_map_nodelets
    LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
)
INSTALL(TARGETS trigger_mapping_action_client set_reference_map_client get_map_client
    RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
)
INSTALL(FILES nodelet_plugins.xml CMakeLists.txt Makefile manifest.xml
    DESTINATION "${CMAKE_INSTALL_PREFIX}"
)
INSTALL(FILES docs/*
    DESTINATION "${CMAKE_INSTALL_PREFIX}/docs"
)

include(CPack)
