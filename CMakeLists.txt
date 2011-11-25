cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Append to CPACK_SOURCE_IGNORE_FILES a semicolon-separated list of
# directories (or patterns, but directories should suffice) that should
# be excluded from the distro.  This is not the place to put things that
# should be ignored everywhere, like "build" directories; that happens in
# rosbuild/rosbuild.cmake.  Here should be listed packages that aren't
# ready for inclusion in a distro.
#
# This list is combined with the list in rosbuild/rosbuild.cmake.  Note
# that CMake 2.6 may be required to ensure that the two lists are combined
# properly.  CMake 2.4 seems to have unpredictable scoping rules for such
# variables.
#list(APPEND CPACK_SOURCE_IGNORE_FILES /core/experimental)

rosbuild_make_distribution(0.1.0)

#IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
set(CMAKE_INSTALL_PREFIX /opt/ros/electric/stacks/cob_environment_perception_intern)
#ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

include(InstallRequiredSystemLibraries)

set(CPACK_GENERATOR DEB)
SET(CPACK_SET_DESTDIR "ON")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "cob_environment_perception_intern")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER goa@ipa.fhg.de)
#set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Readme.txt")
#set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
#set(CPACK_INSTALL_CMAKE_PROJECTS cob_3d_mapping_point_map/build;cob_3d_mapping_point_map;ALL;cob_3d_mapping_point_map)
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
#set(CPACK_PACKAGE_VERSION_PATCH "0")

set(CPACK_PACKAGE_FILE_NAME cob_environment_perception_intern-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}-ubuntu-11.04)
SET(CPACK_PACKAGE_NAME cob_environment_perception_intern-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR})

set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "ros-electric-common (>= 1.6.1-s132),
         ros-electric-perception-pcl (>= 1.0.2-s132),
	ros-electric-nodelet-core,
	ros-electric-driver-common,
	ros-electric-geometry"
)
# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_point_map/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#message(STATUS ${CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES})
# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_point_map/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_point_map/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

INSTALL(FILES stack.xml
    DESTINATION "${CMAKE_INSTALL_PREFIX}"
)
#cob_3d_mapping_point_map
INSTALL(PROGRAMS cob_3d_mapping_point_map/lib/libcob_3d_mapping_point_map.so cob_3d_mapping_point_map/lib/libcob_3d_mapping_point_map_nodelets.so
    DESTINATION "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_point_map/lib"
)
INSTALL(FILES cob_3d_mapping_point_map/nodelet_plugins.xml cob_3d_mapping_point_map/CMakeLists.txt cob_3d_mapping_point_map/Makefile cob_3d_mapping_point_map/manifest.xml
    DESTINATION "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_point_map/"
)
INSTALL(FILES cob_3d_mapping_point_map/ros/launch/aggregate_point_map.launch
    DESTINATION "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_point_map/ros/launch"
)

#cob_3d_mapping_common
INSTALL(PROGRAMS cob_3d_mapping_common/bin/field_of_view_node cob_3d_mapping_common/bin/keyframe_detector_node
    DESTINATION "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_common/bin"
)
INSTALL(FILES cob_3d_mapping_common/CMakeLists.txt cob_3d_mapping_common/Makefile cob_3d_mapping_common/manifest.xml
    DESTINATION "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_common/"
)
INSTALL(FILES cob_3d_mapping_common/ros/launch/field_of_view.launch
    DESTINATION "${CMAKE_INSTALL_PREFIX}/cob_3d_mapping_common/ros/launch"
)


include(CPack)
