cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

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

#find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
#include(${QT_USE_FILE})

#add_definitions(-DQT_NO_KEYWORDS)

#pkg_check_modules(OGRE OGRE)
#include_directories( ${OGRE_INCLUDE_DIRS} )
#link_directories( ${OGRE_LIBRARY_DIRS} )


#uncomment if you have defined messages
rosbuild_genmsg()

#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_library(cob_3d_mapping_rviz_plugins #src/shape_display.cpp
#    src/shape_base.cpp
#    src/shape_marker.cpp
    src/polypartition.cpp)
#    src/init.cpp)
#rosbuild_add_executable(cob_3d_mapping_rviz_plugins_test src/test.cpp)




# Add wx widgets
#find_package(wxWidgets REQUIRED)
#include(${wxWidgets_USE_FILE})
#include_directories( ${wxWidgets_INCLUDE_DIRS} )

#target_link_libraries(${BUT_DISPLAY_PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES} )

# BUT data fusion
#rosbuild_add_executable(view src/but_data_fusion/view.cpp)
