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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include ${PROJECT_SOURCE_DIR}/ros/include /usr/local/include/boost_1_49 /usr/local/include/nurbs++)
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include ${PROJECT_SOURCE_DIR}/ros/include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


#	EXECUTABLES
rosbuild_add_executable(slam_node
	ros/src/slam_node.cpp
	common/src/curved_polygons/polygon_merger.cpp
	common/src/lib/polypartition.cpp
)
target_link_libraries(slam_node nurbsf matrix matrixN matrixI CGAL)
rosbuild_add_compile_flags(slam_node ${SSE_FLAGS} -frounding-math)


#	TESTING
#rosbuild_add_gtest(Tatoms ros/test/atoms.cpp)
rosbuild_add_executable(test_atoms ros/test/atoms.cpp)
target_link_libraries(test_atoms gtest CGAL)
rosbuild_add_compile_flags(test_atoms ${SSE_FLAGS} -frounding-math)

rosbuild_add_executable(test_merging ros/test/merging.cpp)
target_link_libraries(test_merging gtest nurbsf matrix matrixN matrixI CGAL CGAL_PDB CGAL_Core)
rosbuild_add_compile_flags(test_merging ${SSE_FLAGS} -frounding-math)

rosbuild_add_executable(test_slam ros/test/slam.cpp common/src/curved_polygons/polygon_merger.cpp common/src/lib/polypartition.cpp)
target_link_libraries(test_slam gtest nurbsf matrix matrixN matrixI CGAL ${SSE_FLAGS})
rosbuild_add_compile_flags(test_slam ${SSE_FLAGS} -frounding-math)
