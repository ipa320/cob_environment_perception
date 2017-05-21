cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_features_evaluation)

set(catkin_RUN_PACKAGES
	cob_3d_features
	cob_3d_mapping_common
	cob_3d_segmentation
	cob_3d_visualization
	roscpp
	pcl_ros
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

## System dependencies are found with CMake's conventions
#find_package(OpenCV REQUIRED)
#find_package(VTK REQUIRED COMPONENTS vtkCommon vtkFiltering vtkRendering)
find_package(Boost REQUIRED COMPONENTS program_options)
#find_package(PCL REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


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
	LIBRARIES
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		#OpenCV
		#VTK
		Boost
		#PCL
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	#common/include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	#${OpenCV_INCLUDE_DIRS}
	#${PCL_INCLUDE_DIRS}
	#${VTK_INCLUDE_DIRS}
)


add_executable(test_invariant_feature common/test/test_invariant_feature.cpp)
add_dependencies(test_invariant_feature ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_invariant_feature
	fftw3
	fftw3f
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(tf_from_bag common/test/tf_from_bag.cpp)
add_dependencies(tf_from_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_from_bag
	${catkin_LIBRARIES}
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS test_invariant_feature tf_from_bag
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)