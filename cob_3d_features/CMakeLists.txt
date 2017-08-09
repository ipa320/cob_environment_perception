cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_features)

set(catkin_RUN_PACKAGES
	cob_3d_mapping_common
	cob_3d_mapping_msgs
#	cob_3d_mapping_tools
	cob_3d_visualization
	libpolypartition
	pcl_ros
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)
find_package(VTK REQUIRED COMPONENTS)# vtkCommon vtkFiltering vtkRendering)
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
		common/include
	LIBRARIES
		cob_3d_features
		cob_3d_features_omp
		s2kit
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		OpenCV
		VTK
		Boost
#		PCL
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
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
#	${PCL_INCLUDE_DIRS}
	${VTK_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_features
	common/src/fast_edge_estimation_3d.cpp
	common/src/edge_estimation_3d.cpp
	common/src/edge_estimation_2d.cpp
	common/src/edge_extraction.cpp
	common/src/knn_classifier.cpp
	common/src/most_discriminating_data_points.cpp
	common/src/organized_curvature_estimation.cpp
	common/src/organized_normal_estimation.cpp
	common/src/organized_normal_estimation_edge.cpp)
target_link_libraries(cob_3d_features
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
#	${PCL_LIBRARIES}
)
add_dependencies(cob_3d_features ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(cob_3d_features_omp
	common/src/organized_curvature_estimation_omp.cpp
	common/src/organized_normal_estimation_omp.cpp
	common/src/organized_normal_estimation_edge_omp.cpp
	common/src/fast_edge_estimation_3d_omp.cpp)
target_link_libraries(cob_3d_features_omp
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
#	${PCL_LIBRARIES}
)
add_dependencies(cob_3d_features_omp ${catkin_EXPORTED_TARGETS})
#set_target_properties(cob_3d_features_omp PROPERTIES COMPILE_FLAGS "-fopenmp"
#	LINK_FLAGS "-fopenmp"
#)


add_library(s2kit common/src/ShapeSPH/cospmls.cpp common/src/ShapeSPH/FST_semi_memo_fftw.cpp common/src/ShapeSPH/cospmls.cpp common/src/ShapeSPH/fft_grids.cpp common/src/ShapeSPH/FST_semi_memo.cpp common/src/ShapeSPH/FST_semi_memo_fftw.cpp common/src/ShapeSPH/naive_synthesis.cpp common/src/ShapeSPH/newFCT.cpp common/src/ShapeSPH/primitive.cpp common/src/ShapeSPH/primitive_FST.cpp common/src/ShapeSPH/seminaive.cpp common/src/ShapeSPH/seminaive_fftw.cpp common/src/ShapeSPH/FFTcode.cpp common/src/ShapeSPH/csecond.cpp common/src/ShapeSPH/weights.cpp common/src/ShapeSPH/OURperms.cpp common/src/ShapeSPH/oddweights.cpp common/src/ShapeSPH/OURmods.cpp common/src/ShapeSPH/indextables.cpp)
add_dependencies(s2kit ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(test_organized_curvature_estimation common/test/test_organized_curvature_estimation.cpp)
target_link_libraries(test_organized_curvature_estimation
	cob_3d_features
	cob_3d_features_omp
	${catkin_LIBRARIES}
	${VTK_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(test_organized_curvature_estimation ${catkin_EXPORTED_TARGETS})

# Test Surface Classification using organized principal curvatures
add_executable(test_surface_class_estimation common/test/test_surface_class_estimation.cpp)
target_link_libraries(test_surface_class_estimation
	cob_3d_features
	cob_3d_features_omp
	${catkin_LIBRARIES}
	${VTK_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(test_surface_class_estimation ${catkin_EXPORTED_TARGETS})

# Test program for 2D (on RGB or depth) and 3D EdgeEstimation + visualization
add_executable(test_edge_sharpening common/test/test_edge_sharpening.cpp)
target_link_libraries(test_edge_sharpening
	cob_3d_features
	cob_3d_features_omp
	${catkin_LIBRARIES}
	${VTK_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(test_edge_sharpening ${catkin_EXPORTED_TARGETS})

add_executable(profile_oce common/test/profile_oce.cpp)
target_link_libraries(profile_oce
	cob_3d_features
	cob_3d_features_omp
	${catkin_LIBRARIES}
	${VTK_LIBRARIES}
	${OpenCV_LIBRARIES}
)
add_dependencies(profile_oce ${catkin_EXPORTED_TARGETS})

add_executable(profile_fee_omp common/test/profile_fee_omp.cpp)
target_link_libraries(profile_fee_omp
	cob_3d_features
	cob_3d_features_omp
	${catkin_LIBRARIES}
	${VTK_LIBRARIES}
	${OpenCV_LIBRARIES}
)
add_dependencies(profile_fee_omp ${catkin_EXPORTED_TARGETS})

add_executable(profile_fee common/test/profile_fee.cpp)
target_link_libraries(profile_fee
	cob_3d_features
	cob_3d_features_omp
	${catkin_LIBRARIES}
	${VTK_LIBRARIES}
	${OpenCV_LIBRARIES}
)
add_dependencies(profile_fee ${catkin_EXPORTED_TARGETS})

add_executable(profile_ne common/test/profile_ne.cpp)
target_link_libraries(profile_ne
	cob_3d_features
	cob_3d_features_omp
	${catkin_LIBRARIES}
)
add_dependencies(profile_ne ${catkin_EXPORTED_TARGETS})

add_executable(eval_invariant_feature common/test/evaluate_invariant_feature.cpp)
target_link_libraries(eval_invariant_feature
	s2kit
	fftw3
	fftw3f
	${catkin_LIBRARIES}
#	${PCL_LIBRARIES}
)
add_dependencies(eval_invariant_feature ${catkin_EXPORTED_TARGETS})

add_executable(invariant_feature_node ros/src/invariant_feature_node.cpp)
target_link_libraries(invariant_feature_node
	s2kit
	fftw3
	fftw3f
	${catkin_LIBRARIES}
#	${PCL_LIBRARIES}
)
add_dependencies(invariant_feature_node ${catkin_EXPORTED_TARGETS})

# extracts pc from organized cloud and saves as ppm:
add_executable(extract_organized_curvatures common/test/extract_organized_curvatures.cpp)
target_link_libraries(extract_organized_curvatures
	cob_3d_features
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(extract_organized_curvatures ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_3d_features_omp cob_3d_features s2kit profile_fee eval_invariant_feature extract_organized_curvatures profile_fee_omp invariant_feature_node profile_oce profile_ne
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY common/include/ShapeSPH
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)
