cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_evaluation_features)

set(catkin_RUN_PACKAGES
	cob_3d_features
	cob_3d_mapping_common
	cob_3d_mapping_tools
	sensor_msgs
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

find_package(Boost REQUIRED COMPONENTS filesystem program_options signals system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED) # COMPONENTS vtkCommon vtkFiltering vtkRendering)

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
	include
LIBRARIES
	${PROJECT_NAME}
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${VTK_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# extracts fpfh, pc and rsd feature values from a scene into separated pcd files:
add_executable(extract_feature_values src/extract_feature_values.cpp)
add_dependencies(extract_feature_values ${catkin_EXPORTED_TARGETS})
target_link_libraries(extract_feature_values
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

# some tools for fpfh training:
add_executable(merge_knn_training_data src/merge_knn_training_data.cpp)
add_dependencies(merge_knn_training_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge_knn_training_data
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(merge_fpfh_training_data src/merge_fpfh_training_data.cpp)
add_dependencies(merge_fpfh_training_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge_fpfh_training_data
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(reduce_fpfh_training_data src/reduce_fpfh_training_data.cpp)
add_dependencies(reduce_fpfh_training_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(reduce_fpfh_training_data
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(fpfh_primitives src/fpfh_primitives.cpp)
add_dependencies(fpfh_primitives ${catkin_EXPORTED_TARGETS})
target_link_libraries(fpfh_primitives
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${VTK_LIBRARIES}
)

# some tools to generate fpfh, pc and rsd test data and evaluate various parameters
add_executable(evaluate_test_data src/evaluate_test_data.cpp)
add_dependencies(evaluate_test_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(evaluate_test_data
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
set_target_properties(evaluate_test_data PROPERTIES COMPILE_FLAGS -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

add_executable(generate_test_data src/generate_test_data.cpp)
add_dependencies(generate_test_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(generate_test_data
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
set_target_properties(generate_test_data PROPERTIES COMPILE_FLAGS -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

#add_executable(evaluation_features src/evaluation_features.cpp) #deactivated because rsd was removed from PCL
#add_dependencies(evaluation_features ${catkin_EXPORTED_TARGETS})
#target_link_libraries(evaluation_features
#	${catkin_LIBRARIES}
#	${Boost_LIBRARIES}
#	${OpenCV_LIBRARIES}
#	${VTK_LIBRARIES}
#)
#set_target_properties(evaluation_features PROPERTIES COMPILE_FLAGS -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

add_executable(evaluation_features_fpfh src/evaluation_features_fpfh.cpp)
add_dependencies(evaluation_features_fpfh ${catkin_EXPORTED_TARGETS})
target_link_libraries(evaluation_features_fpfh
	${catkin_LIBRARIES}
)

add_executable(evaluation_fpfh_svm_trainer src/evaluation_fpfh_svm_trainer.cpp)
add_dependencies(evaluation_fpfh_svm_trainer ${catkin_EXPORTED_TARGETS})
target_link_libraries(evaluation_fpfh_svm_trainer
	${catkin_LIBRARIES}
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS extract_feature_values merge_knn_training_data merge_fpfh_training_data reduce_fpfh_training_data fpfh_primitives evaluate_test_data generate_test_data evaluation_features_fpfh evaluation_fpfh_svm_trainer      # evaluation_features
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)
