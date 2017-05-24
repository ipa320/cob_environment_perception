cmake_minimum_required(VERSION 2.8.3)
project(libnuklei)

find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(LAPACK REQUIRED)
#find_package(GSL REQUIRED)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
	# Fallback to cmake_modules
	find_package(cmake_modules REQUIRED)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
	set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
	set(EIGEN3_DEFINITIONS ${EIGEN_DEFINITIONS})
	# Possibly map additional variables to the EIGEN3_ prefix.
else()
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(
	${EIGEN3_INCLUDE_DIRS}
)

catkin_destinations() # set-up destination variables
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include(ExternalProject)
ExternalProject_Add(EP_${PROJECT_NAME}
	URL ${PROJECT_SOURCE_DIR}/nuklei-code

	SOURCE_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src
	BINARY_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build

	CONFIGURE_COMMAND ""
	#CMAKE_ARGS -DCMAKE_PROJECT_${PROJECT_NAME}_INCLUDE=${PROJECT_SOURCE_DIR}/cmake/eigen-external-project.cmake
	
	BUILD_COMMAND COMMAND echo "################################################################################ EIGEN3_INCLUDE_DIRS: " ${EIGEN3_INCLUDE_DIRS}
		COMMAND export CPPFLAGS="-I${EIGEN3_INCLUDE_DIRS}/ $CPPFLAGS"
		COMMAND ${CMAKE_COMMAND} -E chdir ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src ./scons.py use_cgal=yes prefix=${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build install
		# copy headers to devel space (catkin does not like headers in source space)
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build/include ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		# copy libs, set-up soname chain
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build/lib/libnuklei.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
	INSTALL_COMMAND ""
)

catkin_package(
	CFG_EXTRAS libnuklei-extras.cmake # catkin_package does not support artifacts in devel space
	EXPORTED_TARGETS EP_${PROJECT_NAME}
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libnuklei.so
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)