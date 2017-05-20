cmake_minimum_required(VERSION 2.8.3)
project(liblemon)

#find_package(catkin REQUIRED)

#catkin_destinations() # set-up destination variables
#file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#include(ExternalProject)
#ExternalProject_Add(EP_${PROJECT_NAME}
#	URL ${PROJECT_SOURCE_DIR}/lemon
#
#	SOURCE_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src
#
#	CONFIGURE_COMMAND ""
#	BUILD_COMMAND ""
#		# copy headers to devel space (catkin does not like headers in source space)
#		COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/lemon ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#	INSTALL_COMMAND ""
#)

#catkin_package(
#	CFG_EXTRAS liblemon-extras.cmake # catkin_package does not support artifacts in devel space
#	EXPORTED_TARGETS EP_${PROJECT_NAME}
#)

#install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
#	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)



set(catkin_RUN_PACKAGES
	roscpp
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
#	LIBRARIES lemon
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
#	DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
#install(TARGETS 
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/lemon/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../lemon
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
	PATTERN ".svn" EXCLUDE
)