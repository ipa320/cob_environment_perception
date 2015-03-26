cmake_minimum_required(VERSION 2.8.3)
project(libpolypartition)

set(catkin_RUN_PACKAGES
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
	#cmake_modules
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Creates a bunch of environment variables that may be used later in the script
## e.g. catkin_INCLUDE_DIRS
find_package(catkin REQUIRED  # COMPONENTS
	#${catkin_BUILD_PACKAGES}
)

#add_custom_target(
#    get_polypartition ALL
#    COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.svn)

#add_custom_target(
#    copy_phidget21 ALL
#    COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/libphidget21.so.0  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libphidget21.so
#)
#add_custom_target(
#    copy_phidget210 ALL
#    COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/libphidget21.so.0  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libphidget21.so.0
#)
#message(STATUS "Copying libs to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}")


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS:
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
    	common/include
    LIBRARIES
    	polypartition
)

include_directories(
  common/include
)

add_library(polypartition
  common/src/polypartition.cpp
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS polypartition
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)






#set_target_properties(phidget21 PROPERTIES LINKER_LANGUAGE CXX)

#add_dependencies(copy_phidget21 build_phidget21)
#add_dependencies(copy_phidget210 build_phidget21)
#add_dependencies(phidget21 copy_phidget21)
#add_dependencies(polypartition get_polypartition)#copy_phidget210)

## Mark executables and/or libraries for installation
#nstall(TARGETS
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#
