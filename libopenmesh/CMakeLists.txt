cmake_minimum_required(VERSION 2.8.3)
project(libopenmesh)

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

message(STATUS "Project source dir: ${PROJECT_SOURCE_DIR}")
add_custom_target(
    build_openmesh24 ALL
    COMMAND test -e ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED || (cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball && touch ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED)
)


set(libraries "OpenMeshCore;OpenMeshTools")
foreach(library ${libraries})
	add_custom_target(
		copy_${library} ALL
		COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/common/lib/OpenMesh/lib${library}.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
		COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/common/lib/OpenMesh/lib${library}.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
		COMMAND ln -sf ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/lib${library}.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/lib${library}.so
    	COMMENT "Copying ${PROJECT_SOURCE_DIR}/common/lib/OpenMesh/lib${library}.so to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}"
	)
endforeach()

catkin_package(
	INCLUDE_DIRS
		common/include
	LIBRARIES
		OpenMeshCore
		OpenMeshTools
	CATKIN_DEPENDS
	DEPENDS
)

include_directories(
	common/include
)

foreach(library ${libraries})
	add_library(${library} SHARED)
	add_dependencies(${library} copy_${library})
	add_dependencies(copy_${library} build_openmesh24)
	set_target_properties(${library} PROPERTIES LINKER_LANGUAGE CXX)
endforeach()
set_target_properties(build_openmesh24 PROPERTIES LINKER_LANGUAGE CXX)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS OpenMeshCore OpenMeshTools
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)