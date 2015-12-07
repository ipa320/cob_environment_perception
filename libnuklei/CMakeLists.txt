cmake_minimum_required(VERSION 2.8.3)
project(libnuklei)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)

message(STATUS "Project source dir: ${PROJECT_SOURCE_DIR}")
add_custom_target(
    build_nuklei ALL
    COMMAND test -e ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED || (cmake -E chdir ${PROJECT_SOURCE_DIR}/nuklei-code ./scons.py use_cgal=yes prefix=${PROJECT_SOURCE_DIR}/nuklei install && touch ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED)
)

set(libraries "nuklei")
foreach(library ${libraries})
	add_custom_target(
		copy_${library} ALL
		COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/nuklei/lib/lib${library}.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
    	COMMENT "Copying ${PROJECT_SOURCE_DIR}/nuklei/lib/lib${library}.so to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}"
	)
endforeach()

catkin_package(
	INCLUDE_DIRS
		nuklei/include
	LIBRARIES
		nuklei
	CATKIN_DEPENDS
	DEPENDS
)

include_directories(
	nuklei/include
)

foreach(library ${libraries})
	add_library(${library} SHARED)
	add_dependencies(${library} copy_${library})
	add_dependencies(copy_${library} build_nuklei)
	set_target_properties(${library} PROPERTIES LINKER_LANGUAGE CXX)
endforeach()
set_target_properties(build_nuklei PROPERTIES LINKER_LANGUAGE CXX)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS nuklei
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
