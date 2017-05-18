cmake_minimum_required(VERSION 2.8.3)
project(libnuklei)

find_package(catkin REQUIRED)
find_package(LAPACK REQUIRED)
#find_package(GSL REQUIRED)

catkin_destinations() # set-up destination variables
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include(ExternalProject)
ExternalProject_Add(EP_${PROJECT_NAME}
    URL ${PROJECT_SOURCE_DIR}/nuklei-code

    SOURCE_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src
    BINARY_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build

    CONFIGURE_COMMAND ""
    BUILD_COMMAND COMMAND ${CMAKE_COMMAND} -E chdir ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src ./scons.py use_cgal=yes prefix=${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build install
    
        # copy headers to devel space (catkin does not like headers in source space)
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build/include ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        # copy libs, set-up soname chain
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build/lib/libnuklei.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build/bin/nuklei ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
    INSTALL_COMMAND ""
)

catkin_package(
    CFG_EXTRAS libnuklei-extras.cmake # catkin_package does not support artifacts in devel space
    EXPORTED_TARGETS EP_${PROJECT_NAME}
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libnuklei.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/nuklei
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)






#message(STATUS "Project source dir: ${PROJECT_SOURCE_DIR}")
#add_custom_target(
#    build_nuklei ALL
#    COMMAND test -e ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED || (cmake -E chdir ${PROJECT_SOURCE_DIR}/nuklei-code ./scons.py use_cgal=yes prefix=${PROJECT_SOURCE_DIR}/nuklei install && touch ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED)
#)

#set(libraries "nuklei")
#foreach(library ${libraries})
#	add_custom_target(
#		copy_${library} ALL
#		COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/nuklei/lib/lib${library}.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
#		COMMENT "Copying ${PROJECT_SOURCE_DIR}/nuklei/lib/lib${library}.so to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}"
#	)
#endforeach()

#catkin_package(
#	INCLUDE_DIRS
#		nuklei/include
#	LIBRARIES
#		nuklei
#	CATKIN_DEPENDS
#	DEPENDS
#)

#include_directories(
#	nuklei/include
#)

#foreach(library ${libraries})
#	add_library(${library} SHARED)
#	add_dependencies(${library} copy_${library})
#	add_dependencies(copy_${library} build_nuklei)
#	set_target_properties(${library} PROPERTIES LINKER_LANGUAGE CXX)
#endforeach()
#set_target_properties(build_nuklei PROPERTIES LINKER_LANGUAGE CXX)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
#install(TARGETS nuklei
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY common/include
#	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#)
