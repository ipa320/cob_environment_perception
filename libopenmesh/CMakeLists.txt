cmake_minimum_required(VERSION 2.8.3)
project(libopenmesh)

find_package(catkin REQUIRED)

catkin_destinations() # set-up destination variables
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include(ExternalProject)
ExternalProject_Add(EP_${PROJECT_NAME}
	URL https://github.com/ipa320/thirdparty/raw/master/OpenMesh-2.4.tar.gz
	URL_MD5 7449b5437527113befad8e02cf52ecd1

	SOURCE_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src
	BINARY_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build

	CONFIGURE_COMMAND ${CMAKE_COMMAND} ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src -DCMAKE_BUILD_TYPE=Release -DBUILD_APPS=OFF
	BUILD_COMMAND ${CMAKE_COMMAND} --build . --config Release
		# copy headers to devel space (catkin does not like headers in source space)
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/src ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		# copy libs, set-up soname chain
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build/Build/lib/OpenMesh/libOpenMeshCore.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build/Build/lib/OpenMesh/libOpenMeshCore.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
		COMMAND ${CMAKE_COMMAND} -E create_symlink libOpenMeshCore.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libOpenMeshCore.so
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build/Build/lib/OpenMesh/libOpenMeshTools.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-build/Build/lib/OpenMesh/libOpenMeshTools.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
		COMMAND ${CMAKE_COMMAND} -E create_symlink libOpenMeshTools.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libOpenMeshTools.so
	INSTALL_COMMAND ""
)

catkin_package(
	CFG_EXTRAS libopenmesh-extras.cmake # catkin_package does not support artifacts in devel space
	EXPORTED_TARGETS EP_${PROJECT_NAME}
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libOpenMeshCore.so
				 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libOpenMeshCore.so.2.4
				 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libOpenMeshTools.so
				 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libOpenMeshTools.so.2.4
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)