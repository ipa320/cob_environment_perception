cmake_minimum_required(VERSION 2.8.3)
project(openmesh)

message(STATUS "Project source dir: ${PROJECT_SOURCE_DIR}")
add_custom_target(
    build_openmesh24 ALL
    COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball)

#add_custom_target(
#    copy_openmesh24 ALL
#    COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/OpenMesh/libOpenMeshCore.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshCore.so
#    COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/OpenMesh/libOpenMeshTools.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshTools.so
#)
add_custom_target(
    copy_openmesh24 ALL
    COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/OpenMesh/libOpenMeshCore.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshCore.so.2.4
    COMMAND ln -sf ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshCore.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshCore.so
    COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/OpenMesh/libOpenMeshTools.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshTools.so.2.4
    COMMAND ln -sf ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshTools.so.2.4 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshTools.so
)
message(STATUS "Copying libs to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}")

add_library(openmesh24 SHARED)
set_target_properties(openmesh24 PROPERTIES LINKER_LANGUAGE CXX)

#add_dependencies(copy_openmesh24 build_openmesh24)
add_dependencies(copy_openmesh24 build_openmesh24)
#add_dependencies(openmesh24 copy_openmesh24)
add_dependencies(openmesh24 copy_openmesh24)




