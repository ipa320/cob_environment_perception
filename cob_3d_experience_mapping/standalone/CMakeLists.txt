cmake_minimum_required(VERSION 2.8.3)
project(experience_mapping)

message("building for system: ${CMAKE_SYSTEM_NAME2}")

if(${CMAKE_SYSTEM_NAME2} MATCHES "Android")
	# Target system
	set(CMAKE_SYSTEM_NAME  Android)

	set(CMAKE_SYSTEM_VERSION 1)

	# Compiler to build for the target
	set(CMAKE_C_COMPILER /opt/arm-linux-androideabi-4.8/bin/arm-linux-androideabi-gcc)
	set(CMAKE_CXX_COMPILER /opt/arm-linux-androideabi-4.8/bin/arm-linux-androideabi-g++)
endif()

find_package(Eigen REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ../common/include
  ../../lemon
)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

## Declare a cpp executable
add_library(standalone_experience_mapping_${CMAKE_SYSTEM_NAME} SHARED ../src/exp_mapping_lemon_standalone.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(standalone_experience_mapping_${CMAKE_SYSTEM_NAME}
  ${catkin_LIBRARIES}
)

