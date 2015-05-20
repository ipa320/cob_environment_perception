cmake_minimum_required(VERSION 2.8.3)
project(experience_mapping)

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
add_library(standalone_experience_mapping SHARED ../src/exp_mapping_lemon_standalone.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(standalone_experience_mapping
  ${catkin_LIBRARIES}
)

