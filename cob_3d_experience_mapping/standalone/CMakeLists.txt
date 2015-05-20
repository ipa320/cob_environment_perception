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
add_executable(standalone_experience_mapping_lemon_node ../src/exp_mapping_lemon_standalone.cpp)

# make sure configure headers are built before any node using them
add_dependencies(standalone_experience_mapping_lemon_node ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(standalone_experience_mapping_lemon_node
  ${catkin_LIBRARIES}
)

