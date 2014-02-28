cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cob_3d_mapping_msgs
  eigen_conversions
  libgpc
  pcl_conversions
)

find_package(PCL REQUIRED)


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
    ros/include
    common/include
  LIBRARIES
    cob_3d_mapping_common
  CATKIN_DEPENDS
    cob_3d_mapping_msgs
    eigen_conversions
    libgpc
    pcl_conversions
  DEPENDS
    PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(cob_3d_mapping_common
	common/src/shape.cpp
	common/src/polygon.cpp
	common/src/cylinder.cpp
	common/src/polypartition.cpp
	common/src/dominant_color.cpp
        common/src/io.cpp
)

## Declare a cpp executable
add_executable(test_minimal_rectangle common/test/test_minimal_rectangle.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(test_minimal_rectangle 
  ${catkin_LIBRARIES}
)

#Unit tests
add_executable(test_polygon common/test/test_polygon.cpp)
add_executable(test_cylinder common/test/test_cylinder.cpp)

target_link_libraries(test_polygon 
  cob_3d_mapping_common 
  gtest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(test_cylinder 
  cob_3d_mapping_common 
  gtest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(test_polygon ${catkin_EXPORTED_TARGETS})
add_dependencies(test_cylinder ${catkin_EXPORTED_TARGETS})

