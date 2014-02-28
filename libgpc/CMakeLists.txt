cmake_minimum_required(VERSION 2.8.3)
project(libgpc)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    gpc
)

include_directories(
  include
)

add_library(gpc src/gpc.c)





