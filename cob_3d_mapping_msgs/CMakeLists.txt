cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  actionlib_msgs
  cob_object_detection_msgs
  message_generation
)

#######################################
## Declare ROS messages and services ##
#######################################


## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
    action
  FILES
    PlaneExtraction.action
    TableObjectCluster.action
    Trigger.action
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
    msg
  FILES
    CurvedPolygonArray.msg
    CurvedPolygon.msg
    Feature.msg
    FilterObject.msg
    PolylinePoint.msg
    ShapeArray.msg
    Shape.msg
    SimilarityScore.msg
    
    Point2D.msg
    Polygon.msg
    Plane.msg
    PlaneScene.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    GetApproachPoseForPolygon.srv
    GetBoundingBoxes.srv
    GetGeometryMap.srv
    GetObjectsOfClass.srv
    GetPlane.srv
    GetPointMap.srv
    GetTables.srv
    ModifyMap.srv
    MoveToTable.srv
    SetBoundingBoxes.srv
    SetGeometryMap.srv
    SetPointMap.srv
    UpdateMap.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    actionlib_msgs
    cob_object_detection_msgs
)

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
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    std_msgs
    actionlib_msgs
    cob_object_detection_msgs
    message_runtime
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#  test/detection_component_testing.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


