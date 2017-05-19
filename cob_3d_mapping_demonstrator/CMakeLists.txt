cmake_minimum_required(VERSION 2.8.3)
project(cob_3d_mapping_demonstrator)

set(catkin_RUN_PACKAGES
	actionlib
	brics_actuator
	cob_3d_mapping_msgs
	cob_script_server
	cob_srvs
	diagnostic_msgs
	roscpp
	rviz
	std_srvs
	urdf
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

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
		common/include
		ros/include
	LIBRARIES
		cob_3d_mapping_demonstrator_rviz
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		Qt4
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	common/include
	ros/include
	${catkin_INCLUDE_DIRS}
	${QT_INCLUDE_DIRS}
)

## Declare a cpp library
## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
	ros/include/cob_3d_mapping_demonstrator/rviz_title.h
	ros/include/cob_3d_mapping_demonstrator/rviz_buttons.h
)

add_library(cob_3d_mapping_demonstrator_rviz 
	ros/src/rviz_buttons.cpp
	ros/src/rviz_title.cpp
	${MOC_FILES}
)
target_link_libraries(cob_3d_mapping_demonstrator_rviz	
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)
add_dependencies(cob_3d_mapping_demonstrator_rviz ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(cob_3d_mapping_demonstrator 
	ros/src/demonstrator_node.cpp 
	common/src/demonstrator_control.cpp 
	common/src/demonstrator_control_maestro.cpp 
	common/src/serial_device.cpp
)
target_link_libraries(cob_3d_mapping_demonstrator 
	${catkin_LIBRARIES}
)
add_dependencies(cob_3d_mapping_demonstrator ${catkin_EXPORTED_TARGETS})

add_executable(testClient ros/test/test_client.cpp)
target_link_libraries(testClient 
	${catkin_LIBRARIES}
)
add_dependencies(testClient ${catkin_EXPORTED_TARGETS})

add_executable(testClientPOS ros/test/test_clientPOS.cpp)
target_link_libraries(testClientPOS 
	${catkin_LIBRARIES}
)
add_dependencies(testClientPOS ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_3d_mapping_demonstrator_rviz cob_3d_mapping_demonstrator
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/ ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)