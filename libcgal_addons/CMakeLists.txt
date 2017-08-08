cmake_minimum_required(VERSION 2.8.3)
project(libcgal_addons)

find_package(catkin REQUIRED roscpp)

# check for ROS kinetic, i.e. ${roscpp_VERSION_MINOR} is >= 12
message("roscpp version: ${roscpp_VERSION_MAJOR}.${roscpp_VERSION_MINOR}.${roscpp_VERSION_PATCH}")
if ("${roscpp_VERSION_MAJOR}.${roscpp_VERSION_MINOR}.${roscpp_VERSION_PATCH}" VERSION_LESS "1.12.0")
	message("Using ROS indigo, jade or older. No cgal addons necessary.")
else ("${roscpp_VERSION_MAJOR}.${roscpp_VERSION_MINOR}.${roscpp_VERSION_PATCH}" VERSION_LESS "1.12.0")
	message("Using ROS kinetic or newer.")

	# install cgal-qt5-dev
	execute_process(COMMAND dpkg -l libcgal-qt5-dev RESULT_VARIABLE RESULT OUTPUT_VARIABLE MARKER_A_OUTPUT ERROR_VARIABLE MARKER_A_OUTPUT)
	if(RESULT EQUAL 0)
		message("libcgal-qt5-dev = FOUND")
	else(RESULT EQUAL 0)
		message("libcgal-qt5-dev = NOT FOUND")
	endif(RESULT EQUAL 0)
	
	if(NOT (RESULT EQUAL 0))
		message("Installing libcgal-qt5-dev:")
	
		# update apt-get
		message("sudo -E apt-get update -q:")
		execute_process(COMMAND sudo -E apt-get update -q OUTPUT_VARIABLE COMMAND_1_OUTPUT ERROR_VARIABLE COMMAND_1_OUTPUT)
		message("${COMMAND_1_OUTPUT}")
		
		# install opencv-nonfree
		message("sudo -E apt-get install --yes libcgal-qt5-dev:")
		execute_process(COMMAND sudo -E apt-get install --yes libcgal-qt5-dev OUTPUT_VARIABLE COMMAND_2_OUTPUT ERROR_VARIABLE COMMAND_2_OUTPUT)
		message("${COMMAND_2_OUTPUT}")
		
	endif(NOT (RESULT EQUAL 0))

endif ("${roscpp_VERSION_MAJOR}.${roscpp_VERSION_MINOR}.${roscpp_VERSION_PATCH}" VERSION_LESS "1.12.0")

catkin_package()
