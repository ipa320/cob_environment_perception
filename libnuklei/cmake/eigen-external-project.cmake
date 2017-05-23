find_package(Eigen3)
if(NOT EIGEN3_FOUND)
	# Fallback to cmake_modules
	find_package(cmake_modules REQUIRED)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
	set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
	set(EIGEN3_DEFINITIONS ${EIGEN_DEFINITIONS})
	# Possibly map additional variables to the EIGEN3_ prefix.
else()
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(
	${EIGEN3_INCLUDE_DIRS}
)

message("EIGEN3_INCLUDE_DIRS ##########################################################################################################################")
message(${EIGEN3_INCLUDE_DIRS})
message("##############################################################################################################################################")