# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cob_3d_segmentation: 7 messages, 0 services")

set(MSG_I_FLAGS "-Icob_3d_segmentation:/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg;-Iactionlib_msgs:/opt/ros/groovy/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/groovy/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cob_3d_segmentation_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_cpp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchGoal.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_cpp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_cpp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchGoal.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionGoal.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionFeedback.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionResult.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_cpp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_cpp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_cpp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation
)

### Generating Services

### Generating Module File
_generate_module_cpp(cob_3d_segmentation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cob_3d_segmentation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cob_3d_segmentation_generate_messages cob_3d_segmentation_generate_messages_cpp)

# target for backward compatibility
add_custom_target(cob_3d_segmentation_gencpp)
add_dependencies(cob_3d_segmentation_gencpp cob_3d_segmentation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cob_3d_segmentation_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_lisp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchGoal.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_lisp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_lisp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchGoal.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionGoal.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionFeedback.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionResult.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_lisp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_lisp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_lisp(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation
)

### Generating Services

### Generating Module File
_generate_module_lisp(cob_3d_segmentation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cob_3d_segmentation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cob_3d_segmentation_generate_messages cob_3d_segmentation_generate_messages_lisp)

# target for backward compatibility
add_custom_target(cob_3d_segmentation_genlisp)
add_dependencies(cob_3d_segmentation_genlisp cob_3d_segmentation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cob_3d_segmentation_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_py(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchGoal.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_py(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_py(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchGoal.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionGoal.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionFeedback.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionResult.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_py(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_py(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation
)
_generate_msg_py(cob_3d_segmentation
  "/home/josh/workspace/cob_environment_perception/cob_3d_segmentation/devel/share/cob_3d_segmentation/msg/ObjectWatchFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation
)

### Generating Services

### Generating Module File
_generate_module_py(cob_3d_segmentation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cob_3d_segmentation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cob_3d_segmentation_generate_messages cob_3d_segmentation_generate_messages_py)

# target for backward compatibility
add_custom_target(cob_3d_segmentation_genpy)
add_dependencies(cob_3d_segmentation_genpy cob_3d_segmentation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cob_3d_segmentation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_3d_segmentation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(cob_3d_segmentation_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
add_dependencies(cob_3d_segmentation_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_3d_segmentation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(cob_3d_segmentation_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
add_dependencies(cob_3d_segmentation_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_3d_segmentation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(cob_3d_segmentation_generate_messages_py actionlib_msgs_generate_messages_py)
add_dependencies(cob_3d_segmentation_generate_messages_py geometry_msgs_generate_messages_py)
