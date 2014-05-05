# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "action_server: 28 messages, 0 services")

set(MSG_I_FLAGS "-Iaction_server:/home/ncos/mipt-airdrone/devel/share/action_server/msg;-Iactionlib_msgs:/opt/ros/hydro/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(action_server_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongResult.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)
_generate_msg_cpp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
)

### Generating Services

### Generating Module File
_generate_module_cpp(action_server
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(action_server_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(action_server_generate_messages action_server_generate_messages_cpp)

# target for backward compatibility
add_custom_target(action_server_gencpp)
add_dependencies(action_server_gencpp action_server_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_server_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongResult.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)
_generate_msg_lisp(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
)

### Generating Services

### Generating Module File
_generate_module_lisp(action_server
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(action_server_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(action_server_generate_messages action_server_generate_messages_lisp)

# target for backward compatibility
add_custom_target(action_server_genlisp)
add_dependencies(action_server_genlisp action_server_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_server_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongResult.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/MoveAlongGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/SwitchWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/ApproachWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)
_generate_msg_py(action_server
  "/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_server/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
)

### Generating Services

### Generating Module File
_generate_module_py(action_server
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(action_server_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(action_server_generate_messages action_server_generate_messages_py)

# target for backward compatibility
add_custom_target(action_server_genpy)
add_dependencies(action_server_genpy action_server_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_server_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_server
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(action_server_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
add_dependencies(action_server_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_server
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(action_server_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
add_dependencies(action_server_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_server
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(action_server_generate_messages_py actionlib_msgs_generate_messages_py)
add_dependencies(action_server_generate_messages_py std_msgs_generate_messages_py)
