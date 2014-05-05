# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "action_client: 28 messages, 0 services")

set(MSG_I_FLAGS "-Iaction_client:/home/ncos/mipt-airdrone/devel/share/action_client/msg;-Iactionlib_msgs:/opt/ros/hydro/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(action_client_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallResult.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)
_generate_msg_cpp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
)

### Generating Services

### Generating Module File
_generate_module_cpp(action_client
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(action_client_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(action_client_generate_messages action_client_generate_messages_cpp)

# target for backward compatibility
add_custom_target(action_client_gencpp)
add_dependencies(action_client_gencpp action_client_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_client_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallResult.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)
_generate_msg_lisp(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
)

### Generating Services

### Generating Module File
_generate_module_lisp(action_client
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(action_client_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(action_client_generate_messages action_client_generate_messages_lisp)

# target for backward compatibility
add_custom_target(action_client_genlisp)
add_dependencies(action_client_genlisp action_client_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_client_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallResult.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallGoal.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationAction.msg"
  "${MSG_I_FLAGS}"
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionFeedback.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionResult.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachWallFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/hydro/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)
_generate_msg_py(action_client
  "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
)

### Generating Services

### Generating Module File
_generate_module_py(action_client
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(action_client_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(action_client_generate_messages action_client_generate_messages_py)

# target for backward compatibility
add_custom_target(action_client_genpy)
add_dependencies(action_client_genpy action_client_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_client_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_client
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(action_client_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
add_dependencies(action_client_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_client
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(action_client_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
add_dependencies(action_client_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_client
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(action_client_generate_messages_py actionlib_msgs_generate_messages_py)
add_dependencies(action_client_generate_messages_py std_msgs_generate_messages_py)
