# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "roscopter: 6 messages, 0 services")

set(MSG_I_FLAGS "-Iroscopter:/home/ncos/mipt-airdrone/src/roscopter/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(roscopter_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roscopter
)
_generate_msg_cpp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roscopter
)
_generate_msg_cpp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/VFR_HUD.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roscopter
)
_generate_msg_cpp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/Mavlink_RAW_IMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roscopter
)
_generate_msg_cpp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/Attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roscopter
)
_generate_msg_cpp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/RC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roscopter
)

### Generating Services

### Generating Module File
_generate_module_cpp(roscopter
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roscopter
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(roscopter_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(roscopter_generate_messages roscopter_generate_messages_cpp)

# target for backward compatibility
add_custom_target(roscopter_gencpp)
add_dependencies(roscopter_gencpp roscopter_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roscopter_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roscopter
)
_generate_msg_lisp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roscopter
)
_generate_msg_lisp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/VFR_HUD.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roscopter
)
_generate_msg_lisp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/Mavlink_RAW_IMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roscopter
)
_generate_msg_lisp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/Attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roscopter
)
_generate_msg_lisp(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/RC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roscopter
)

### Generating Services

### Generating Module File
_generate_module_lisp(roscopter
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roscopter
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(roscopter_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(roscopter_generate_messages roscopter_generate_messages_lisp)

# target for backward compatibility
add_custom_target(roscopter_genlisp)
add_dependencies(roscopter_genlisp roscopter_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roscopter_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter
)
_generate_msg_py(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter
)
_generate_msg_py(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/VFR_HUD.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter
)
_generate_msg_py(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/Mavlink_RAW_IMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter
)
_generate_msg_py(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/Attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter
)
_generate_msg_py(roscopter
  "/home/ncos/mipt-airdrone/src/roscopter/msg/RC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter
)

### Generating Services

### Generating Module File
_generate_module_py(roscopter
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(roscopter_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(roscopter_generate_messages roscopter_generate_messages_py)

# target for backward compatibility
add_custom_target(roscopter_genpy)
add_dependencies(roscopter_genpy roscopter_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roscopter_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roscopter)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roscopter
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(roscopter_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(roscopter_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(roscopter_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roscopter)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roscopter
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(roscopter_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(roscopter_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(roscopter_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roscopter
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(roscopter_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(roscopter_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(roscopter_generate_messages_py geometry_msgs_generate_messages_py)
