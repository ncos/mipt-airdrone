# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${roscopter_DIR}/.." "msg" roscopter_MSG_INCLUDE_DIRS UNIQUE)
set(roscopter_MSG_DEPENDENCIES std_msgs;sensor_msgs;geometry_msgs)
