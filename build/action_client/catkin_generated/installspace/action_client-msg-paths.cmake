# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${action_client_DIR}/.." "msg;msg;msg;msg" action_client_MSG_INCLUDE_DIRS UNIQUE)
set(action_client_MSG_DEPENDENCIES actionlib_msgs;std_msgs)
