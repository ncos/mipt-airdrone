cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy sensor_msgs std_msgs message_generation)
find_package(mavlink REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Attitude.msg
  ControllerOutput.msg
  Control.msg
  CurrentMission.msg
  FilteredPosition.msg
  Mavlink_RAW_IMU.msg
  MissionItem.msg
  RC.msg
  State.msg
  Status.msg
  VFR_HUD.msg
  Waypoint.msg
  WaypointList.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  APMCommand.srv
  SendWaypoint.srv
  SendWaypointList.srv
  XBEECommand.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs#   sensor_msgs#   std_msgs
)

if(NOT DEFINED mavlink_DIALECTS)
  list(APPEND mavlink_DIALECTS "ardupilotmega")
  list(APPEND mavlink_DIALECTS "pixhawk")
  list(APPEND mavlink_DIALECTS "common")
endif()

set(MAVLINK_DIALECT "ardupilotmega" CACHE STRING "MAVLink dialect selector")
set_property(CACHE MAVLINK_DIALECT PROPERTY STRINGS ${mavlink_DIALECTS})



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
#  INCLUDE_DIRS include
#  LIBRARIES roscopter 
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs message_runtime
  DEPENDS mavlink
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
)

add_definitions(
    -DMAVLINK_DIALECT=${MAVLINK_DIALECT}
)
## Declare a cpp library
# add_library(roscopter
#   src/${PROJECT_NAME}/roscopter.cpp
# )

## Declare a cpp executable
# add_executable(roscopter_node src/roscopter_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(roscopter_node roscopter_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(roscopter_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS roscopter roscopter_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_roscopter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
