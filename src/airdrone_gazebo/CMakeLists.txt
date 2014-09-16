cmake_minimum_required(VERSION 2.8.3)
project(airdrone_gazebo)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs std_srvs geometry_msgs nav_msgs tf topic_tools cmake_modules gazebo_ros_control hardware_interface controller_interface)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})


#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetBias.srv
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Altimeter.msg
  AttitudeCommand.msg
  Compass.msg
  ControllerState.msg
  HeadingCommand.msg
  HeightCommand.msg
  MotorCommand.msg
  MotorPWM.msg
  MotorStatus.msg
  PositionXYCommand.msg
  RawImu.msg
  RawMagnetic.msg
  RawRC.msg
  RC.msg
  RuddersCommand.msg
  ServoCommand.msg
  Supply.msg
  ThrustCommand.msg
  VelocityXYCommand.msg
  VelocityZCommand.msg
  YawrateCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

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
    DEPENDS gazebo Boost
    LIBRARIES hector_quadrotor_propulsion hector_quadrotor_aerodynamics hector_quadrotor_controller_gazebo
    CATKIN_DEPENDS roscpp std_srvs std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime tf topic_tools hardware_interface controller_interface
    INCLUDE_DIRS
)

###########
## Build ##
###########

# simulator plugins
add_library(diffdrive_plugin_6w drone/plugins/src/diffdrive_plugin_6w.cpp)
target_link_libraries(diffdrive_plugin_6w ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(diffdrive_plugin_multi_wheel drone/plugins/src/diffdrive_plugin_multi_wheel.cpp)
target_link_libraries(diffdrive_plugin_multi_wheel ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_reset_plugin drone/plugins/src/reset_plugin.cpp)
target_link_libraries(hector_gazebo_reset_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_ros_imu drone/plugins/src/gazebo_ros_imu.cpp)
target_link_libraries(hector_gazebo_ros_imu ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_imu ${PROJECT_NAME}_generate_messages_cpp)

add_library(hector_gazebo_ros_magnetic drone/plugins/src/gazebo_ros_magnetic.cpp)
target_link_libraries(hector_gazebo_ros_magnetic ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_ros_gps drone/plugins/src/gazebo_ros_gps.cpp)
target_link_libraries(hector_gazebo_ros_gps ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_ros_sonar drone/plugins/src/gazebo_ros_sonar.cpp)
target_link_libraries(hector_gazebo_ros_sonar ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_servo_plugin drone/plugins/src/servo_plugin.cpp)
target_link_libraries(hector_servo_plugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_ros_baro drone/plugins/src/gazebo_ros_baro.cpp)
target_link_libraries(hector_gazebo_ros_baro ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_baro ${PROJECT_NAME}_generate_messages_cpp)

add_library(hector_gazebo_quadrotor_simple_controller drone/plugins/src/gazebo_quadrotor_simple_controller.cpp)
target_link_libraries(hector_gazebo_quadrotor_simple_controller ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_quadrotor_propulsion drone/plugins/src/gazebo_quadrotor_propulsion.cpp)
target_link_libraries(hector_gazebo_quadrotor_propulsion ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} hector_quadrotor_propulsion)
add_dependencies(hector_gazebo_quadrotor_propulsion ${PROJECT_NAME}_generate_messages_cpp)

add_library(hector_gazebo_quadrotor_aerodynamics drone/plugins/src/gazebo_quadrotor_aerodynamics.cpp)
target_link_libraries(hector_gazebo_quadrotor_aerodynamics ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} hector_quadrotor_aerodynamics)

add_library(hector_quadrotor_propulsion drone/plugins/src/quadrotor_propulsion.cpp)
target_link_libraries(hector_quadrotor_propulsion ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(hector_quadrotor_propulsion ${PROJECT_NAME}_generate_messages_cpp)

add_library(hector_quadrotor_aerodynamics drone/plugins/src/quadrotor_aerodynamics.cpp)
target_link_libraries(hector_quadrotor_aerodynamics ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# controllers:
add_library(hector_quadrotor_pose_controller drone/plugins/src/pose_controller.cpp)
target_link_libraries(hector_quadrotor_pose_controller hector_quadrotor_controller_gazebo)

add_library(hector_quadrotor_twist_controller drone/plugins/src/twist_controller.cpp)
target_link_libraries(hector_quadrotor_twist_controller hector_quadrotor_controller_gazebo ${BOOST_LIBRARIES})

add_library(hector_quadrotor_motor_controller drone/plugins/src/motor_controller.cpp)
target_link_libraries(hector_quadrotor_motor_controller hector_quadrotor_controller_gazebo)

add_library(hector_quadrotor_controller_gazebo drone/plugins/src/quadrotor_hardware_gazebo.cpp drone/plugins/src/quadrotor_interface.cpp drone/plugins/src/pid.cpp)
target_link_libraries(hector_quadrotor_controller_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(hector_quadrotor_controller_gazebo ${PROJECT_NAME}_generate_messages_cpp)

# helper nodes:
add_executable(message_to_tf drone/plugins/src/message_to_tf.cpp)
target_link_libraries(message_to_tf ${catkin_LIBRARIES})


