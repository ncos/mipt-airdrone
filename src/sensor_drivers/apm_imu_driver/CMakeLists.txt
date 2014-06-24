cmake_minimum_required(VERSION 2.8.3)
project(apm_imu_driver)



find_package(catkin REQUIRED COMPONENTS 
				geometry_msgs
				message_generation 
				roscpp 
				sensor_msgs 
				std_msgs)


add_message_files(
  FILES
  Attitude.msg
  Control.msg
  Mavlink_RAW_IMU.msg
  RC.msg
  State.msg
  VFR_HUD.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(${catkin_INCLUDE_DIRS})



add_executable(apm_imu_driver_nd main.cpp)
target_link_libraries(apm_imu_driver_nd ${catkin_LIBRARIES})
add_dependencies(apm_imu_driver_nd ${PROJECT_NAME}_gencpp)


