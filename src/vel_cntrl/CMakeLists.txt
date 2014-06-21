cmake_minimum_required(VERSION 2.8.3)

#SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wcpp" )
#SET( CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE} -Wcpp" ) #Флаги компилятора
SET(COMPILE_FLAGS "-Wno-deprecated")   #Флаги компилятора
add_definitions(${COMPILE_FLAGS})

project(vel_cntrl)



find_package(catkin REQUIRED COMPONENTS 
				geometry_msgs 
				message_generation 
				roscpp 
				sensor_msgs
				visualization_msgs 
				std_msgs
				std_srvs)


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

###########
## Build ##
###########


include_directories(${catkin_INCLUDE_DIRS})





set(SOURCES main.cpp)	  # Установка переменной со списком исходников

add_executable(vel_cntrl_nd ${SOURCES})
target_link_libraries(vel_cntrl_nd ${catkin_LIBRARIES})
add_dependencies(vel_cntrl_nd ${PROJECT_NAME}_gencpp)
















