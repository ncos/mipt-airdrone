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


FILE(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_LIST_DIR}/msg msg/*.msg)

add_message_files(
  #DIRECTORY action
  FILES ${MSG_FILES}
)

FILE(GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_LIST_DIR}/srv srv/*.srv)

add_service_files(
  #DIRECTORY action
  FILES ${SRV_FILES}
)


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
















