cmake_minimum_required(VERSION 2.8.3)
project(flow_to_odom)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
				geometry_msgs
                                cv_bridge 
				image_transport 
				message_generation 
				roscpp 
				sensor_msgs 
				std_msgs
                                tf)

find_package(OpenCV REQUIRED)


add_message_files(
  FILES
  OpticalFlow.msg
)

generate_messages(   
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)  


catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs std_msgs geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(flow_to_odom_nd flow_to_odom.cpp)
target_link_libraries(flow_to_odom_nd ${catkin_LIBRARIES})
add_dependencies(flow_to_odom_nd ${PROJECT_NAME}_gencpp)


