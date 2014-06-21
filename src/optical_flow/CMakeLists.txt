cmake_minimum_required(VERSION 2.8.3)
project(optical_flow)



find_package(catkin REQUIRED COMPONENTS 
				geometry_msgs 
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
  DEPENDENCIES  
  std_msgs  
)  


catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(${catkin_INCLUDE_DIRS})


set(SOURCES main.cpp flow.cpp flow.h)
add_executable(optical_flow_nd ${SOURCES})

target_link_libraries(optical_flow_nd ${catkin_LIBRARIES} 
			              ${OpenCV_LIBS})


add_executable(px_flow_to_odom_nd px_flow_to_odom.cpp)
target_link_libraries(px_flow_to_odom_nd ${catkin_LIBRARIES} )



