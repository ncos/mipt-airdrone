cmake_minimum_required(VERSION 2.8.3)
project(optical_flow)



find_package(catkin REQUIRED COMPONENTS 
				geometry_msgs 
				image_transport 
				message_generation 
				roscpp 
				sensor_msgs 
				std_msgs)

find_package(OpenCV REQUIRED)
catkin_package()


include_directories(	${catkin_INCLUDE_DIRS})





set(SOURCES main.cpp flow.cpp flow.h)
add_executable(optical_flow_nd ${SOURCES})

target_link_libraries(optical_flow_nd ${catkin_LIBRARIES} 
			              ${OpenCV_LIBS})







