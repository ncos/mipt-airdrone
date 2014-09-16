cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control)


set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS 
				geometry_msgs 
				message_generation 
				roscpp 
				sensor_msgs
				visualization_msgs 
				std_msgs
				std_srvs)




catkin_package()

###########
## Build ##
###########


include_directories(${catkin_INCLUDE_DIRS})


set(SOURCES main.cpp)

set(SFML_LIBS sfml-graphics sfml-window sfml-system )

add_executable(keyboard_control_nd ${SOURCES})
target_link_libraries(keyboard_control_nd ${catkin_LIBRARIES} 
                                          ${SFML_LIBS} )
















