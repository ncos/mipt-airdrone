cmake_minimum_required(VERSION 2.8.3)
project(tf_manager)



find_package(catkin REQUIRED COMPONENTS 
				geometry_msgs 
				message_generation 
				roscpp 
				sensor_msgs
                                tf
				std_msgs)



catkin_package()

###########
## Build ##
###########


include_directories(${catkin_INCLUDE_DIRS})


set(SOURCES tf_manager.cpp)

add_executable(tf_manager_nd ${SOURCES})
target_link_libraries(tf_manager_nd ${catkin_LIBRARIES})















