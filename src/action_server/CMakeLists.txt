cmake_minimum_required(VERSION 2.8.3)
project(action_server)



find_package(catkin REQUIRED COMPONENTS 
				cv_bridge 
				geometry_msgs 
				image_transport 
				message_generation 
				pcl_conversions 
				pcl_ros 
				roscpp 
				sensor_msgs
				visualization_msgs 
				actionlib_msgs
				std_msgs)

find_package(OpenCV REQUIRED)



FILE(GLOB ACTION_SRCS RELATIVE ${CMAKE_CURRENT_LIST_DIR}/action action/*.action)
add_action_files(
  DIRECTORY action
  FILES ${ACTION_SRCS}
)  


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)


catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########


include_directories(	${catkin_INCLUDE_DIRS}
			${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




set(SOURCES main.cpp RANSAC.cpp RANSAC.h passage_finder.cpp passage_finder.h)

add_executable(action_server_nd ${SOURCES})
target_link_libraries(action_server_nd	 	${catkin_LIBRARIES} 
						${OpenCV_LIBS} 
						${PCL_LIBRARIES})

add_dependencies(action_server_nd ${PROJECT_NAME}_gencpp)















