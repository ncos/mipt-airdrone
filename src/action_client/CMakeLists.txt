cmake_minimum_required(VERSION 2.8.3)
project(action_client)



find_package(catkin REQUIRED COMPONENTS 
				geometry_msgs 
				message_generation 
				pcl_ros
				rospy
				roslib
				smach
				smach_ros
				actionlib
				sensor_msgs
				actionlib_msgs
				std_msgs)


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





