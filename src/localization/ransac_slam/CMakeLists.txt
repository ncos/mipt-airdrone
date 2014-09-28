cmake_minimum_required(VERSION 2.8.3)
project(ransac_slam)



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
				tf
                                tf_conversions
				laser_geometry
				std_msgs)

find_package(OpenCV REQUIRED)


add_message_files(
  FILES
  LineMap.msg
)

generate_messages(   
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)  


catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs std_msgs geometry_msgs)

###########
## Build ##
###########


include_directories(	${catkin_INCLUDE_DIRS}
			${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




set(SOURCES main.cpp lines.cpp passage.h passage.cpp lines.h advanced_types.cpp advanced_types.h da_vinci.cpp da_vinci.h)

add_executable(ransac_slam_nd ${SOURCES})
target_link_libraries(ransac_slam_nd	 	${catkin_LIBRARIES} 
						${OpenCV_LIBS} 
						${PCL_LIBRARIES})
add_dependencies(ransac_slam_nd ${PROJECT_NAME}_gencpp)















