cmake_minimum_required(VERSION 2.8.3)


SET(COMPILE_FLAGS "-Wno-deprecated")   #Флаги компилятора
add_definitions(${COMPILE_FLAGS})

project(altitude_cntrl)



find_package(catkin REQUIRED COMPONENTS 
				cv_bridge 
				geometry_msgs 
				image_transport 
				message_generation 
				pcl_conversions 
				pcl_ros 
				roscpp 
				sensor_msgs 
				std_msgs)

find_package(OpenCV REQUIRED)
catkin_package()


include_directories(	${catkin_INCLUDE_DIRS}
			${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



set(SOURCES main.cpp ransac.cpp)
add_executable(altitude_cntrl_nd ${SOURCES})
target_link_libraries(altitude_cntrl_nd ${catkin_LIBRARIES} 
					${OpenCV_LIBS} 
					${PCL_LIBRARIES})







