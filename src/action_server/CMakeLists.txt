cmake_minimum_required(VERSION 2.8.3)

#SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wcpp" )
#SET( CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE} -Wcpp" ) #Флаги компилятора
SET(COMPILE_FLAGS "-Wno-deprecated")   #Флаги компилятора
add_definitions(${COMPILE_FLAGS})

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



add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
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
			${PCL_INCLUDE_DIRS}
			${VTK_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(VTK_LIBS 	libvtkCommon.so 
		libvtkFiltering.so 
		libvtkRendering.so 
		libvtkGraphics.so	)



set(SOURCES main.cpp RANSAC.cpp RANSAC.h passage_finder.cpp passage_finder.h)

add_executable(action_server_nd ${SOURCES})
target_link_libraries(action_server_nd	 	${catkin_LIBRARIES} 
						${OpenCV_LIBS} 
						${PCL_LIBRARIES}
						${VTK_LIBS})

















