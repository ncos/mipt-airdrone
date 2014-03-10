cmake_minimum_required(VERSION 2.8.3)

#SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wcpp" )
#SET( CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE} -Wcpp" ) #Флаги компилятора
SET(COMPILE_FLAGS "-Wno-deprecated")   #Флаги компилятора
add_definitions(${COMPILE_FLAGS})

project(rgbd_shrinker)



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
				std_msgs)

find_package(OpenCV REQUIRED)
catkin_package()

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


add_executable(rgbd_shrinker_node main.cpp)
target_link_libraries(rgbd_shrinker_node 	${catkin_LIBRARIES} 
						${OpenCV_LIBS} 
						${PCL_LIBRARIES}
						${VTK_LIBS})

















