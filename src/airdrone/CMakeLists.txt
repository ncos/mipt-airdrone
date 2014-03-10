cmake_minimum_required(VERSION 2.8.3)

#SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wcpp" )
#SET( CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE} -Wcpp" ) #Флаги компилятора
SET(COMPILE_FLAGS "-Wno-deprecated")   #Флаги компилятора
add_definitions(${COMPILE_FLAGS})

project(airdrone)



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

###########
## Build ##
###########

set(CUSTOM_LIBRARY_DIRS lib/PCV lib/KinectIN lib/Pose)

include_directories(	${catkin_INCLUDE_DIRS}
			${PCL_INCLUDE_DIRS}
			${VTK_INCLUDE_DIRS}
			${CUSTOM_LIBRARY_DIRS}	)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(VTK_LIBS 	libvtkCommon.so 
		libvtkFiltering.so 
		libvtkRendering.so 
		libvtkGraphics.so	)

set(CUSTOM_LIBS PCV
		KinectIN
		Pose	)

#add_subdirectory(${CUSTOM_LIBRARY_DIRS})
foreach(SUBDIR ${CUSTOM_LIBRARY_DIRS}) 
    add_subdirectory(${SUBDIR}) 
endforeach(SUBDIR) 

add_executable(airdrone_com_node core/airdrone_com.cpp)
target_link_libraries(airdrone_com_node ${catkin_LIBRARIES} 
					${OpenCV_LIBS} 
					${PCL_LIBRARIES}
					${VTK_LIBS}
					${CUSTOM_LIBS})

















