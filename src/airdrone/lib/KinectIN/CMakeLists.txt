cmake_minimum_required(VERSION 2.8) # Проверка версии CMake.
                                    # Если версия установленой программы
                                    # старее указаной, произайдёт аварийный выход.

#SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wcpp" )
#SET( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wcpp" ) #Флаги компилятора

project(KinectIN)			    # Название проекта

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

include_directories(	${catkin_INCLUDE_DIRS}
			${PCL_INCLUDE_DIRS}
			${VTK_INCLUDE_DIRS}	)


set(VTK_LIBS libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)
set(SOURCE_LIB kin_control.cpp kin_process.cpp kin.h)	  # Установка переменной со списком исходников


add_library(KinectIN STATIC ${SOURCE_LIB}) # Создание статической библиотеки
target_link_libraries(KinectIN 	${catkin_LIBRARIES} 
				${OpenCV_LIBS} 
				${PCL_LIBRARIES}
				${VTK_LIBS})




