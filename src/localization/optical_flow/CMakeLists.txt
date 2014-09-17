cmake_minimum_required(VERSION 2.8.3)
project(optical_flow)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
				geometry_msgs
                                cv_bridge 
				image_transport 
				message_generation 
				roscpp 
				sensor_msgs 
				std_msgs
                                tf)

find_package(OpenCV REQUIRED)


add_message_files(
  FILES
  OpticalFlow.msg
)

generate_messages(   
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)  


catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs std_msgs geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})

# This macro will list all folders in a directory
MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            SET(dirlist ${dirlist} ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

set(GPU_FLOW_MODULES_PATH ${CMAKE_CURRENT_SOURCE_DIR}/optical_flow_drivers/gpu_based)
set(CPU_FLOW_MODULES_PATH ${CMAKE_CURRENT_SOURCE_DIR}/optical_flow_drivers/cpu_based)

SUBDIRLIST(GPU_SUBDIRS ${GPU_FLOW_MODULES_PATH})
SUBDIRLIST(CPU_SUBDIRS ${CPU_FLOW_MODULES_PATH})


# Different realizations of optical flow algorithm
# CPU based:
MESSAGE("Parsing CPU optical flow directories: ${CPU_SUBDIRS}")
FOREACH(subdir ${CPU_SUBDIRS})
    set(ROOT ${CPU_FLOW_MODULES_PATH}/${subdir})
    MESSAGE("\tCreating node 'of_cpu_${subdir}_nd'")
    set(SOURCES ${ROOT}/main.cpp ${ROOT}/flow.cpp ${ROOT}/flow.h)
    add_executable(of_cpu_${subdir}_nd ${SOURCES})
    target_link_libraries(of_cpu_${subdir}_nd ${catkin_LIBRARIES} 
    			                      ${OpenCV_LIBS})
    add_dependencies(of_cpu_${subdir}_nd ${PROJECT_NAME}_gencpp)
ENDFOREACH()


# GPU based:
MESSAGE("Parsing GPU optical flow directories: ${GPU_SUBDIRS}")
FOREACH(subdir ${GPU_SUBDIRS})
    set(ROOT ${GPU_FLOW_MODULES_PATH}/${subdir})
    MESSAGE("\tCreating node 'of_gpu_${subdir}_nd'")
    set(SOURCES ${ROOT}/main.cpp ${ROOT}/flow.cpp ${ROOT}/flow.h)
    add_executable(of_gpu_${subdir}_nd ${SOURCES})
    target_link_libraries(of_gpu_${subdir}_nd ${catkin_LIBRARIES} 
    			                      ${OpenCV_LIBS})
    add_dependencies(of_gpu_${subdir}_nd ${PROJECT_NAME}_gencpp)
ENDFOREACH()


# To convert from 'flow' message to '/odom' transform
add_executable(raw_flow_to_odom_nd ${CMAKE_CURRENT_SOURCE_DIR}/optical_flow_drivers/raw_flow_to_odom.cpp)
target_link_libraries(raw_flow_to_odom_nd ${catkin_LIBRARIES})
add_dependencies(raw_flow_to_odom_nd ${PROJECT_NAME}_gencpp)


