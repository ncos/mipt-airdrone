cmake_minimum_required(VERSION 2.8.3)
project(lib_rgbdtools)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS 
				cv_bridge
				pcl_conversions 
				pcl_ros 
				roscpp)

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Boost:
find_package(Boost REQUIRED)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})

# Dependencies - PCL:
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/)
# Dependencies - Suitesparse:
find_package(SuiteSparse REQUIRED)


# Dependencies - G2O:
find_package(G2O REQUIRED)



include_directories(	include
                        ${catkin_INCLUDE_DIRS}
			${PCL_INCLUDE_DIRS}
			${OpenCV_INCLUDE_DIRS}
			${VTK_INCLUDE_DIRS}
			${Boost_INCLUDE_DIRS}
			${SUITESPARSE_INCLUDE_DIRS}
			${G2O_INCLUDE_DIR})


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES rgbdtools
   DEPENDS SuiteSparse)

####################################################
# Build rgbdtools library.

add_library (rgbdtools SHARED
  src/rgbd_frame.cpp
  src/rgbd_keyframe.cpp
  src/rgbd_util.cpp
  src/map_util.cpp
  src/features/feature_detector.cpp
  src/features/orb_detector.cpp
  src/features/surf_detector.cpp
  src/features/gft_detector.cpp
  src/features/star_detector.cpp
  src/registration/motion_estimation.cpp
  src/registration/motion_estimation_icp_prob_model.cpp
  src/graph/keyframe_graph_detector.cpp
  src/graph/keyframe_graph_solver.cpp
  src/graph/keyframe_graph_solver_g2o.cpp)

target_link_libraries (rgbdtools
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${G2O_LIBRARIES})
 
####################################################
# Build global cloud align applications.

add_executable(global_cloud_align apps/global_cloud_align.cpp)
target_link_libraries (global_cloud_align rgbdtools ${OpenCV_LIBRARIES}) 

####################################################
# Build keyframe associations applications.

add_executable(keyframe_associations apps/keyframe_associations.cpp)
target_link_libraries (keyframe_associations rgbdtools ${OpenCV_LIBRARIES}) 

