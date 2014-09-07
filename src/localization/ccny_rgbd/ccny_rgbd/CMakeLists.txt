cmake_minimum_required(VERSION 2.8.3)
project(ccny_rgbd)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation message_runtime pcl_ros nav_msgs std_srvs tf dynamic_reconfigure cv_bridge sensor_msgs geometry_msgs visualization_msgs image_transport image_geometry nodelet lib_rgbdtools)

include_directories(include ${catkin_INCLUDE_DIRS} ${lib_rgbdtools_INCLUDE_DIRS} )
add_definitions( "-O3" )

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddManualKeyframe.srv
  GenerateGraph.srv
  Load.srv
  PublishKeyframe.srv
  PublishKeyframes.srv
  Save.srv
  SolveGraph.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_srvs
)

# Generate dynamic parameters
generate_dynamic_reconfigure_options(
  cfg/FeatureDetector.cfg
  cfg/GftDetector.cfg
  cfg/OrbDetector.cfg
  cfg/StarDetector.cfg
  cfg/SurfDetector.cfg
  cfg/RGBDImageProc.cfg
)

################################################################
# Dependencies
################################################################


# OpenCV
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_libraries(${OpenCV_LIBRARIES})

# Octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_libraries(${PCL_LIBRARIES})

#g2o
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/)
find_package(G2O REQUIRED)
#set(G2O_LIBRARIES 
#  g2o_core 
#  g2o_stuff
#  g2o_solver_cholmod 
#  g2o_solver_csparse 
#  g2o_types_slam3d
#  cholmod)

# boost
find_package(Boost REQUIRED )

catkin_package(
   CATKIN_DEPENDS lib_rgbdtools
   DEPENDS octomap)

################################################################
# Build visual odometry application
################################################################

add_executable(visual_odometry_node 
  src/node/visual_odometry_node.cpp
  src/apps/visual_odometry.cpp
  src/util.cpp)
  
#add_dependencies(example_node ${PROJECT_NAME}_gencfg)
add_dependencies(visual_odometry_node     ${${PROJECT_NAME}_EXPORTED_TARGETS})  
target_link_libraries (visual_odometry_node
  rgbdtools
  boost_signals
  boost_system
  boost_filesystem
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})
  
################################################################
# Build keyframe mapper application
################################################################

add_executable(keyframe_mapper_node
  src/node/keyframe_mapper_node.cpp
  src/apps/keyframe_mapper.cpp
  src/util.cpp)
  
add_dependencies(keyframe_mapper_node     ${${PROJECT_NAME}_EXPORTED_TARGETS})  
target_link_libraries (keyframe_mapper_node
  rgbdtools
  boost_signals
  boost_system
  boost_filesystem
  boost_regex
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${G2O_LIBRARIES})
  
################################################################
# Build feature viewer application
################################################################

add_executable(feature_viewer_node 
  src/node/feature_viewer_node.cpp
  src/apps/feature_viewer.cpp
  src/util.cpp)
  
add_dependencies(feature_viewer_node     ${${PROJECT_NAME}_EXPORTED_TARGETS})  
target_link_libraries (feature_viewer_node
  rgbdtools
  boost_signals
  boost_system
  boost_filesystem
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES})
  
################################################################
# Build image proc application
################################################################

add_library(rgbd_image_proc_app 
  src/apps/rgbd_image_proc.cpp
  src/util.cpp)

add_dependencies(rgbd_image_proc_app     ${${PROJECT_NAME}_EXPORTED_TARGETS})  
target_link_libraries (rgbd_image_proc_app  
  rgbdtools
  boost_signals
  boost_system
  boost_filesystem
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES})
  
add_executable(rgbd_image_proc_node src/node/rgbd_image_proc_node.cpp)
add_library(rgbd_image_proc_nodelet src/nodelet/rgbd_image_proc_nodelet.cpp)

add_dependencies(rgbd_image_proc_node    ${${PROJECT_NAME}_EXPORTED_TARGETS})  
add_dependencies(rgbd_image_proc_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})  

target_link_libraries(rgbd_image_proc_node    rgbd_image_proc_app)
target_link_libraries(rgbd_image_proc_nodelet rgbd_image_proc_app)

