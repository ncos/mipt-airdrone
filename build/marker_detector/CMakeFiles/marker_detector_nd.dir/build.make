# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ncos/mipt-airdrone/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ncos/mipt-airdrone/build

# Include any dependencies generated for this target.
include marker_detector/CMakeFiles/marker_detector_nd.dir/depend.make

# Include the progress variables for this target.
include marker_detector/CMakeFiles/marker_detector_nd.dir/progress.make

# Include the compile flags for this target's objects.
include marker_detector/CMakeFiles/marker_detector_nd.dir/flags.make

marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o: marker_detector/CMakeFiles/marker_detector_nd.dir/flags.make
marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o: /home/ncos/mipt-airdrone/src/marker_detector/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ncos/mipt-airdrone/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o"
	cd /home/ncos/mipt-airdrone/build/marker_detector && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/marker_detector_nd.dir/main.cpp.o -c /home/ncos/mipt-airdrone/src/marker_detector/main.cpp

marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/marker_detector_nd.dir/main.cpp.i"
	cd /home/ncos/mipt-airdrone/build/marker_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ncos/mipt-airdrone/src/marker_detector/main.cpp > CMakeFiles/marker_detector_nd.dir/main.cpp.i

marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/marker_detector_nd.dir/main.cpp.s"
	cd /home/ncos/mipt-airdrone/build/marker_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ncos/mipt-airdrone/src/marker_detector/main.cpp -o CMakeFiles/marker_detector_nd.dir/main.cpp.s

marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o.requires:
.PHONY : marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o.requires

marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o.provides: marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o.requires
	$(MAKE) -f marker_detector/CMakeFiles/marker_detector_nd.dir/build.make marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o.provides.build
.PHONY : marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o.provides

marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o.provides.build: marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o

marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o: marker_detector/CMakeFiles/marker_detector_nd.dir/flags.make
marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o: /home/ncos/mipt-airdrone/src/marker_detector/detector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ncos/mipt-airdrone/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o"
	cd /home/ncos/mipt-airdrone/build/marker_detector && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/marker_detector_nd.dir/detector.cpp.o -c /home/ncos/mipt-airdrone/src/marker_detector/detector.cpp

marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/marker_detector_nd.dir/detector.cpp.i"
	cd /home/ncos/mipt-airdrone/build/marker_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ncos/mipt-airdrone/src/marker_detector/detector.cpp > CMakeFiles/marker_detector_nd.dir/detector.cpp.i

marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/marker_detector_nd.dir/detector.cpp.s"
	cd /home/ncos/mipt-airdrone/build/marker_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ncos/mipt-airdrone/src/marker_detector/detector.cpp -o CMakeFiles/marker_detector_nd.dir/detector.cpp.s

marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o.requires:
.PHONY : marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o.requires

marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o.provides: marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o.requires
	$(MAKE) -f marker_detector/CMakeFiles/marker_detector_nd.dir/build.make marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o.provides.build
.PHONY : marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o.provides

marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o.provides.build: marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o

# Object files for target marker_detector_nd
marker_detector_nd_OBJECTS = \
"CMakeFiles/marker_detector_nd.dir/main.cpp.o" \
"CMakeFiles/marker_detector_nd.dir/detector.cpp.o"

# External object files for target marker_detector_nd
marker_detector_nd_EXTERNAL_OBJECTS =

/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libcv_bridge.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_calib3d.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_contrib.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_core.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_features2d.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_flann.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_gpu.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_highgui.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_imgproc.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_legacy.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_ml.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_nonfree.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_objdetect.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_photo.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_stitching.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_superres.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_video.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_videostab.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libimage_transport.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libpcl_ros_filters.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libpcl_ros_io.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libpcl_ros_tf.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_common.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_kdtree.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_octree.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_search.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_io.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_sample_consensus.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_filters.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_visualization.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_outofcore.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_features.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_segmentation.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_people.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_registration.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_recognition.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_keypoints.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_surface.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_tracking.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_apps.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_iostreams-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_serialization-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libqhull.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libOpenNI.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libflann_cpp_s.a
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libdynamic_reconfigure_config_init_mutex.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libnodeletlib.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libbondcpp.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libtinyxml.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libclass_loader.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libPocoFoundation.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/x86_64-linux-gnu/libdl.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libroslib.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosbag.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosbag_storage.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_program_options-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libtopic_tools.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libtf.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libtf2_ros.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libactionlib.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libmessage_filters.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libtf2.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libroscpp.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_signals-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_filesystem-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosconsole.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/liblog4cxx.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_regex-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librostime.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_date_time-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_system-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_thread-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libcpp_common.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libconsole_bridge.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_calib3d.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_contrib.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_core.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_features2d.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_flann.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_gpu.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_highgui.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_imgproc.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_legacy.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_ml.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_nonfree.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_objdetect.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_photo.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_stitching.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_superres.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_video.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libopencv_videostab.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libimage_transport.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libpcl_ros_filters.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libpcl_ros_io.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libpcl_ros_tf.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_common.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_kdtree.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_octree.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_search.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_io.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_sample_consensus.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_filters.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_visualization.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_outofcore.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_features.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_segmentation.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_people.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_registration.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_recognition.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_keypoints.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_surface.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_tracking.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libpcl_apps.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_iostreams-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_serialization-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libqhull.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libOpenNI.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libflann_cpp_s.a
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libdynamic_reconfigure_config_init_mutex.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libnodeletlib.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libbondcpp.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libtinyxml.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libclass_loader.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libPocoFoundation.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/x86_64-linux-gnu/libdl.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libroslib.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosbag.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosbag_storage.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_program_options-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libtopic_tools.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libtf.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libtf2_ros.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libactionlib.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libmessage_filters.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libtf2.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libroscpp.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_signals-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_filesystem-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosconsole.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/liblog4cxx.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_regex-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/librostime.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_date_time-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_system-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/libboost_thread-mt.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libcpp_common.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: /opt/ros/hydro/lib/libconsole_bridge.so
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: marker_detector/CMakeFiles/marker_detector_nd.dir/build.make
/home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd: marker_detector/CMakeFiles/marker_detector_nd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd"
	cd /home/ncos/mipt-airdrone/build/marker_detector && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/marker_detector_nd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
marker_detector/CMakeFiles/marker_detector_nd.dir/build: /home/ncos/mipt-airdrone/devel/lib/marker_detector/marker_detector_nd
.PHONY : marker_detector/CMakeFiles/marker_detector_nd.dir/build

marker_detector/CMakeFiles/marker_detector_nd.dir/requires: marker_detector/CMakeFiles/marker_detector_nd.dir/main.cpp.o.requires
marker_detector/CMakeFiles/marker_detector_nd.dir/requires: marker_detector/CMakeFiles/marker_detector_nd.dir/detector.cpp.o.requires
.PHONY : marker_detector/CMakeFiles/marker_detector_nd.dir/requires

marker_detector/CMakeFiles/marker_detector_nd.dir/clean:
	cd /home/ncos/mipt-airdrone/build/marker_detector && $(CMAKE_COMMAND) -P CMakeFiles/marker_detector_nd.dir/cmake_clean.cmake
.PHONY : marker_detector/CMakeFiles/marker_detector_nd.dir/clean

marker_detector/CMakeFiles/marker_detector_nd.dir/depend:
	cd /home/ncos/mipt-airdrone/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ncos/mipt-airdrone/src /home/ncos/mipt-airdrone/src/marker_detector /home/ncos/mipt-airdrone/build /home/ncos/mipt-airdrone/build/marker_detector /home/ncos/mipt-airdrone/build/marker_detector/CMakeFiles/marker_detector_nd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : marker_detector/CMakeFiles/marker_detector_nd.dir/depend

