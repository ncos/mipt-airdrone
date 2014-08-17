CCNY RGB-D tools 
===================================

Copyright (C) 2013, City University of New York  
CCNY Robotics Lab  
<http://robotics.ccny.cuny.edu/>
 
Overview
-----------------------------------

The stack contains ROS applications for visual odometry and mapping using RGB-D cameras. 
The applications are built on top of the [rgbdtools](https://github.com/ccny-ros-pkg/rgbdtools.git) library.

This code is at an experimental stage, and licensed under the GPLv3 license.

Get started
-----------------------------------

1. Install dependencies: *ros-indigo-libg2o* and *SuiteSparse* (if it is not already installed)
2. A standalone OpenCV is needed (some features are not supported by ros opencv). It is better to use the latest stable release
3. Build the package via catkin_make
4. Launch *roslaunch ccny_launch ccny_rgbd-demo.launch* to check the project out

More info
-----------------------------------

Documentation:

 * ROS wiki: http://ros.org/wiki/ccny_rgbd_tools
 * API: http://ros.org/doc/fuerte/api/ccny_rgbd/html/

Videos:
 * Visual odometry & 3D mapping: http://youtu.be/YE9eKgek5pI
 * Feature viewer: http://youtu.be/kNkrPuBu8JA
