mipt-airdrone
=============

Multicopter autopilot system for CROC aerial robotics competition

### airdrone_gazebo
1. This package based on hector_quadrotor and provides simple simulator for airdrone project
2. Useful ROS HOWTO's: 
  - [ROS Hydro installation](http://wiki.ros.org/hydro/Installation/Ubuntu "Read this to install ROS on your system")
  - [ROS Tutorial](http://wiki.ros.org/ROS/Tutorials "This is a brief ROS tutorial. Helps to understand basic ROS concepts")
  - [Actionlib](http://wiki.ros.org/actionlib "Look into the actionlib tutorials if you want to deal with the code")
  - [Smach](http://wiki.ros.org/smach/Tutorials "The state mashine library that controls the drone's behavior")
  - [Hector quadrotor](http://wiki.ros.org/hector_quadrotor "We use this project for simulation")
3. Some additional packages are to be installed to successfully compile the code. I'll maybe write about it one day, but at the moment you have to look at the compiler errors and set everything up yourself ;)
4. To use bash scripts launch *.../mipt-airdrone/contrib/INSTALL.py*. It will generate scripts for you and place them to *.../mipt-airdrone/contrib/launchers_gen*. It will also add these to your system PATH. Please take into account that your *~/.bashrc* will be modified! If you want to move the *.../mipt-airdrone* folder do it, and after that relaunch *.../mipt-airdrone/contrib/INSTALL.py*. The paths will (hopefully) be reconfigured automatically.
5. *INSTALL.py* when launched also creates nice desktop shortcuts for your convenience. They are named *Ad...smth...* and are easily found via Unity search. (Press Dash Home button and type *'Ad'*) The config files are placed into *~/.local/share/applications* folder.

### To change the spawn point in Gazebo:
 1. Go to *.../mipt-airdrone/src/airdrone_launch/launch/airdrone_simulator.launch*
 2. Set the coordinates by adjusting `<arg name="x/y/z" value="???"/>`

### To change the default gazebo camera position:
 1. Go to *.../mipt-airdrone/src/airdrone_gazebo/worlds/<...>.world*
 2. Find the `<camera name='some_camera_name'>`
 3. Change the <pose> tag 				*(There will be the `<pose>X Y Z R P Y</pose>` string, put in the correct numbers)*

### To change map:
 1. **DO** it carefully!
 2. If you still decided to add a custom map create a model in Google Sketch Up (free to download)
 3. The model should have .dae extension and the altitude of the floor at the robot spawn point should be zero
 4. It is important that it would be zero!
 5. Rename the .dae file to "test_chamber0.dae". **Do NOT** rename the generated folder with images! (sometimes it will not be generated) But if you desperately want to rename, edit the .dae file so it can resolve all paths.
 6. Put the files in *.../mipt-airdrone/src/airdrone_gazebo/Media/models* folder
 7. If the model jumps on its spawn point than you probably did not set the world floor altitude to zero. You can probably edit the *...mipt-airdrone/src/airdrone_gazebo/worlds/test_chamber0.world* to fix that.

### To edit the robot model:
 1. Be careful here too)
 2. Go to *.../mipt-airdrone/src/airdrone_gazebo/quadrotor_description*
 3. Edit the urdf/quadrotor_with_kinect.urdf.xacro file to set up kinect position and orientation
 4. Other features are coming soon...


