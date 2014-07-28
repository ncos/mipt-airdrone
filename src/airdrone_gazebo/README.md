mipt-airdrone
=============

Multicopter autopilot system for CROC aerial robotics competition
airdrone_gazebo package

### Introduction
1. This package based on hector_quadrotor and provides simple simulator for airdrone project
2. Useful links: 
  - [Hector quadrotor](http://wiki.ros.org/hector_quadrotor "hector quadrotor wiki")

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


