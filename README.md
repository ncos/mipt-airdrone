mipt-airdrone
=============

Multicopter autopilot system for CROC aerial robotics competition

## BRIEF MANUAL:

### To build the project type:
1. Manually. Go to project directory and enter:
  - catkin_make                                          *(To just build the project)*
  - catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8 *(To build project files for eclipse)*

2. Via bash scripts (need to manually edit bash scripts to configure paths). Go to *.../mipt-airdrone/contrib/launchers* or add the launchers to your system PATH and enter:
  - rebuild                                              *(To just build the project)*
  - rebuild_we                                           *(To build project files for eclipse)*

### To start copter type:
1. Manually. Type in the terminal:
  - roslaunch airdrone_launch airdrone.launch            *(To start core nodes for simulation)*
  - roslaunch airdrone_launch airdrone_real.launch       *(To start nodes for the real robot connected to the laptop)*
  - roslaunch airdrone_launch airdrone_simulator.launch  *(To open Rviz and Gazebo for simulation)*

2. Via bash scripts (need to manually edit bash scripts to configure paths). Go to *.../mipt-airdrone/contrib/launchers* or add the launchers to your system PATH and enter:
  - airdrone_launch                                      *(To start core nodes for simulation)*
  - airdrone_real_launch                                 *(To start nodes for the real robot connected to the laptop)*
  - airdrone_test                                        *(To open Rviz and Gazebo for simulation)*

### To change the spawn point in Gazebo:
 1. Go to *.../mipt-airdrone/src/airdrone_launch/launch/airdrone_simulator.launch*
 2. Set the coordinates by adjusting `<arg name="x/y/z" value="???"/>`

### To change the default gazebo camera position:
 1. Go to *.../mipt-airdrone/src/airdrone_gazebo/worlds/<...>.world*
 2. Find the `<camera name='some_camera_name'>`
 3. Change the <pose> tag 				*(There will be the `<pose>X Y Z R P Y</pose>` string, put in the correct numbers)*

### To change map:
 1. **DO** it carefully!
 2. If you stil decided to add a custom map create a model in google Sketch Up (free to download)
 3. The model should have .dae extension and the altitude of the floor at the robot spawn point should be zero
 4. It is important that it would be zero!
 5. Rename the .dae file to "test_chamber0.dae". **Do NOT** rename the generated folder with images! (sometimes it will not be generated) But if you desperately want to rename, edit the .dae file so it can resolve all paths.
 6. Put the files in .../mipt-airdrone/src/airdrone_gazebo/Media/models folder
 7. If the model jumps on its spawn point than you probably did not set the world floor altitude to zero. You can probably edit the *...mipt-airdrone/src/airdrone_gazebo/worlds/test_chamber0.world* to fix that.

### To edit the robot model:
 1. Be careful here too)
 2. Go to *.../mipt-airdrone/src/airdrone_launch/inherited_from_hector/urdf*
 3. Edit the quadrotor_with_kinect.urdf.xacro file to set up kinect position and orientation
 4. Other features are coming soon...


