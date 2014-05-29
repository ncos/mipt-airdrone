mipt-airdrone
=============

Multicopter autopilot system for CROC aerial robotics competition

BRIEF MANUAL:

To build the project type:
 go to project directory and enter
 - catkin_make                                          // To just build the project
 // To build project files for eclipse:
 - catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8
 alternatively via bash scripts:
 - rebuild                                              // To just build the project
 - rebuild_we                                           // To build project files for eclipse

To start copter type:
 - roslaunch airdrone_launch airdrone.launch            // To start core nodes for simulation
 - roslaunch airdrone_launch airdrone_real.launch       // To start nodes for the real robot connected to the laptop
 - roslaunch airdrone_launch airdrone_simulator.launch  // To open Rviz and Gazebo for simulation
 alternatively via bash scripts (need to manually configure paths):
 - airdrone_launch                                      // To start core nodes for simulation
 - airdrone_real_launch                                 // To start nodes for the real robot connected to the laptop
 - airdrone_test                                        // To open Rviz and Gazebo for simulation

To change the spawn point in Gazebo:
1) Go to .../mipt-airdrone/src/airdrone_launch/launch/airdrone_simulator.launch
2) Set the coordinates by adjusting <arg name="x/y/z" value="???"/>
