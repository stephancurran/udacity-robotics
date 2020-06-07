# Intro
These are the projects for the Udacity Robotics Software Engineer Nanodegree Program.

# Build My World
Everything except the plugin are used in later projects; this is included for completeness.  
The plugin is built as follows (instructions from Udacity lesson):  
$ `cd /home/workspace/build_my_world`  
$ `mkdir build`  
$ `cd build/`  
$ `cmake ../`  
$ `make # You might get errors if your system is not up to date!`  
$ `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/workspace/myrobot/build`  

Run with `gazebo build_my_world/world/the_office.world`  

# Where Am I?
Uses the Map Server, ACML, and Move Base packages to help localise and navigate.  
Launch Gazebo and Rviz with:  
`roslaunch my_robot world.launch`  

Launch Map Server, ACML, and Move Base with:  
`roslaunch my_robot acml.launch`  

Optionally, start teleop with:  
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`  

