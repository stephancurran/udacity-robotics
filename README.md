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



# Go Chase It!
The `ball_chaser` package makes the robot move toward the white ball by checking the camera input for white pixels, and aims for the midpoint of the leftmost and rightmost found. Start with:  
`roslaunch my_robot world.launch`  
`roslaunch ball_chaser ball_chaser.launch`  



# Where Am I?
Uses the Map Server, ACML, and Move Base packages to help localize and navigate.  
Launch Gazebo and Rviz with:  
`roslaunch my_robot world.launch`  

Launch Map Server, ACML, and Move Base with:  
`roslaunch my_robot acml.launch`  

Optionally, start teleop with:  
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`  



# Map My World
This project uses `rtabmap_ros` to map out the environment.  Start the world and move the robot around with `teleop` :  
`roslaunch my_robot world.launch`  
`roslaunch my_robot mapping.launch`  
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`  

This image shows the route taken and the resulting map.  

![The route taken when mapping](images/rtab_route.png)



And here's the occupancy map.

![Occupancy map](images/occupancy_map.png)



The resulting database is available [here](https://drive.google.com/file/d/1HUX0t3hWFJUYfYJY_8GsWH--63Zs6orE/view?usp=sharing), and can be put in `my_robot/maps/rtabmap.db` to  be used for localization, similar to the *Where Am I?* project.  
`roslaunch my_robot world.launch`  
`roslaunch my_robot localization.launch`  
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`  



Here's a side-by-side of the RGB and depth images from the robot's starting point.

![RGB vs Depth comparison](images/rgb_depth_comparison.png)
