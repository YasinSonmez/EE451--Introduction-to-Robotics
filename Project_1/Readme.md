# Project 1: Robot Control Using Moveit Package

## Introduction

This project is focused on creating a PR robot using urdf files and then moving it using the ROS Moveit interface.

<img src="https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Images/pr_robot.gif" width="800">

## Dependecies
- ROS
- Moveit

## Running the Code
- First open the Project_1 folder and build the catkin workspace:
```
catkin build
```
- Then launch the robot config file and open the RViz:
```
roslaunch pr_config demo.launch
```
- Then run the mover code using the created node:
```
rosrun pr_move pr_move_node
```
- In the RViz open RvizVisualToolsGui panel and press 'next' in the RvizVisualToolsGui window. You should see the robot plan and move through it's workspace.
- After robot moves through it's workspace the position of the point should be printed to console along with it's translated and rotated positions.

## Files
- [`pr.urdf`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Project_1/src/pr_config/urdf_files/pr.urdf) This is the description of robot using urdf format.
- [`pr_move_node.cpp`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Project_1/src/pr_move/src/pr_move_node.cpp) C++ code for moving the robot and getting the ccordinates using Moveit API.

## Creation Process
Here are the steps I've taken to create this project:
1. Written the urdf file
2. Created config package pr_config of PR robot using moveit setup assistant(and defined move groups):
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
3. Created package pr_run to move the robot using Moveit
   - Edited cmakelists.txt to include necessary packages and made the code executable 
