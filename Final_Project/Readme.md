# Final Project: Warehouse Robotic Tasks

## Introduction

This project is focused on
- Creating an RRP robot
- Spawning objects with different sizes on different coordinates
- Picking & Placing using the ROS Moveit interface

<img src="https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Images/RRP_cylinders.png" width="800">


## Dependencies
- ROS
- Moveit

## Running the Code
- First, open the Final_Project folder and build the catkin workspace:
```
catkin_make
```
- Then read the parameters from the YAML file:
```
roslaunch function read_params.launch
```
- Then launch the robot config file and open the RViz:
```
roslaunch rrp_robot demo.launch
```
- Then run the pick & place code that also spawns the cylinders:
```
rosrun function function_node
```
- First, you should see the RRP robot spawned
- After reading the parameters for cylinder blocks positions, cylinders are spawned
- Then the robot will move towards the intended coordinate and will pick & place to goal positions one by one

## Files
- [`scara_robot_right.urdf`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Final_Project/URDF_Files/scara_robot_right.urdf) This is the description of the RRP robot using urdf format.
- [`function_node.cpp`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Final_Project/src/function/src/function_node.cpp) C++ code for moving the robot and getting the coordinates and jacobian matrix using Moveit API.

## Creation Process
Here are the steps we've taken to create this project:
1. Written the urdf file
2. Created package rrp_robot of RRP robot using moveit setup assistant(and defined move groups):
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
3. Created node 'function_node' to spawn the cylinders and move the robot using Moveit:
- Edited cmakelists.txt to include necessary packages and made the code executable 

