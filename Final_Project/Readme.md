# Final Project B: Warehouse Robotic Tasks

## Introduction

This project is focused on
- Creating an RRP robot
- Spawning objects with different sizes on different coordinates
- Picking & Placing using the ROS Moveit interface

<p float="left">
  <img src="https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Images/chess2.gif" width="450" />
  <img src="https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Images/arrange_2.gif" width="470" /> 
</p>

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
To see the demo of arranging objects:
roslaunch function read_params.launch

To see the scholar's mate demo:
roslaunch function read_params_2.launch
```
- Then launch the robot config file and open the RViz:
```
roslaunch rrp_robot_2 demo.launch
```
- Then run the pick & place code that also spawns the cylinders:
```
To see the demo of arranging objects:
rosrun function function_node

To see the scholar's mate demo:
rosrun chess chess_node
```
- First, you should see the RRP robot spawned
- After reading the parameters for cylinder blocks positions, cylinders are spawned
- Then the robot will move towards the intended coordinate and will pick & place to goal positions one by one

## Files
- [`params.yaml`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Final_Project/src/function/config/params.yaml) Given objects' parameter file in yaml format (height, radius, position, goal position).
- [`params_2.yaml`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Final_Project/src/function/config/params_2.yaml) Given objects' parameter file in yaml format for chess configuration (height, radius, position, goal position).
- [`scara_robot_right.urdf`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Final_Project/URDF_Files/scara_robot_right.urdf) This is the description of the RRP robot using urdf format.
- [`function_node.cpp`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Final_Project/src/function/src/function_node.cpp) C++ code to spawn objects and pick & place operation of the robot using MoveIt!.
- [`chess.cpp`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Final_Project/src/chess/src/chess.cpp) C++ code to spawn objects and pick & place operation of the robot using MoveIt! for the chess configuration.

## Creation Process
Here are the steps we've taken to create this project:
1. Written the urdf file
2. Created package 'rrp_robot_2' of RRP robot using moveit setup assistant(and defined move groups):
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
3. Created yaml file to spesify object positions.
4. Created node 'function_node' to spawn the cylinders and move the robot using Moveit:
- Edited cmakelists.txt to include necessary packages and made the code executable 
5. Created node 'chess_node' to spawn the cylinders in chess configuration and move the robot using Moveit to do a scholar's mate:
- Edited cmakelists.txt to include necessary packages and made the code executable 

 ## About The Given Object Informations (Yaml File Format)
 - In order to spawn object in the simulation world, we needed to read parameter from a yaml file.
 - Due to our robot position "0, 0, 0", we choose our 1mx1m square workspace in the range of "0.2 < x < 1.2", "0.2 < y < 1.2".
 - Our yaml file has the following structure and our C++ code depends on this structure.
 
```
N: 2      # Number of objects
p1x: 0.7  # First object initial x position
p1y: 0.8  # First object initial y position
p1: 0.05  # First object radius
h1: 0.12  # First object height
g1x: 0.7  # First object goal x position
g1y: 0.5  # First object goal y position

p2x: 0.4  # Second object initial x position
.
.
.
```

## References
1- https://github.com/yangliu28/two_scara_collaboration
