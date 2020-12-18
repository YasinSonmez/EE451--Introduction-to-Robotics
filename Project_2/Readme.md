# Project 2: Robot Kinematics and Inverse Kinematics Using Moveit Package

## Introduction

This project is focused on using previously created PR robot to calculate forwards and inverse kinematics using the ROS Moveit interface.

<img src="https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Images/pr_transform.png" width="600">

## Dependecies
- ROS
- Moveit

## Running the Code
- First open the Project_2 folder and build the catkin workspace:
```
catkin build
```
- Then launch the robot config file and open the RViz:
```
roslaunch pr_config demo.launch
```
- Then run the manipulator core code using the created node:
```
rosrun manipulator_core robot_model_node 
```
- First you should see the robot joint values.
- Then you will see the intended joint values and corresponding transformation matrices and end effectors coordinates.
- Then the result of inverse kinematics which should be same as intended joint values.
- Finally you should see the robot plan and move through it's workspace.

## Files
- [`pr.urdf`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Project_2/src/urdf%20files/pr.urdf) This is the description of robot using urdf format.
- [`robot_model.cpp`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Project_2/src/manipulator_core/src/robot_model.cpp) C++ code for moving the robot and getting the cordinates and transformation matrix using Moveit API.
- [`project2_report.pdf`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Project_2/project2_report.pdf) Project report answering the given questions and explaining the output of the program.

## Creation Process
Here are the steps I've taken to create this project:
1. Written the urdf file
2. Created config package pr_config of PR robot using moveit setup assistant(and defined move groups):
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
3. Created package manipulator core to move the robot and calculate the transformation matrices using Moveit
   - Edited cmakelists.txt to include necessary packages and made the code executable 
