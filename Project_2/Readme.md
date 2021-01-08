# Project 2: Robot Kinematics and Inverse Kinematics Using Moveit Package

## Introduction

This project is focused on using a previously created PR robot to calculate forwards and inverse kinematics using the ROS Moveit interface.

<img src="https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Images/pr_robot_velocity.gif" width="600">

## Dependecies
- ROS
- Moveit

## Running the Code
- First, open the Project_3 folder and build the catkin workspace:
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
- First, you should see the Jacobian matrix for the random position.
- Then you will see the jacobian matrix, velocity, angular velocity, differential position, and differential angle for every time step using the following formulas for joint velocities:
<img src="https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Images/joint_velocities.png" width="800">

- Then the robot will move towards intended coordinates differentially

Note: I have implemented two methods to move the robot. First, using the differential coordinates to update the intended coordinates and moving towards them. Second, using the updated joint values to update the state of the robot. The first solution gives errors sometimes that is why I have commented it out in the code.

## Files
- [`pr.urdf`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Project_3/src/urdf%20files/pr.urdf) This is the description of the robot using urdf format.
- [`robot_model.cpp`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Project_3/src/manipulator_core/src/robot_model.cpp) C++ code for moving the robot and getting the coordinates and jacobian matrix using Moveit API.
- [`project3_report.pdf`](https://github.com/YasinSonmez/EE451--Introduction-to-Robotics/blob/master/Project_3/project3_report.pdf) Project report answering the given questions and explaining the output of the program.

## Creation Process
Here are the steps I've taken to create this project:
1. Written the urdf file
2. Created config package pr_config of PR robot using moveit setup assistant(and defined move groups):
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
3. Created package 'manipulator core' to move the robot and calculate the jacobian matrices using Moveit
   - Edited cmakelists.txt to include necessary packages and made the code executable 
