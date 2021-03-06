/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>


#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;


void openGripper()
{

}

void closeGripper()
{

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "function_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("base_group");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.setPlanningTime(60.0);

  int N;
  nh.getParam("/N", N);
  double object[N][6];

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(N);

  // Add the first table where the cube will originally be kept.
	for(int i=0; i<N; i++)
	{
		string param;
		string num,x,y;
		num = to_string(i+1);

		param = "/p" + num + "x";
		nh.getParam(param, object[i][0]);
		param = "/p" + num + "y";
		nh.getParam(param, object[i][1]);
		param = "/p" + num;
		nh.getParam(param, object[i][2]);
		param = "/h" + num;
		nh.getParam(param, object[i][3]);
		param = "/g" + num + "x";
		nh.getParam(param, object[i][4]);
		param = "/g" + num + "y";
		nh.getParam(param, object[i][5]);

		x = to_string(object[i][0]);
		y = to_string(object[i][1]);
		//param = "rosrun gazebo_ros spawn_model -file ~/final_project_ws/URDF_Files/cylinder.urdf -urdf -x "
		// + x + " -y " + y + " -z 0 -model cylinder" + num;

		//const char *cstr = param.c_str();
		//system(cstr);

		collision_objects[i].id = "cylinder" + num;
		collision_objects[i].header.frame_id = "base_link";

		/* Define the primitive and its dimensions. */
		collision_objects[i].primitives.resize(1);
		collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].CYLINDER;
		collision_objects[i].primitives[0].dimensions.resize(3);
		collision_objects[i].primitives[0].dimensions[0] = object[i][3];
		collision_objects[i].primitives[0].dimensions[1] = object[i][2];

		/* Define the pose of the table. */
		collision_objects[i].primitive_poses.resize(1);
		collision_objects[i].primitive_poses[0].position.x = object[i][0];
		collision_objects[i].primitive_poses[0].position.y = object[i][1];
		collision_objects[i].primitive_poses[0].position.z = object[i][3] / 2;

		collision_objects[i].operation = collision_objects[i].ADD;
	}

  planning_scene_interface.applyCollisionObjects(collision_objects);

//---------------------------------------------------------------------


  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  // pick(group);

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  geometry_msgs::Pose target_pose1;
  group.setGoalTolerance(0.02);

  int reached[N] = {0};
  int sum_reached = 0;
  for(int i = 0;sum_reached<N;i++)	//loop for pick and place operation
  {
  bool intersection = false;
  for(int j = 0; j<N; j++)
  {
    bool a = (object[i][4] < collision_objects[j].primitive_poses[0].position.x + object[j][2] + object[i][2]) 
    && (object[i][4] > collision_objects[j].primitive_poses[0].position.x - object[j][2] - object[i][2]);
    bool b = (object[i][5] < collision_objects[j].primitive_poses[0].position.y + object[j][2] + object[i][2]) 
    && (object[i][5] > collision_objects[j].primitive_poses[0].position.y - object[j][2] - object[i][2]);
  if(a&&b) intersection=true;
  }

  cout<<intersection<<endl;
  if(reached[i] || intersection)
  {if(i==N-1) i=-1;
  continue;}
  else
  {
// go to the object position
  target_pose1.position.x = object[i][0];
  target_pose1.position.y = object[i][1];
  target_pose1.position.z = object[i][3] + 0.24;
  group.setJointValueTarget(target_pose1);
  group.move();
  ROS_INFO("OBJECT REACHED");

// attach to the object
  group.attachObject(collision_objects[i].id);
  ROS_INFO("OBJECT ATTACHED");

// go to the goal position
  target_pose1.position.x = object[i][4];
  target_pose1.position.y = object[i][5];
  group.setJointValueTarget(target_pose1);
  group.move();
  ROS_INFO("GOAL REACHED");

// detach the object
  group.detachObject(collision_objects[i].id);
  ROS_INFO("OBJECT DETACHED");

  ROS_INFO_STREAM(i << ". OBJECT TRANSPORTED");
  sum_reached++;
  reached[i] = 1;

  collision_objects[i].primitive_poses[0].position.x = object[i][4];
  collision_objects[i].primitive_poses[0].position.y = object[i][5];

  }
  cout<<i<<endl;
  cout<<sum_reached<<endl;


  if(i==N-1) i=-1;
  }

// go to the initial position
  target_pose1.position.x = 0.5;
  target_pose1.position.y = 0.5;
  target_pose1.position.z = 0.68;
  group.setJointValueTarget(target_pose1);
  group.move();
  ROS_INFO("FINAL POSITION REACHED");

  ROS_INFO_STREAM("TOTAL NUMBER OF OBJECTS TRANSPORTED: " << N);

  ros::waitForShutdown();
  return 0;
}
