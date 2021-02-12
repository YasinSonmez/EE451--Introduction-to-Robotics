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



int main(int argc, char** argv)
{
// initializing ros node
  ros::init(argc, argv, "function_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

//initializing moveit interfaces, planning_scene_interface and move_group_interface
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("base_group");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.setPlanningTime(60.0);




  int N;	// number of objects to be transported
  nh.getParam("/N", N);	// getting the number of objects from the rosparam server
  double object[N][6];	// array where holds the information about object locations and dimensions

  std::vector<moveit_msgs::CollisionObject> collision_objects;	// creating a collision object object
  collision_objects.resize(N);

	for(int i=0; i<N; i++)	// loop where we get information about objects from rosparam server
	{
		string param;
		string num,x,y;
		num = to_string(i+1);

		// getting information from rosparam server
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

		//x = to_string(object[i][0]);
		//y = to_string(object[i][1]);
		//param = "rosrun gazebo_ros spawn_model -file ~/final_project_ws/URDF_Files/cylinder.urdf -urdf -x "
		// + x + " -y " + y + " -z 0 -model cylinder" + num;

		//const char *cstr = param.c_str();
		//system(cstr);

		/* spawning objects in rviz as collision objects */
		collision_objects[i].id = "cylinder" + num;
		collision_objects[i].header.frame_id = "base_link";

		/* Define the primitive and its dimensions. */
		collision_objects[i].primitives.resize(1);
		collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].CYLINDER;
		collision_objects[i].primitives[0].dimensions.resize(3);
		collision_objects[i].primitives[0].dimensions[0] = object[i][3];
		collision_objects[i].primitives[0].dimensions[1] = object[i][2];

		/* Define the pose of the cylinder. */
		collision_objects[i].primitive_poses.resize(1);
		collision_objects[i].primitive_poses[0].position.x = object[i][0];
		collision_objects[i].primitive_poses[0].position.y = object[i][1];
		collision_objects[i].primitive_poses[0].position.z = object[i][3] / 2;

		collision_objects[i].operation = collision_objects[i].ADD;
	}

  planning_scene_interface.applyCollisionObjects(collision_objects); // apply collision objects to the scene

//---------------------------------------------------------------------


  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  geometry_msgs::Pose target_pose1;	// create an object for position targets
  group.setGoalTolerance(0.02);

  for(int i=0; i<N; i++)	//loop for pick and place operation
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

  ROS_INFO_STREAM(i+1 << ". OBJECT TRANSPORTED");

  }

// go to the some position
  target_pose1.position.x = -0.5;
  target_pose1.position.y = -0.5;
  group.setJointValueTarget(target_pose1);
  group.move();
  ROS_INFO("FINAL POSITION REACHED");

  ROS_INFO_STREAM("TOTAL NUMBER OF OBJECTS TRANSPORTED: " << N);

  ros::shutdown();
  return 0;
}
