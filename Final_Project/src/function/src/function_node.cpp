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
  //group.setGoalTolerance(0.1);




  for(int i=0; i<N; i++)	//loop for pick and place operation
  {

  target_pose1.position.x = 0.5;
  target_pose1.position.y = 0.5;
  target_pose1.position.z = 0.68;
  group.setApproximateJointValueTarget(target_pose1, "gripper_link");
  group.plan(my_plan);
  group.execute(my_plan);
  group.move();

  target_pose1.position.x = object[i][0];
  target_pose1.position.y = object[i][1];
  target_pose1.position.z = 0.68;
  group.setApproximateJointValueTarget(target_pose1, "gripper_link");
  group.move();
  ROS_INFO("OBJECT REACHED");

  target_pose1.position.z = object[i][3] + 0.21;
  group.setApproximateJointValueTarget(target_pose1, "gripper_link");
  group.move();

  group.attachObject(collision_objects[i].id);
  ROS_INFO("OBJECT ATTACHED");

  target_pose1.position.z = 0.68;
  group.setApproximateJointValueTarget(target_pose1, "gripper_link");
  group.move();

  target_pose1.position.x = object[i][4];
  target_pose1.position.y = object[i][5];
  target_pose1.position.z = 0.68;
  group.setApproximateJointValueTarget(target_pose1, "gripper_link");
  group.move();
  ROS_INFO("GOAL REACHED");

  target_pose1.position.z = object[i][3] + 0.21;
  group.setApproximateJointValueTarget(target_pose1, "gripper_link");
  group.move();

  group.detachObject(collision_objects[i].id);
  ROS_INFO("OBJECT DETACHED");

  }

  target_pose1.position.x = 0.5;
  target_pose1.position.y = 0.5;
  target_pose1.position.z = 0.68;
  group.setApproximateJointValueTarget(target_pose1);
  group.move();
  ROS_INFO("FINAL POSITION REACHED");

  ros::waitForShutdown();
  return 0;
}
