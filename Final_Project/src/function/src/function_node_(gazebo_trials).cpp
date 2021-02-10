#include <ros/ros.h>
#include <string.h>
#include <stdlib.h>

#include<iostream>
#include<fstream>
#include <tf/transform_datatypes.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "gazebo_cube_spawner.h"

using namespace std;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}


int main(int argc, char **argv){

	ros::init(argc, argv, "funtion_node");

	ros::AsyncSpinner spinner(1);
    spinner.start();

//Getting Given Parameters from rosparam
	ros::NodeHandle nh2;
	int N;
	nh2.getParam("/N", N);
	double object[N][6];
	string param;
	string num,x,y;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.resize(N);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  	moveit::planning_interface::MoveGroupInterface group("base_group");

	for(int i=0; i<N; i++)
	{
		num = to_string(i+1);
		param = "/p" + num + "x";
		nh2.getParam(param, object[i][0]);
		param = "/p" + num + "y";
		nh2.getParam(param, object[i][1]);
		param = "/p" + num;
		nh2.getParam(param, object[i][2]);
		param = "/h" + num;
		nh2.getParam(param, object[i][3]);
		param = "/g" + num + "x";
		nh2.getParam(param, object[i][4]);
		param = "/g" + num + "y";
		nh2.getParam(param, object[i][5]);

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
		collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
		collision_objects[i].primitives[0].dimensions.resize(3);
		collision_objects[i].primitives[0].dimensions[0] = object[i][2]*2;
		collision_objects[i].primitives[0].dimensions[1] = object[i][2]*2;
		collision_objects[i].primitives[0].dimensions[2] = object[i][3];

		/* Define the pose of the table. */
		collision_objects[i].primitive_poses.resize(1);
		collision_objects[i].primitive_poses[0].position.x = object[i][0];
		collision_objects[i].primitive_poses[0].position.y = object[i][1];
		collision_objects[i].primitive_poses[0].position.z = 0;

		collision_objects[i].operation = collision_objects[i].ADD;
	}

//	planning_scene_interface.applyCollisionObjects(collision_objects);

	addCollisionObjects(planning_scene_interface);

//-------------------------------------------


// Spawning obstacles


//-------------------------------------------

// Moveit Part




//----------------------------------------
	ros::spinOnce();

	return 0;
}
