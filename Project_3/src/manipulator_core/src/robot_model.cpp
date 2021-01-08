#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_model_and_robot_state");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Setting up to start using the RobotModel class is very easy. In
	// general, you will find that most higher-level components will
	// return a shared pointer to the RobotModel. You should always use
	// that when possible. In this example, we will start with such a
	// shared pointer and discuss only the basic API. You can have a
	// look at the actual code API for these classes to get more
	// information about how to use more features provided by these
	// classes.
	//
	// We will start by instantiating a
	// `RobotModelLoader`_
	// object, which will look up
	// the robot description on the ROS parameter server and construct a
	// :moveit_core:`RobotModel` for us to use.
	//
	// .. _RobotModelLoader:
	//     http://docs.ros.org/melodic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html

	// Assumes that a robot has been loaded
	// One approach is to use a launch file

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	// Using the :moveit_core:`RobotModel`, we can construct a
	// :moveit_core:`RobotState` that maintains the configuration
	// of the robot. We will set all joints in the state to their
	// default values. We can then get a
	// :moveit_core:`JointModelGroup`, which represents the robot
	// model for a particular group, e.g. the "pr_arm" of the robot
	// robot.

	// Set the robot kinematic state
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

	kinematic_state->setToDefaultValues();

	//MoveIt operates on sets of joints called “planning groups” and stores them in an object called the JointModelGroup.
	//Throughout MoveIt the terms “planning group” and “joint model group” are used interchangably.
	const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("pr_move");

	const std::vector<std::string > &joint_names = joint_model_group->getVariableNames();

	// Get Joint Values
	// ^^^^^^^^^^^^^^^^
	// We can retreive the current set of joint values stored in the state for the PR arm.
	std::vector<double> joint_values;

	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

	static
	const std::string PLANNING_GROUP = "pr_move";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	// Planning using velocity kinematics
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Let's set a joint space goal, find the velocities, displacements and move towards it. 
	//
	// To start, we'll create an pointer that references the current robot's state.
	// RobotState is the object that contains all the current position/velocity/acceleration data.
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	//
	// Next get the current set of joint values for the group.
	std::vector<double> joint_group_positions;

	// Define timesteps and t
	double delta_t = 0.01;
	double t = 0;

	// Define the q_dot, d1_dot, theta2_dot, v, w, delta_o, delta_theta
	Eigen::Vector2d q_dot;
	Eigen::Vector3d d1_dot, theta2_dot, v, w, delta_o, delta_theta;
	Eigen::VectorXd vw;

	// Define quaternions for rotation
	tf2::Quaternion q_orig, q_rot, q_new;
	q_orig.setRPY(0, 0, 0);

	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// Move to starting state
	joint_group_positions[0] = 0.0;
	joint_group_positions[1] = 0.0;

	move_group.setJointValueTarget(joint_group_positions);
	move_group.move();

	// Modify the joints velocity step by step for k=600 timesteps and dt=1/100s 
	for (int k = 1; k < 600; k++)
	{
		t = k *0.01;	// t in seconds
		if (k <= 100)
		{
			q_dot[0] = -0.01 * t;
			q_dot[1] = -0.1 * t;	// radians
		}
		else if (k <= 500)
		{
			q_dot[0] = -0.1;
			q_dot[1] = -1;	// radians
		}
		else
		{
			q_dot[0] = -(0.1 - 0.01 *(t - 5));
			q_dot[1] = -(1 - 0.1 *(t - 5));	// radians
		}

		// Get the Jacobian
		// ^^^^^^^^^^^^^^^^
		// We can get the Jacobian from the :moveit_core:`RobotState`.
		Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
		Eigen::MatrixXd jacobian;

		kinematic_state->getJacobian(joint_model_group,
			kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
			reference_point_position, jacobian);
		std::cout << "Jacobian: \n" << jacobian << "\n";

		// Calculate v and w and print them
		vw = jacobian * q_dot;
		v = vw.head(3);
		w = vw.tail(3);
		std::cout << "v = \n" << v << std::endl;
		std::cout << "w = \n" << w << std::endl;

		// Calculate displacements and print them
		delta_o = v * delta_t;
		delta_theta = w * delta_t;
		std::cout << "delta_o = \n" << delta_o << std::endl;
		std::cout << "delta_theta = \n" << delta_theta << std::endl;

		// We can move the robot to desired coordinates using either joint values or coordinate displacements
		// 1- Using coordinate displacements (Problematic for now):
		/*
		geometry_msgs::Pose target_pose;	// Define message

		// Find end effector coordinates
		const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link4");
		const Eigen::Vector4d origin(0,0,0,1); 

		// Change the coordinates by using displacement
		const Eigen::Vector3d end_effector_coordinates = (end_effector_state.matrix()*origin).head(3) + delta_o; 

		// Stuff the new corrdinates to message
		target_pose.position.x = end_effector_coordinates[0];
		target_pose.position.y = end_effector_coordinates[1];
		target_pose.position.z = end_effector_coordinates[2];

		// Get the orientation in Euler angles
		q_rot.setRPY(delta_theta[0], delta_theta[1], delta_theta[2]);
		q_new = q_rot*q_orig; 	// Calculate the new orientation
		q_new.normalize();

		// Stuff the new rotation back into the target pose. This requires conversion into a msg type
		tf2::convert(q_new, target_pose.orientation);

		move_group.setPoseTarget(target_pose);	// set the target position
		move_group.move();
		*/

		// 2- Move using joint values displacements
		joint_group_positions[0] += q_dot[0] *delta_t;
		joint_group_positions[1] += q_dot[1] *delta_t;

		kinematic_state->setJointGroupPositions(joint_model_group, joint_group_positions);

		move_group.setJointValueTarget(joint_group_positions);
		move_group.move();
		std::cout << "k = " << k << std::endl << std::endl;	// Print the step number
	}
	ros::shutdown();
	return 0;
}