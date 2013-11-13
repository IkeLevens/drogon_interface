/* 
 * File:   iktest.cpp
 * Author: Isaac Yochelson
 *
 * Created on August 29, 2013, 10:08 AM
 */
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <cstdlib>			//standard library for C/C++
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>

using namespace std;

void moveToPose(string joint, move_group_interface::MoveGroup& group, geometry_msgs::Pose* pose);
void pick (int cup, move_group_interface::MoveGroup& group, string joint);
void place (int cup, move_group_interface::MoveGroup& group, string joint);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Test_Node");
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	// this connecs to a running instance of the move_group node
	move_group_interface::MoveGroup leftGroup("left_arm");
	move_group_interface::MoveGroup rightGroup("right_arm");
	leftGroup.setPlannerId("PRMstarkConfigDefault");
	leftGroup.setStartStateToCurrentState();
	rightGroup.setPlannerId("PRMstarkConfigDefault");
	rightGroup.setStartStateToCurrentState();
	ROS_INFO("setting target");
	pick(5, leftGroup, "left_wrist");
	place(0, leftGroup, "left_wrist");
	pick(-3, rightGroup, "right_wrist");
	place(0, rightGroup, "right_wrist");
	pick(2, leftGroup, "left_wrist");
	place(0, leftGroup, "left_wrist");
	pick(0, rightGroup, "right_wrist");
	place(-3, rightGroup, "right_wrist");
	return 0;
}
void pick (int cup, move_group_interface::MoveGroup& group, string joint) {
	string arm;
	if (joint == "left_wrist") {
		arm = "left";
	} else {
		arm = "right";
	}
	geometry_msgs::Pose* pose = new geometry_msgs::Pose;
	pose->position.x = 0.82;
	pose->position.y = 0.104 * cup;
	pose->position.z = 0.35;
	pose->orientation.x = 0;
	pose->orientation.y = 1;
	pose->orientation.z = 0;
	pose->orientation.w = 0;
	moveToPose(joint, group, pose);
	pose->position.z = 0.14;
	moveToPose(joint, group, pose);
	stringstream ss;
	ss << "python /home/pracsys/workspaces/hydro_ws/src/drogon_interface/src/gripper.py close " << arm;
	system(ss.str().c_str());
	pose->position.z = 0.35;
	moveToPose(joint, group, pose);
}
void place (int cup, move_group_interface::MoveGroup& group, string joint) {
	string arm;
	if (joint == "left_wrist") {
		arm = "left";
	} else {
		arm = "right";
	}
	geometry_msgs::Pose* pose = new geometry_msgs::Pose;
	pose->position.x = 0.82;
	pose->position.y = 0.104 * cup;
	pose->position.z = 0.35;
	pose->orientation.x = 0;
	pose->orientation.y = 1;
	pose->orientation.z = 0;
	pose->orientation.w = 0;
	moveToPose(joint, group, pose);
	pose ->position.z = 0.19;
	moveToPose(joint, group, pose);
	stringstream ss;
	ss << "python /home/pracsys/workspaces/hydro_ws/src/drogon_interface/src/gripper.py open " << arm;
	system(ss.str().c_str());
	pose->position.z = 0.35;
	moveToPose(joint, group, pose);
	if (arm == "left") {
		pose->position.y = 0.25;
	} else {
		pose->position.y = -0.25;
	}
	moveToPose(joint, group, pose);
}
void moveToPose(string joint, move_group_interface::MoveGroup& group, geometry_msgs::Pose* pose) {
	bool gotPlan;
	moveit::planning_interface::MoveGroup::Plan plan;
	group.setPoseTarget(*pose, joint);
	ROS_INFO("requesting plan.");
	gotPlan = group.plan(plan);
	if (gotPlan) {
		ROS_INFO("plan received.");
		cout << plan.trajectory_ << endl;
		group.execute(plan);
		ROS_INFO("plan execution completed.");
	}
}
