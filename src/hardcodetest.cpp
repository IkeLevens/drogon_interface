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

void fillMap(map<string, double> &goal, string filename);
void savePlan(string filename, moveit::planning_interface::MoveGroup::Plan* plan);
string input = "test.txt";
string output = "testingPlanSaver.trj";

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Test_Node");
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	// this connecs to a running instance of the move_group node
	move_group_interface::MoveGroup group("left_arm");
	group.setPlannerId("PRMstarkConfigDefault");
	group.setStartStateToCurrentState();
	ROS_INFO("setting target");
	geometry_msgs::Pose pose;
	pose.position.x = 0.82;
	pose.position.y = 0.52;
	pose.position.z = 0.35;
	pose.orientation.x = 0;
	pose.orientation.y = 1;
	pose.orientation.z = 0;
	pose.orientation.w = 0;
	group.setPoseTarget(pose, "left_wrist");
	moveit::planning_interface::MoveGroup::Plan plan;
	ROS_INFO("requesting plan.");
	bool gotPlan = group.plan(plan);
	if (gotPlan) {
		ROS_INFO("plan received.");
		group.execute(plan);
		ROS_INFO("plan execution completed.");
	}
	system("python /home/pracsys/workspaces/hydro_ws/src/drogon_interface/src/gripper.py open left");
	pose.position.x = 0.82;
	pose.position.y = 0.52;
	pose.position.z = 0.14;
	group.setPoseTarget(pose, "left_wrist");
	ROS_INFO("requesting plan.");
	gotPlan = group.plan(plan);
	if (gotPlan) {
		ROS_INFO("plan received.");
		cout << plan.trajectory_ << endl;
		group.execute(plan);
		ROS_INFO("plan execution completed.");
	}
	system("python /home/pracsys/workspaces/hydro_ws/src/drogon_interface/src/gripper.py close left");
	pose.position.x = 0.82;
	pose.position.y = 0.52;
	pose.position.z = 0.35;
	group.setPoseTarget(pose, "left_wrist");
	ROS_INFO("requesting plan.");
	gotPlan = group.plan(plan);
	if (gotPlan) {
		ROS_INFO("plan received.");
		group.execute(plan);
		ROS_INFO("plan execution completed.");
	}
	pose.position.x = 0.82;
	pose.position.y = 0;
	pose.position.z = 0.35;
	group.setPoseTarget(pose, "left_wrist");
	ROS_INFO("requesting plan.");
	gotPlan = group.plan(plan);
	if (gotPlan) {
		ROS_INFO("plan received.");
		group.execute(plan);
		ROS_INFO("plan execution completed.");
	}
	pose.position.x = 0.82;
	pose.position.y = 0;
	pose.position.z = 0.19;
	group.setPoseTarget(pose, "left_wrist");
	ROS_INFO("requesting plan.");
	gotPlan = group.plan(plan);
	if (gotPlan) {
		ROS_INFO("plan received.");
		group.execute(plan);
		ROS_INFO("plan execution completed.");
	}
	system("python /home/pracsys/workspaces/hydro_ws/src/drogon_interface/src/gripper.py open left");
	return 0;
}
