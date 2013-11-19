/* 
 * File:   hardcodetest.cpp
 * Author: Isaac Yochelson
 *
 * Created on August 11, 2013, 2:00 PM
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
void savePlan(string filename, moveit::planning_interface::MoveGroup::Plan* plan);
void reset();
int ct;

int main(int argc, char** argv)
{
	ct = 12;
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
	geometry_msgs::Pose center;
	center.position.x = .75;
	center.position.y = -.05;
	center.position.z = .1;
	center.orientation.x = 0;
	center.orientation.y = .8;
	center.orientation.z = 0;
	center.orientation.w = .6;
	geometry_msgs::Pose one(center);
	one.position.x = .776;
	one.position.y = -0.08;
	one.position.z = 0.145;
	geometry_msgs::Pose two(center);
	two.position.x = .765;
	two.position.y = -0.102;
	two.position.z = 0.145;
	geometry_msgs::Pose three(center);
	three.position.x = .75;
	three.position.y = -0.11;
	three.position.z = 0.1;
	geometry_msgs::Pose four(center);
	four.position.x = .7350;
	four.position.y = -.102;
	four.position.z = 0.055;
	geometry_msgs::Pose five(center);
	five.position.x = .724;
	five.position.y = -0.08;
	five.position.z = 0.055;
	geometry_msgs::Pose six(center);
	six.position.x = .72;
	six.position.y = -0.05;
	six.position.z = 0.048;
	geometry_msgs::Pose eleven(center);
	eleven.position.x = .776;
	eleven.position.y = -0.02;
	eleven.position.z = 0.145;
	geometry_msgs::Pose ten(center);
	ten.position.x = .765;
	ten.position.y = 0.002;
	ten.position.z = 0.145;
	geometry_msgs::Pose nine(center);
	nine.position.x = .75;
	nine.position.y = 0.01;
	nine.position.z = 0.1;
	geometry_msgs::Pose eight(center);
	eight.position.x = .7350;
	eight.position.y = .002;
	eight.position.z = 0.055;
	geometry_msgs::Pose seven(center);
	seven.position.x = .724;
	seven.position.y = -0.02;
	seven.position.z = 0.055;
	geometry_msgs::Pose twelve(center);
	twelve.position.x = .78;
	twelve.position.y = -0.05;
	twelve.position.z = 0.152;
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &one);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &two);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &three);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &four);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &five);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &six);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &seven);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &eight);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &nine);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &ten);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &eleven);
	reset();
	moveToPose("right_wrist", rightGroup, &center);
	moveToPose("right_wrist", rightGroup, &twelve);
	reset();
	return 0;
}
void reset()
{
	system ("rosrun joint_trajectory file_playback.py -f /home/pracsys/trajectories/vilmi/down.trj");
}
void moveToPose(string joint, move_group_interface::MoveGroup& group, geometry_msgs::Pose* pose)
{
	bool gotPlan;
	moveit::planning_interface::MoveGroup::Plan plan;
	group.setPoseTarget(*pose, joint);
	ROS_INFO("requesting plan.");
	gotPlan = group.plan(plan);
	if (gotPlan) {
		ROS_INFO("plan received.");
		cout << plan.trajectory_ << endl;
		stringstream ss;
		ss << "/home/pracsys/trajectories/vilmi/plan" << ++ct << ".trj";
		savePlan(ss.str(), &plan);
		group.execute(plan);
		ROS_INFO("plan execution completed.");
	}
}
void savePlan(string filename, moveit::planning_interface::MoveGroup::Plan* plan)
{
	ofstream planOutput;
	planOutput.open(filename.c_str());
	planOutput << "time";
	for(vector<string>::iterator it =  plan->trajectory_.joint_trajectory.joint_names.begin();
			it !=  plan->trajectory_.joint_trajectory.joint_names.end(); it++) {
		planOutput << "," << *it;
	}
	planOutput << "\n";
	for(vector<trajectory_msgs::JointTrajectoryPoint>::iterator pt =  plan->trajectory_.joint_trajectory.points.begin();
			pt !=  plan->trajectory_.joint_trajectory.points.end(); pt++) {
		double secs = (*pt).time_from_start.toSec();
		stringstream pointStream;
		pointStream << secs;
		for(vector<double>::iterator dt = (*pt).positions.begin(); dt != (*pt).positions.end(); dt++) {
			pointStream << "," << *dt;
		}
		planOutput << pointStream.str() << "\n";
	}
	planOutput.close();
}
