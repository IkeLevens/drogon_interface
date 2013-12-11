/* 
 * File:   curves.cpp
 * Author: Isaac Yochelson
 *
 * Created on August 22, 2013, 10:30 AM
 */
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <baxter_msgs/EndpointState.h>
#include <cstdlib>			//standard library for C/C++
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>

using namespace std;

string filename = "trj_curve_left_15.txt";
string output = "trj_curve_left_15.trj";
ofstream planOutput;
double addTime = 0;
geometry_msgs::Pose target;

moveit::planning_interface::MoveGroup::Plan planToPose(string joint, move_group_interface::MoveGroup& group, geometry_msgs::Pose* pose);
void openPlan(string filename, moveit::planning_interface::MoveGroup::Plan plan);
void savePlan(moveit::planning_interface::MoveGroup::Plan plan);
void closePlan();
void fillMap(map<string, vector<double> > &goal, string filename);
void generateAndSavePlan(move_group_interface::MoveGroup* group, string joint);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Test_Node");
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	// Create MoveGroup instances for each arm, set them to PRM*, and create a pointer to be assigned
	// to the correct arm to be used for each target.
	move_group_interface::MoveGroup leftGroup("left_arm");
	move_group_interface::MoveGroup rightGroup("right_arm");
	leftGroup.setPlannerId("PRMstarkConfigDefault");
	leftGroup.setStartStateToCurrentState();
	rightGroup.setPlannerId("PRMstarkConfigDefault");
	rightGroup.setStartStateToCurrentState();
	move_group_interface::MoveGroup* group;

	target.orientation.x = 0;
	target.orientation.y = 1;
	target.orientation.z = 0;
	target.orientation.w = 0;
	group = &rightGroup;
	string joint = "right_wrist";
	for (int i = 3; i < 8; ++i) {
		addTime = 0;
		stringstream filestream;
		filestream << "trj_curve_right_" << i << ".txt";
		filename = filestream.str();
		stringstream outstream;
		outstream << "trj_curve_right_" << i << ".trj";
		output = outstream.str();
		system ("rosrun joint_trajectory file_playback.py -f clear.trj");
		generateAndSavePlan(group, joint);
	}
	for (int i = 3; i < 8; ++i) {
		addTime = 0;
		stringstream filestream;
		filestream << "trj_straight_right_" << i << ".txt";
		filename = filestream.str();
		stringstream outstream;
		outstream << "trj_straight_right_" << i << ".trj";
		output = outstream.str();
		system ("rosrun joint_trajectory file_playback.py -f clear.trj");
		generateAndSavePlan(group, joint);
	}
	/*
	group = &leftGroup;
	joint = "left_wrist";
	for (int i = 13; i > 8; --i) {
		addTime = 0;
		stringstream filestream;
		filestream << "trj_curve_left_" << i << ".txt";
		filename = filestream.str();
		stringstream outstream;
		outstream << "trj_curve_left_" << i << ".trj";
		output = outstream.str();
		system ("rosrun joint_trajectory file_playback.py -f clear.trj");
		generateAndSavePlan(group, joint);
	}
	for (int i = 13; i > 8; --i) {
		addTime = 0;
		stringstream filestream;
		filestream << "trj_straight_left_" << i << ".txt";
		filename = filestream.str();
		stringstream outstream;
		outstream << "trj_straight_left_" << i << ".trj";
		output = outstream.str();
		system ("rosrun joint_trajectory file_playback.py -f clear.trj");
		generateAndSavePlan(group, joint);
	}
	*/
	return 0;
}
void generateAndSavePlan(move_group_interface::MoveGroup* group, string joint)
{
	
	moveit::planning_interface::MoveGroup::Plan plan;
	map<string, vector<double> > targetMap;
	fillMap(targetMap, filename);
	vector<double>::iterator xiter = targetMap["x"].begin();	
	vector<double>::iterator yiter = targetMap["y"].begin();
	vector<double>::iterator ziter = targetMap["z"].begin();
	for (xiter= targetMap["x"].begin(); xiter != targetMap["x"].end(); ++xiter) {
		target.position.x = *xiter;
		target.position.y = *yiter;
		target.position.z = *ziter;
		++yiter;
		++ziter;
		plan = planToPose(joint, *group, &target);
		if (xiter == targetMap["x"].begin()) {
			openPlan(output, plan);
		}
		savePlan(plan);
	}
	closePlan();
}
moveit::planning_interface::MoveGroup::Plan planToPose(string joint, move_group_interface::MoveGroup& group, geometry_msgs::Pose* pose)
{
	bool gotPlan;
	moveit::planning_interface::MoveGroup::Plan plan;
	group.setPoseTarget(*pose, joint);
	ROS_INFO("requesting plan.");
	gotPlan = group.plan(plan);
	if (gotPlan) {
		ROS_INFO("plan received.");
//		cout << plan.trajectory_ << endl;
		group.execute(plan);
		ROS_INFO("plan execution completed.");
	}
	return plan;
}
void fillMap(map<string, vector<double> > &goals, string filename)
{
	ROS_INFO("filling map");
	ifstream goalInput;
	cout << filename.c_str() << endl;
	goalInput.open(filename.c_str());
	double value;
	string keyLine = "x,y,z";
	string valueLine;
	cout << keyLine << endl;
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> commaDelimited(", ");
	tokenizer keys(keyLine, commaDelimited);
	while (getline(goalInput, valueLine)) {
		tokenizer values(valueLine, commaDelimited);
		tokenizer::iterator value_iter = values.begin();
		for (tokenizer::iterator key_iter = keys.begin();
		key_iter != keys.end() && value_iter != values.end(); ++key_iter)
		{
			value = atof((*value_iter).c_str());
			goals[*key_iter].push_back(value);
			++value_iter;
		}
	}
	goalInput.close();
}
void openPlan(string filename, moveit::planning_interface::MoveGroup::Plan plan)
{
	planOutput.open(filename.c_str());
	planOutput << "time";
	for(vector<string>::iterator it =  plan.trajectory_.joint_trajectory.joint_names.begin();
			it !=  plan.trajectory_.joint_trajectory.joint_names.end(); it++) {
		planOutput << "," << *it;
	}
	planOutput << "\n";
}
void closePlan()
{
	planOutput.close();
}
void savePlan(moveit::planning_interface::MoveGroup::Plan plan)
{
	double secs = 0;
	for(vector<trajectory_msgs::JointTrajectoryPoint>::iterator pt =  plan.trajectory_.joint_trajectory.points.begin();
			pt !=  plan.trajectory_.joint_trajectory.points.end(); pt++) {
		secs = (*pt).time_from_start.toSec() / 4;
		stringstream pointStream;
		pointStream << secs + addTime;
		for(vector<double>::iterator dt = (*pt).positions.begin(); dt != (*pt).positions.end(); dt++) {
			pointStream << "," << *dt;
		}
		planOutput << pointStream.str() << "\n";
	}
	addTime += secs;
}
