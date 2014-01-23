/* 
 * File:   lines.cpp
 * Author: Isaac Yochelson
 *
 * Created on August 14, 2013, 3:54 PM
 */
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <baxter_core_msgs/EndpointState.h>
#include <cstdlib>			//standard library for C/C++
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>

using namespace std;

string filename = "cup1.pose";
string output = "cup1.trj";

void moveToPose(string joint, move_group_interface::MoveGroup& group, geometry_msgs::Pose* pose);
void pick (int cup, move_group_interface::MoveGroup& group, string joint);
void place (int cup, move_group_interface::MoveGroup& group, string joint);

void fillMap(map<string, double> &goal, string filename);
void savePlan(string filename, moveit::planning_interface::MoveGroup::Plan* plan);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Test_Node");
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;

	map<string, double> targetMap;
	fillMap(targetMap, filename);
	geometry_msgs::Pose target;
	target.position.x = targetMap["x"];
	target.position.y = targetMap["y"];
	target.position.z = targetMap["z"];
	// this connecs to a running instance of the move_group node
	move_group_interface::MoveGroup leftGroup("left_arm");
	move_group_interface::MoveGroup rightGroup("right_arm");
	leftGroup.setPlannerId("PRMstarkConfigDefault");
	leftGroup.setStartStateToCurrentState();
	rightGroup.setPlannerId("PRMstarkConfigDefault");
	rightGroup.setStartStateToCurrentState();
	move_group_interface::MoveGroup* group;
	string joint;
	if (targetMap["y"] < 0) {
		group = &rightGroup;
		joint = "right_wrist";
	} else {
		group = &leftGroup;
		joint = "left_wrist";
	}
	geometry_msgs::Pose startPose;
	cout << "current pose:\n" << group->getCurrentPose(joint).pose;
	startPose = group->getCurrentPose(joint).pose;
	cout << "startPose:\n" << startPose << endl;
	cout << "target:\n" << target.position << endl;
	int ctMax = 20;
	double dx = (target.position.x - startPose.position.x) / ctMax;
	double dy = (target.position.y - startPose.position.y) / ctMax;
	double dz = (target.position.z - startPose.position.z) / ctMax;
	startPose.orientation.x = 0;
	startPose.orientation.y = 1;	
	startPose.orientation.z = 0;
	startPose.orientation.w = 0;
//	double dox = (target.orientation.x - startPose.orientation.x) / ctMax;
//	double doy = (target.orientation.y - startPose.orientation.y) / ctMax;
//	double doz = (target.orientation.z - startPose.orientation.z) / ctMax;
//	double dow = (target.orientation.w - startPose.orientation.w) / ctMax;
	stringstream ss;
	ss	<< "dx: " << dx << " dy: " << dy << " dz: " << dz << endl;
	cout << ss.str() << endl;
	for (int ct = 0; ct < ctMax; ++ct) {
		startPose.position.x = startPose.position.x + dx;
		startPose.position.y = startPose.position.y + dy;
		startPose.position.z = startPose.position.z + dz;
//		startPose.orientation.x = startPose.orientation.x + dox;
//		startPose.orientation.y = startPose.orientation.y + doy;
//		startPose.orientation.z = startPose.orientation.z + doz;
//		startPose.orientation.w = startPose.orientation.w + dow;
		cout << startPose.position << endl;
		moveToPose(joint, *group, &startPose);
	}
	return 0;
}
void moveToPose(string joint, move_group_interface::MoveGroup& group, geometry_msgs::Pose* pose) {
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
}
void fillMap(map<string, double> &goal, string filename)
{
	ROS_INFO("filling map");
	ifstream goalInput;
	cout << filename.c_str() << endl;
	goalInput.open(filename.c_str());
	string keyLine;
	string valueLine;
	getline(goalInput, keyLine);
	cout << keyLine << endl;
	getline(goalInput, valueLine);
	cout << valueLine << endl;
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> commaDelimited(",");
	tokenizer keys(keyLine, commaDelimited);
	tokenizer values(valueLine, commaDelimited);
	double value;
	tokenizer::iterator value_iter = values.begin();
	for (tokenizer::iterator key_iter = keys.begin();
		key_iter != keys.end() && value_iter != values.end(); ++key_iter)
	{
		if ((*key_iter) == "time" || (*key_iter) == "left_gripper" || (*key_iter) == "right_gripper") 
		{
			++value_iter;
			continue;
		}
		cout << *key_iter << endl;
		value = atof((*value_iter).c_str());
		goal[*key_iter] = value;
		++value_iter;
	}
	goalInput.close();
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
