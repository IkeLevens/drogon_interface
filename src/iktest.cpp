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

bool running = true;
bool closing = true;
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
	move_group_interface::MoveGroup group("right_arm");
	group.setPlannerId("PRMstarkConfigDefault");
	group.setStartStateToCurrentState();
	ROS_INFO("setting target");
	string line;
	double x, y, z;
	while(running) {
		cout << "X" << endl;
		cin >> line;
		if (line == "end\n") {
			running = false;
			break;
		}
		x = atof(line.c_str());
		cout << "Y" << endl;
		cin >> line;
		y = atof(line.c_str());
		cout << "Z" << endl;
		cin >> line;
		z = atof(line.c_str());
		geometry_msgs::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
		pose.orientation.x = .74329414;
		pose.orientation.y = 0;
		pose.orientation.z = 0;
		pose.orientation.w = 0.668964732;
		group.setPoseTarget(pose, "right_wrist");
//		group.setPositionTarget(x, y , z, "left_wrist");
		moveit::planning_interface::MoveGroup::Plan plan;
		ROS_INFO("requesting plan.");
		bool gotPlan = group.plan(plan);
		if (gotPlan) {
			ROS_INFO("plan received.");
//			cout << "saving plan as: " << output << endl;
//			savePlan(output, &plan);
			ROS_INFO("executing plan");
			group.execute(plan);
			ROS_INFO("plan execution completed.");
		}
	}
	ros::waitForShutdown();
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
	getline(goalInput, valueLine);
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
