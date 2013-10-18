/* 
 * File:   test.cpp
 * Author: Isaac Yochelson
 *
 * Created on August 29, 2013, 10:08 AM
 */
#include <moveit/move_group_interface/move_group.h>
#include <cstdlib>			//standard library for C/C++
#include <iostream>
#include <fstream>

using namespace std;

void fillMap(map<string, double> &goal, string filename);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Test_Node");
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	sleep(15);
	// this connecs to a running instance of the move_group node
	move_group_interface::MoveGroup group("left_arm");
/*	group.setStartStateToCurrentState();
	// specify our target
	vector<double> goal;
	double temp[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	goal.assign(temp, temp+7);
	group.setJointValueTarget(goal);
	// plan the motion and then move the group to the sampled target 
	group.move();
	group.setStartStateToCurrentState();
	double temp1[] = {-0.7, 0.0, -0.7, 0.0, 0.0, 0.0, 0.0};
	goal.assign(temp1, temp1+7);
	group.setJointValueTarget(goal);
	group.move();
	group.setStartStateToCurrentState();
	double temp2[] = {0.7, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0};
	goal.assign(temp2, temp2+7);
	group.setJointValueTarget(goal);
	group.move();*/
	group.setStartStateToCurrentState();
	map<string, double> goal2;
	fillMap(goal2, "test.txt");
	group.setJointValueTarget(goal2);
//	group.move();
	moveit::planning_interface::MoveGroup::Plan plan;
	group.plan(plan);
	group.execute(plan);
	ros::waitForShutdown();
}
void fillMap(map<string, double> &goal, string filename)
{
	ifstream goalInput;
	cout << filename.c_str() << endl;
	goalInput.open(filename.c_str());
	string line;
	while(getline(goalInput, line)) {
//		ROS_INFO("outer loop:: \n");
//		ROS_INFO(line.c_str());
		stringstream lineStream(line);
		string key;
		string valueString;
		double value;
		while (lineStream >> key) {
			lineStream >> valueString;
			value = atof(valueString.c_str());
			goal[key] = value;
		}
	}
	goalInput.close();
}
