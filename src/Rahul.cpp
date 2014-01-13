/* 
 * File:   Rahul.cpp
 * Author: Isaac Yochelson
 *
 * Created on December 11, 2013, 12:00 PM
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

string filename = "displace.txt";
geometry_msgs::Pose target;
double scale;
double initX=.25;
double initY=-.5;


moveit::planning_interface::MoveGroup::Plan planToPose(string joint, move_group_interface::MoveGroup& group, geometry_msgs::Pose* pose);
void fillMap(map<string, vector<double> > &goal, string filename);
void generatePlan(move_group_interface::MoveGroup* group, string joint);

int main(int argc, char** argv)
{
	scale=3.0/2500;
	cout<<"Init Scale:"<<scale;
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
	target.orientation.y = 0.707106781;
	target.orientation.z = 0;
	target.orientation.w = 0.707106781;
	target.position.z = 0;
	group = &rightGroup;
	string joint = "right_wrist";
	generatePlan(group, joint);
	return 0;
}
void generatePlan(move_group_interface::MoveGroup* group, string joint)
{
	moveit::planning_interface::MoveGroup::Plan plan;
	map<string, vector<double> > targetMap;
	fillMap(targetMap, filename);
	vector<double>::iterator xiter = targetMap["x"].begin();	
	vector<double>::iterator yiter = targetMap["y"].begin();
	for (xiter= targetMap["x"].begin(); xiter != targetMap["x"].end(); ++xiter) {
		if (*xiter > 1) {
			for (int i = 0; i < 5; ++i) {
				target.position.x += .01;
				plan = planToPose(joint, *group, &target);
				group->execute(plan);
			}
			system ("python ~/workspaces/hydro_ws/src/drogon_interface/src/gripper.py close right");
			for (int i = 0; i < 5; ++i) {
				target.position.z += .01;
				plan = planToPose(joint, *group, &target);
				group->execute(plan);
			}
			++yiter;
		} else if (*xiter < -1) {
			for (int i = 0; i < 5; ++i) {
				target.position.z -= .01;
				plan = planToPose(joint, *group, &target);
				group->execute(plan);
			}
			system ("python ~/workspaces/hydro_ws/src/drogon_interface/src/gripper.py open right");
			for (int i = 0; i < 5; ++i) {
				target.position.x -= .01;
				plan = planToPose(joint, *group, &target);
				group->execute(plan);
			}
			++yiter;
		} else {
			target.position.x = *xiter+initX;
			target.position.y = *yiter+initY;
			++yiter;
			plan = planToPose(joint, *group, &target);
			group->execute(plan);
		}
	}
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
	goalInput.open(filename.c_str());
	cout<<"File Opened...";
	double value;
	string keyLine = "y,x";
	string valueLine;
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> commaDelimited(", ");
	tokenizer keys(keyLine, commaDelimited);
	while (getline(goalInput, valueLine)) {
		tokenizer values(valueLine, commaDelimited);
		tokenizer::iterator value_iter = values.begin();
		for (tokenizer::iterator key_iter = keys.begin();
		key_iter != keys.end() && value_iter != values.end(); ++key_iter)
		{
			cout<<"\nBefore value "<<scale;
			value = atof((*value_iter).c_str())*scale;
			cout<<"\nValue: "<<value;

			goals[*key_iter].push_back(value);
			++value_iter;
		}
	}
	goalInput.close();
}
