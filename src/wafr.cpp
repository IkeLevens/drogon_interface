/* 
 * File:   wafr.cpp
 * Author: Isaac Yochelson
 *
 * Created on March 24, 2014, 3:22 PM
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

ofstream outFile;
double addTime = 0;
geometry_msgs::Pose target;
map<string, double> clearState;
move_group_interface::MoveGroup* bothArmsGroup;

moveit::planning_interface::MoveGroup::Plan planToPose(string joint, move_group_interface::MoveGroup& group, geometry_msgs::Pose* pose);
void openPlan(string filename, moveit::planning_interface::MoveGroup::Plan plan);
void savePlan(moveit::planning_interface::MoveGroup::Plan plan, string joint);
void closePlan();
void fillMap(map<string, vector<double> > &goal, string filename);
void generateAndSavePlan(move_group_interface::MoveGroup* group, string joint);

int main(int argc, char** argv)
{
	if (arc < 4) {
		cout << "usage: wafr <# lines / file> <ms per step> <input filename>\n";
		return -1;
	}
	else {
		ros::init(argc, argv, "WAFR_Node");
		// start a ROS spinning thread
		ros::AsyncSpinner spinner(1);
		spinner.start();
		
		// Create MoveGroup instances for each arm, set them to PRM*, and create a pointer to be assigned
		// to the correct arm to be used for each target.
		move_group_interface::MoveGroup leftGroup("left_arm");
		move_group_interface::MoveGroup rightGroup("right_arm");
		move_group_interface::MoveGroup bothGroup("both_arms");
		leftGroup.setPlannerId("PRMstarkConfigDefault");
		leftGroup.setStartStateToCurrentState();
		rightGroup.setPlannerId("PRMstarkConfigDefault");
		rightGroup.setStartStateToCurrentState();
		bothGroup.setPlannerId("PRMstarkConfigDefault");
		bothGroup.setStartStateToCurrentState();
		bothArmsGroup = &bothGroup;
		fillClearState();
		move_group_interface::MoveGroup* group;

		target.orientation.x = 0;
		target.orientation.y = 1;
		target.orientation.z = 0;
		target.orientation.w = 0;
		group = &rightGroup;
		string joint = "right_wrist";
		
		addTime = 0;
		stringstream filestream;
		
		// New functionality goes here.
		return 0;
	}
}
void generateAndSavePlan(move_group_interface::MoveGroup* group, string joint)
{
	
	moveit::planning_interface::MoveGroup::Plan plan;
	map<string, vector<double> > targetMap;
	fillMap(targetMap, filename);
	vector<double>::iterator xiter = targetMap["x"].begin();	
	vector<double>::iterator yiter = targetMap["y"].begin();
	vector<double>::iterator ziter = targetMap["z"].begin();
	vector<double>::iterator qxiter = targetMap["qx"].begin();
	vector<double>::iterator qyiter = targetMap["qy"].begin();
	vector<double>::iterator qziter = targetMap["qz"].begin();
	vector<double>::iterator qwiter = targetMap["qw"].begin();
	cout << "iterators set" << endl;
	for (xiter= targetMap["x"].begin(); xiter != targetMap["x"].end(); ++xiter) {
		target.position.x = *xiter;
		target.position.y = *yiter;
		target.position.z = *ziter;
		cout << "position set" << endl;
		target.orientation.x = *qxiter;
		target.orientation.y = *qyiter;
		target.orientation.z = *qziter;
		target.orientation.w = *qwiter;
		cout << "orientation set" << endl;
		++yiter;
		++ziter;
		++qxiter;
		++qyiter;
		++qziter;
		++qwiter;
		cout << "planning to: " << target << endl;
		plan = planToPose(joint, *group, &target);
		if (xiter == targetMap["x"].begin()) {
			openPlan(output, plan);
		}
		savePlan(plan, joint);
	}
	cout << "closing plan" << endl;
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
	cout << "filling map: " << filename << endl;
	ifstream goalInput;
	goalInput.open(filename.c_str());
	double value;
	string keyLine = "x,y,z,qx,qy,qz,qw";
	string valueLine;
	cout << keyLine << endl;
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> commaDelimited("\t, ");
	tokenizer keys(keyLine, commaDelimited);
	int count = 0;
	while (getline(goalInput, valueLine)) {
		++count;
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
	cout << "fillMap finished: " << count << " lines" << endl;
}
void openPlan(string filename, moveit::planning_interface::MoveGroup::Plan plan)
{
	planOutput.open(filename.c_str());
	planOutput << "time,left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,left_gripper,right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2,right_gripper\n";
}
void closePlan()
{
	planOutput.close();
}
void savePlan(moveit::planning_interface::MoveGroup::Plan plan, string joint)
{
	double secs = 0;
	for(vector<trajectory_msgs::JointTrajectoryPoint>::iterator pt =  plan.trajectory_.joint_trajectory.points.begin();
			pt !=  plan.trajectory_.joint_trajectory.points.end(); pt++) {
		secs = (*pt).time_from_start.toSec() / 2;
		stringstream pointStream;
		pointStream << secs + addTime;
		if (joint.compare("left_wrist")==0) {
			for(vector<double>::iterator dt = (*pt).positions.begin(); dt != (*pt).positions.end(); dt++) {
				pointStream << "," << *dt;
			}
			pointStream << ",100";
			pointStream << "," << clearState["right_s0"];
			pointStream << "," << clearState["right_s1"];
			pointStream << "," << clearState["right_e0"];
			pointStream << "," << clearState["right_e1"];
			pointStream << "," << clearState["right_w0"];
			pointStream << "," << clearState["right_w1"];
			pointStream << "," << clearState["right_w2"];
			pointStream << ",100";
		} else {
			pointStream << "," << clearState["left_s0"];
			pointStream << "," << clearState["left_s1"];
			pointStream << "," << clearState["left_e0"];
			pointStream << "," << clearState["left_e1"];
			pointStream << "," << clearState["left_w0"];
			pointStream << "," << clearState["left_w1"];
			pointStream << "," << clearState["left_w2"];
			pointStream << ",100";
			for(vector<double>::iterator dt = (*pt).positions.begin(); dt != (*pt).positions.end(); dt++) {
				pointStream << "," << *dt;
			}
			pointStream << ",100";
		}
		planOutput << pointStream.str() << "\n";
	}
	addTime += secs;
}

