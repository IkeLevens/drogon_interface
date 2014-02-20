/* 
 * File:   clear.cpp
 * Author: Isaac Yochelson
 *
 * Created on February 5, 2014 1:02 PM
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

map<string, double> clearState;
move_group_interface::MoveGroup* bothArmsGroup;

void fillClearState();
void clear();

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Clear_Node");
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	move_group_interface::MoveGroup bothGroup("both_arms");
	bothGroup.setPlannerId("PRMstarkConfigDefault");
	bothGroup.setStartStateToCurrentState();
	bothArmsGroup = &bothGroup;
	fillClearState();
	clear();
	return 0;
}
void fillClearState()
{
	clearState["left_s0"]=0.8574952594;
	//clearState["left_s0"]=-0.5;
	clearState["left_s1"]=-1.106383642;
	clearState["left_e0"]=-0.0720970969482;
	clearState["left_e1"]=1.260548711;
	clearState["left_w0"]=-0.0720970969482;
	clearState["left_w1"]=1.1830826813;
	clearState["left_w2"]=-0.00498543755493;
	clearState["right_s0"]=-0.708699123193;
	clearState["right_s1"]=-0.980980712732;
	clearState["right_e0"]=-0.282635959845;
	clearState["right_e1"]=1.13859723851;
	clearState["right_w0"]=0.141509727521;
	clearState["right_w1"]=1.31922347607;
	clearState["right_w2"]=-0.00498543755493;
}
void clear()
{
	bothArmsGroup->setJointValueTarget(clearState);
	bothArmsGroup->move();
}

