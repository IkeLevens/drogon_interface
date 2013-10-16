/* 
 * File:   test.cpp
 * Author: Isaac Yochelson
 *
 * Created on August 29, 2013, 10:08 AM
 */
#include <moveit/move_group_interface/move_group.h>
#include <cstdlib>			//standard library for C/C++

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Test_Node");
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	sleep(15);
	// this connecs to a running instance of the move_group node
	move_group_interface::MoveGroup group("left_arm");
	group.setStartStateToCurrentState();
	// specify that our target will be a random one
	double temp[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	vector<double> goal;
	goal.assign(temp, temp+7);
	group.setJointValueTarget(goal);
	// plan the motion and then move the group to the sampled target 
	group.move();
	ros::waitForShutdown();
}
