#include <cstdlib>			//standard library for C/C++
#include <iostream>			//input and output stream packages
#include <fstream>			//file input stream tools for C/C+
#include <sstream>			//For sending topic messages in ROS
#include <ros/ros.h>		//Headers for ros
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>	//moveIt! includes
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <DrogonControlInterfaceLibrary.h>

using namespace std;

void fillMap(map<string, double> &goal, string filename);
DrogonControlInterface* dci;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Planner");
	if (argc < 2) {
		ROS_WARN("a file name is required.\n");
		return -1;
	}
	ROS_INFO("creating DrogonControlInterface\n");
	dci = new DrogonControlInterface();
	ROS_INFO("created DrogonControlInterface\n");
//	dci->waitForMoveit();
	dci->rosEnable();
	map<string, double> goal;
	fillMap(goal, argv[1]);
//	int count = 0;
	stringstream ss;
	typedef std::map<std::string, double>::iterator it_type;
	for(it_type iterator = goal.begin(); iterator != goal.end(); iterator++) {
//		ss << ++count;
//		ROS_INFO(ss.str().c_str());
		ROS_INFO(iterator->first.c_str());
//		ss.str("");
		ss << iterator->second;
		ROS_INFO(ss.str().c_str());
		ss.str("");
	}
	ROS_INFO("requesting plan\n");
	moveit::planning_interface::MoveGroup::Plan* plan = dci->getPlan(goal, drogon::LEFT);
	ROS_INFO("plan received\n");
	dci->executePlan(*plan, drogon::LEFT);
	ROS_INFO("plan executed\n");
	ros::spin();
	return 0;
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
/*
void savePlan(string filename, moveit::planning_interface::MoveGroup::Plan* plan)
{
	ofstream planOutput;
	planOutput << "time";
	planOutput.open(filename.c_str());
	robot_state::RobotState current = plan->trajectory_.getWaypoint(0);
	vector<string> names = current.getVariableNames();
	int wpCount = 1; //get the count of the waypoints in plan.
	int jointCount = 0;
	for (vector<string>::iterator it = names.begin(); it != names.end(); ++it) {
		planOutput << "," << *it;
		++jointCount;
	}
	planOutput << "\n";
	for (int wp = 0; wp < wpCount; ++wp) {
		double timeStamp = plan->trajectory_.getWaypointDurationFromStart(wp);
		current = plan->trajectory_.getWaypoint(wp);
		stringstream linestream;
		linestream << timeStamp;
		double* positions = current.getVariablePositions();
		for (int c = 0; c < jointCount; ++c) {
			linestream << "," << dtos(positions[c];
		}
		linestream << "\n";
		planOutput << linstream.str();
	}
	planOutput.close();
}
*/
