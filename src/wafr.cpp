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
void openOutput(string filename);
void closeOutput();

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
		leftGroup.setPlannerId("PRMstarkConfigDefault");
		leftGroup.setStartStateToCurrentState();

		// New functionality goes here.

		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		boost::char_separator<char> commaDelimited(",");
		string keyLine = "left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2";
		string defaults = "-0.708699123193,-0.980980712732,-0.282635959845,1.13859723851,0.141509727521,1.31922347607,-0.00498543755493,0.0\n";
		map<string, double> goal;
		ifstream configs;
		int lines = atoi(argv[1].c_str());
		int step = atoi(argv[2].c_str());
		configs.open(argv[3].c_str());
		int count = 0;
		string configString;
		bool open = true;
		tokenizer keys(keyLine, commaDelimited);
		while (getline(configs, configString)) {
			if ((count % lines) == 0) {
				if (count != 0) {
					closeOutput();
					system("rosrun baxter_examples joint_trajectory_file_playback -f temp.trj");
				}
				tokenizer config(configString, commaDelimited);
				tokenizer::iterator config_iter = config.begin();
				for (tokenizer::iterator key_iter = keys.begin(); key_iter != keys.end();
						++key_iter) {
					goal[*key_iter] = atof((*config_iter).c_str());
					++config_iter;
				}
				leftGroup.setJointValueTarget(goal);
				leftGroup.move();
				if (atoi(*config_iter) == 1) {
					open = true;
					system("rosrun drogon_interface gripper.py open left");
				} else {
					system("rosrun drogon_interface gripper.py close left");
					open = false;
				}
				openOutput("temp.trj");
				outFile << (count * step) << ",";
				for (tokenizer::iterator key_iter = keys.begin(); key_iter != keys.end();
						++key_iter) {
					outFile << goal[*key_iter] << ",";
				}
				if (open) {
					outFile << "100.0,";
				} else {
					outFile << "0.0,";
				}
				outfile << defaults;
			} else {
				tokenizer config(configString, commaDelimited);
				outFile << (count * step) << ",";
				tokenizer::iterator config_iter = config.begin();
				for (size_t i=0; i<7; ++i) {
					outFile << *config_iter << ",";
					++config_iter;
				}
				if (atoi(*config_iter) == 1) {
					outFile << "100.0,";
				} else {
					outFile << "0.0,";
				}
				outFile << defaults;
			}
		}
	return 0;
	}
}
void fillMap(map<string, vector<double> > &goals, string filename)
{
	cout << "filling map: " << filename << endl;
	ifstream goalInput;
	goalInput.open(filename.c_str());
	double value;
	string keyLine = "left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,left_gripper";
	string valueLine;
	cout << keyLine << endl;
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> commaDelimited(",");
	tokenizer keys(keyLine, commaDelimited);
	int count = 0;
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
		++count;
	}
	goalInput.close();
	cout << "fillMap finished: " << count << " lines" << endl;
}
void openOutput(string filename)
{
	outFile.open(filename.c_str());
	outFIle << "time,left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,left_gripper,right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2,right_gripper\n";
}
void closeOutput()
{
	outFile.close();
}

