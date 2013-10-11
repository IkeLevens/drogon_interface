#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

void commandCallback (const std_msgs::String::ConstPtr& msg)
{
	string data = msg->data;
	if (data.find(';') == std::string::npos) {
		system(("rosrun joint_trajectory file_playback.py -f " + data).c_str());
	}
}
int main (int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Web_Server");
	ros::NodeHandle n;
	system("rosrun tools enable_robot.py -e");
	ros::Subscriber sub = n.subscribe("/web", 1, commandCallback);
	ros::spin();
	system("rosrun tools enable_robot.py -d");
	return 0;
}
