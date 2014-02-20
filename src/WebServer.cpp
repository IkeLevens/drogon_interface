#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

ros::Publisher* pub;
void commandCallback (const std_msgs::String::ConstPtr& msg);
int main (int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Web_Server");
	ros::NodeHandle n;
	system("rosrun baxter_tools enable_robot.py -e");
	ros::Publisher temp = n.advertise<std_msgs::Bool>("/playback", 1, false);
	pub = &temp;
	ros::Subscriber sub = n.subscribe("/web", 1, commandCallback);
	ros::spin();
	system("rosrun baxter_tools enable_robot.py -d");
	return 0;
}
void commandCallback (const std_msgs::String::ConstPtr& msg)
{
	string data = msg->data;
	if (data.find(';') == std::string::npos) {
		std_msgs::Bool msg;
		msg.data = true;
		pub->publish(msg);
		system(("rosrun baxter_examples joint_trajectory_file_playback.py -f " + data).c_str());
		msg.data=false;
		pub->publish(msg);
	}
}
