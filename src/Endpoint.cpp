#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <sys/timeb.h>
#include <baxter_core_msgs/EndpointState.h>
using namespace std;

ofstream leftStream;
ofstream rightStream;

timeb timmy;
short leftRecord;
short rightRecord;
void leftCallback (const baxter_core_msgs::EndpointState& msg);
void rightCallback (const baxter_core_msgs::EndpointState& msg);
int main (int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Endpoint_Listener");
	ros::NodeHandle n;
	leftStream.open("EndpointDumpLeft.txt");
	rightStream.open("EndpointDumpRight.txt");
	ros::Subscriber sub1 = n.subscribe("/robot/limb/left/endpoint/state", 1, leftCallback);
	ros::Subscriber sub2 = n.subscribe("/robot/limb/right/endpoint/state", 1, rightCallback);
	ftime(&timmy);
	leftRecord = timmy.millitm / 100;
	rightRecord = leftRecord;
	cout << leftRecord << endl;
	ros::spin();
	leftStream.close();
	rightStream.close();
	return 0;
}
void leftCallback (const baxter_core_msgs::EndpointState& msg)
{
	ftime(&timmy);
	if (leftRecord == 9 && timmy.millitm / 100 == 0) {
		leftRecord = -1;
	}
	if (timmy.millitm / 100 > leftRecord) {
		leftRecord = (timmy.millitm / 100);
		leftStream << "time: " << timmy.time << leftRecord << " x: " << msg.pose.position.x << " y: "
			<< msg.pose.position.y << " z: " << msg.pose.position.z << "\n";
	}
}
void rightCallback (const baxter_core_msgs::EndpointState& msg)
{
	ftime(&timmy);
	if (rightRecord == 9 && timmy.millitm / 100 == 0) {
		rightRecord = -1;
	}
	if (timmy.millitm / 100 > rightRecord) {
		rightRecord = timmy.millitm / 100;
		rightStream << "time: " << timmy.time << rightRecord << "x: " << msg.pose.position.x << " y: "
			<< msg.pose.position.y << " z: " << msg.pose.position.z << "\n";
	}
}
