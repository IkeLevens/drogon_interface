/* 
 * File:   test.cpp
 * Author: Isaac Yochelson
 *
 * Created on August 29, 2013, 10:08 AM
 */
#include <DrogonControlInterfaceLibrary.h>
#include <cstdlib>			//standard library for C/C++

using namespace std;

DrogonControlInterface* dci;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Keyboard_Node");
	dci = new DrogonControlInterface();
	cout << "created dci" << endl;
	geometry_msgs::Pose pose;
	geometry_msgs::Point position;
	geometry_msgs::Quaternion orientation;
	printf("geometry messages created\n");
	position.x=0.657;
	position.y=0.852;
	position.z=0.039;
	orientation.x=-0.366894936773;
	orientation.y=0.885980397775;
	orientation.z=0.108155782462;
	orientation.w=0.262162481772;
	pose.position = position;
	pose.orientation = orientation;
	printf("pose completed\n");
	std::map<string, double> out;
	dci->getIKSolution(drogon::LEFT, pose, out);
	printf("got IK solution\n");
	for (int i= 0; i < 7; ++i) {
		std::cout << "joint: " << drogon::jointNames[i] << " value: " << out[drogon::jointNames[i]] << std::endl;
	}
	return 0;
}
