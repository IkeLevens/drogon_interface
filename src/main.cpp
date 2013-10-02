/* 
 * File:   main.cpp
 * Author: pracsys
 *
 * Created on June 4, 2013, 2:51 PM
 */

#include <cstdlib>			//standard library for C/C++
#include <iostream>			//input and output stream packages
#include <string>			//character string class
#include <map>				//data structure to store value by reference key
#include <stdio.h>			//headers for standard input and output
#include <unistd.h>			//_getch*/
#include <termios.h>		//_getch*/
#include <unistd.h>
//#include <roscpp>			//ROS C++ API
#include <sstream>			//For sending topic messages in ROS
#include <ros/ros.h>		//Headers for ros
#include <std_msgs/Header.h>//Headers for ros message classes
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>				//Headers for reading joint positions
#include <drogon_interface/GripperCommand.h>	//Headers for messages copied from Baxter RSDK
#include <drogon_interface/JointCommandMode.h>
#include <drogon_interface/JointPositions.h>
#include <ros/callback_queue.h>
using namespace std;
using namespace drogon_interface;

int LEFT = 0;
int RIGHT = 1;
int OPEN = 2;
int CLOSE = 3;
ros::Publisher leftArmPub;
ros::Publisher rightArmPub;
ros::Publisher leftGripPub;
ros::Publisher rightGripPub;
ros::Publisher leftCalibratePub;
ros::Publisher rightCalibratePub;
ros::Publisher leftModePub;
ros::Publisher rightModePub;
ros::Subscriber leftArmSub;
ros::Subscriber rightArmSub;
string jointNames [] = {
	"e0",
	"e1",
	"s0",
	"s1",
	"w0",
	"w1",
	"w2"
};
map<string, double> jointAngleUpperLimits;
map<string, double> jointAngleLowerLimits;
map<string, float> leftGoal;
map<string, float> rightGoal;
map<string, float> leftState;
map<string, float> rightState;
GripperCommand closeCommand;
GripperCommand openCommand;

void rosSetup(ros::NodeHandle n);
void rosDestroy();
void setJointPosition(int arm);
void setJoint (int arm, string joint, float delta);
void gripperAction (int arm, int direction);
void calibrate (int arm, ros::NodeHandle n);
void printHelp ();
void setLeftPositionMode();
void setRightPositionMode();
void setLeftVelocityMode();
void setRightVelocityMode();
void mapDefaults();
char getch();
void mainKeyboardLoop(ros::NodeHandle n);
bool closeEnough (map<string, float> *goal, map<string, float> *state);
void setLimits();

int main(int argc, char** argv) {
	setLimits();
	ros::init(argc, argv, "drogon_interface");
	ros::NodeHandle n;
	rosSetup (n);
	vector<string> names;
	ros::this_node::getSubscribedTopics(names);
	for (unsigned i=0;i<names.size();i++)
		cout<<names[i]<<endl;
	closeCommand.position=0.0;
	closeCommand.force=30.0;
	closeCommand.velocity=100.0;
	closeCommand.holding=0.0;
	closeCommand.deadZone=3.0;
	openCommand.position=100.0;
	openCommand.force=30.0;
	openCommand.velocity=100.0;
	openCommand.holding=0.0;
	openCommand.deadZone=3.0;
	mapDefaults();
	sleep(2);
	ros::spinOnce();
//	ros::MultiThreadedSpinner spinner(4);
//	spinner.spin();
//	ros::spin();
	setJointPosition(LEFT);
	setJointPosition(RIGHT);
	mainKeyboardLoop(n);
	rosDestroy();
	return 0;
}
void leftStateCallback (const sensor_msgs::JointState& msg)
{
//	cout << "leftStateCallback" << endl;
	for (int i=0; i<7; i++)
	{
		leftState[jointNames[i]] = msg.position[i];
	}
}
void rightStateCallback (const sensor_msgs::JointState& msg)
{
//	cout << "rightStateCallback" << endl;
	for (int i=0; i<7; i++)
	{
		rightState[jointNames[i]] = msg.position[i];
	}
}
void setLimits ()
{
	jointAngleUpperLimits["s0"]=1.70167993878;
	jointAngleUpperLimits["s1"]=1.047;
	jointAngleUpperLimits["e0"]=3.05417993878;
	jointAngleUpperLimits["e1"]=2.618;
	jointAngleUpperLimits["w0"]=3.059;
	jointAngleUpperLimits["w1"]=2.094;
	jointAngleUpperLimits["w2"]=3.059;
	jointAngleLowerLimits["s0"]=-1.70167993878;
	jointAngleLowerLimits["s1"]=-2.147;
	jointAngleLowerLimits["e0"]=-3.05417993878;
	jointAngleLowerLimits["e1"]=-0.05;
	jointAngleLowerLimits["w0"]=-3.059;
	jointAngleLowerLimits["w1"]=-1.571;
	jointAngleLowerLimits["w2"]=-3.059;
}
void rosSetup (ros::NodeHandle n)
{
	system("rosrun tools enable_robot.py -e");
	string s = "robot/limb/left/command_joint_angles";
	leftModePub = n.advertise<JointCommandMode>("robot/limb/left/joint_command_mode", 3, true);
	rightModePub = n.advertise<JointCommandMode>("robot/limb/right/joint_command_mode", 3, true);
	leftArmSub = n.subscribe("robot/limb/left/joint_states", 1, leftStateCallback);
	rightArmSub = n.subscribe("robot/limb/right/joint_states", 1, rightStateCallback);
	setLeftPositionMode();
	setRightPositionMode();
	leftArmPub = n.advertise<JointPositions>(s, 3, true);
	rightArmPub = n.advertise<JointPositions>("robot/limb/right/command_joint_angles", 3, true);
	leftGripPub = n.advertise<GripperCommand>("robot/limb/left/accessory/gripper/command_set", 3, true);
	leftCalibratePub = n.advertise<std_msgs::Empty>("robot/limb/left/accessory/gripper/command_calibrate", 3, true);
	rightGripPub = n.advertise<GripperCommand>("robot/limb/right/accessory/gripper/command_set", 3, true);
	rightCalibratePub = n.advertise<std_msgs::Empty>("robot/limb/right/accessory/gripper/command_calibrate", 3, true);
	ros::Publisher ratePub = n.advertise<std_msgs::UInt16>("robot/joint_state_publish_rate", 1, true);
	std_msgs::UInt16 msg;
	msg.data = 10;
	ratePub.publish(msg);
}
void rosDestroy()
{
	system("rosrun tools enable_robot.py -d");
}
void setJointPosition(int arm)
{
	map<string, float> *state = 0;
	ros::Publisher *pub = 0;
	map<string, float> *goal = 0;
	JointPositions msg;
	if (arm == LEFT)
	{
		pub = &leftArmPub;
		goal = &leftGoal;
		state= &leftState;
	}
	else if (arm == RIGHT)
	{
		pub = &rightArmPub;
		goal = &rightGoal;
		state = &rightState;
	}
	else
		cout <<"Error in arm selection." << endl;
	std::cout << " arm : " << arm << std::endl;
	for (int i=0; i<7; ++i)
	{
//		cout << i << ") " << jointNames[i] << endl;
		msg.names.push_back(jointNames[i]);
//		cout << " msg name :" << msg.names[i] << endl;
		msg.angles.push_back(goal->at(jointNames[i]));
//		cout << " msg angle : " << msg.angles[i] << endl;
	}
	std::cout << " done with names! " << endl;
	while (!closeEnough(goal, state))
//	for (int i=0; i<30; i++)
	{
//		cout << "in while loop" << endl;
//		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
		pub->publish(msg);
		usleep(33000);
	}
}
void setJoint (int arm, string joint, float delta)
{
	if (arm == LEFT)
	{
//		cout << joint << "   value: " << leftGoal[joint] << "   delta: " << delta << std::endl;
		double goal = leftGoal[joint];
		if (goal + delta > jointAngleUpperLimits[joint])
			goal = jointAngleUpperLimits[joint];
		else if (goal + delta < jointAngleLowerLimits[joint])
			goal = jointAngleLowerLimits[joint];
		else
			goal = goal + delta;
		leftGoal[joint] = goal;
	}
	else
	{
//		std::cout << joint << "   value: " << rightGoal[joint] << "   delta: " << delta << std::endl;
		double goal = rightGoal[joint];
		if (goal + delta > jointAngleUpperLimits[joint])
			goal = jointAngleUpperLimits[joint];
		else if (goal + delta < jointAngleLowerLimits[joint])
			goal = jointAngleLowerLimits[joint];
		else
			goal = goal + delta;
		rightGoal[joint] = goal;
	}
	setJointPosition(arm);
}
void gripperAction (int arm, int direction)
{
	if (arm == LEFT)
	{
		if (direction == CLOSE)
		{
			leftGripPub.publish(closeCommand);
		}
		else
		{
			leftGripPub.publish(openCommand);
		}
	}
	else if (arm == RIGHT)
	{
		if (direction == CLOSE)
		{
			rightGripPub.publish(closeCommand);
		}
		else
		{
			rightGripPub.publish(openCommand);
		}
	}
	else
	{
		cout <<"Error in arm selection." << endl;
	}
	return;
}
void calibrate (int arm, ros::NodeHandle n)
{
	std_msgs::Bool msg;
	msg.data=true;
	if (arm == LEFT)
	{
		ros::Publisher pub = n.advertise<std_msgs::Bool>("robot/limb/left/accessory/gripper/set_enabled", 1000);
		pub.publish(msg);
		leftCalibratePub.publish(std_msgs::Empty());
	}
	else if (arm == RIGHT)
	{
		ros::Publisher pub = n.advertise<std_msgs::Bool>("robot/limb/right/accessory/gripper/set_enabled", 1000);
		pub.publish(msg);
		rightCalibratePub.publish(std_msgs::Empty());
	}
	else
	{
		cout <<"Error in arm selection." << endl;
	}
	return;
}
void printHelp ()
{
	cout << "leftState: " << endl;
	for (int i=0; i<7; i++)
		cout << jointNames[i] << ": " << leftState[jointNames[i]] << endl;
	cout << "leftGoal: " << endl;
	for (int i=0; i<7; i++)
		cout << jointNames[i] << ": " << leftGoal[jointNames[i]] << endl;
	cout << "rightState: " << endl;
	for (int i=0; i<7; i++)
		cout << jointNames[i] << ": " << rightState[jointNames[i]] << endl;
	cout << "rightGoal: " << endl;
	for (int i=0; i<7; i++)
		cout << jointNames[i] << ": " << rightGoal[jointNames[i]] << endl;
	return;
}
void setLeftPositionMode()
{
	JointCommandMode msg;
	msg.mode = JointCommandMode::POSITION;
	leftModePub.publish(msg);
}
void setRightPositionMode()
{
	JointCommandMode msg;
	msg.mode = JointCommandMode::POSITION;
	rightModePub.publish(msg);
}
void setLeftVelocityMode()
{
	JointCommandMode msg;
	msg.mode = JointCommandMode::VELOCITY;
	leftModePub.publish(msg);
}
void setRightVelocityMode()
{
	JointCommandMode msg;
	msg.mode = JointCommandMode::VELOCITY;
	rightModePub.publish(msg);
}
void mapDefaults()
{
	leftGoal["s0"] = 0.0;
	leftGoal["s1"] = -0.55;
	leftGoal["e0"] = 0.0;
	leftGoal["e1"] = 0.75;
	leftGoal["w0"] = 0.0;
	leftGoal["w1"] = 1.26;
	leftGoal["w2"] = 0.0;
	rightGoal["s0"] = 0.0;
	rightGoal["s1"] = -0.55;
	rightGoal["e0"] = 0.0;
	rightGoal["e1"] = 0.75;
	rightGoal["w0"] = 0.0;
	rightGoal["w1"] = 1.26;
	rightGoal["w2"] = 0.0;
	leftState["s0"] = 0.0;
	leftState["s1"] = -0.55;
	leftState["e0"] = 0.0;
	leftState["e1"] = 0.75;
	leftState["w0"] = 0.0;
	leftState["w1"] = 1.26;
	leftState["w2"] = 0.0;
	rightState["s0"] = 0.0;
	rightState["s1"] = -0.55;
	rightState["e0"] = 0.0;
	rightState["e1"] = 0.75;
	rightState["w0"] = 0.0;
	rightState["w1"] = 1.26;
	rightState["w2"] = 0.0;
}
char getch(){
	char buf=0;
	struct termios old={0};
	fflush(stdout);
	if(tcgetattr(0, &old)<0)
		perror("tcsetattr()");
	old.c_lflag&=~ICANON;
	old.c_lflag&=~ECHO;
	old.c_cc[VMIN]=1;
	old.c_cc[VTIME]=0;
	if(tcsetattr(0, TCSANOW, &old)<0)
		perror("tcsetattr ICANON");
	if(read(0,&buf,1)<0)
		perror("read()");
	old.c_lflag|=ICANON;
	old.c_lflag|=ECHO;
	if(tcsetattr(0, TCSADRAIN, &old)<0)
		perror ("tcsetattr ~ICANON");
//	  printf("%c\n",buf);		//echo the button that was pressed.
	return buf;
 }
bool closeEnough(map<string, float> *goal, map<string, float> *state)
{
	ros::spinOnce();
//	cout << "begin closeEnough" << endl;
//	cout << "goal: " << goal << endl;
//	cout << "state: " << state << endl;
	for (int i=0; i<7; i++)
	{
//		cout << jointNames[i] << ": " << state->at(jointNames[i]) << endl;
//		cout << jointNames[i] << ": " << goal->at(jointNames[i]) << endl;
		if (goal->at(jointNames[i]) - state->at(jointNames[i]) > .025 || goal->at(jointNames[i]) - state->at(jointNames[i]) < -.025)
		{
			cout << "closeEnough: false" << endl;
			return false;
		}
	}
	cout << "closeEnough: true" << endl;
	return true;
}
void mainKeyboardLoop(ros::NodeHandle n) 
{
	char current ='?';
	vector<string> names;
//	ros::this_node::getSubscribedTopics(names);
	for (int i=0; i< names.size(); i++)
		cout << names[i] << endl;
	while (ros::ok())
	{
		switch (current)
		{
			case '9':
				setJoint(LEFT, "s0", 0.1);
				break;
			case '6':
				setJoint(LEFT, "s0", -0.1);
				break;
			case '7':
				setJoint(LEFT, "s1", 0.1);
				break;
			case '8':
				setJoint(LEFT, "s1", -0.1);
				break;
			case 'o':
				setJoint(LEFT, "e0", 0.1);
				break;
			case 'y':
				setJoint(LEFT, "e0", -0.1);
				break;
			case 'i':
				setJoint(LEFT, "e1", 0.1);
				break;
			case 'u':
				setJoint(LEFT, "e1", -0.1);
				break;
			case 'l':
				setJoint(LEFT, "w0", 0.1);
				break;
			case 'h':
				setJoint(LEFT, "w0", -0.1);
				break;
			case 'k':
				setJoint(LEFT, "w1", 0.1);
				break;
			case 'j':
				setJoint(LEFT, "w1", -0.1);
				break;
			case '.':
				setJoint(LEFT, "w2", 0.1);
				break;
			case 'n':
				setJoint(LEFT, "w2", -0.1);
				break;
			case '4':
				setJoint(RIGHT, "s0", 0.1);
				break;
			case '1':
				setJoint(RIGHT, "s0", -0.1);
				break;
			case '3':
				setJoint(RIGHT, "s1", 0.1);
				break;
			case '2':
				setJoint(RIGHT, "s1", -0.1);
				break;
			case 'r':
				setJoint(RIGHT, "e0", 0.1);
				break;
			case 'q':
				setJoint(RIGHT, "e0", -0.1);
				break;
			case 'e':
				setJoint(RIGHT, "e1", 0.1);
				break;
			case 'w':
				setJoint(RIGHT, "e1", -0.1);
				break;
			case 'f':
				setJoint(RIGHT, "w0", 0.1);
				break;
			case 'a':
				setJoint(RIGHT, "w0", -0.1);
				break;
			case 'd':
				setJoint(RIGHT, "w1", 0.1);
				break;
			case 's':
				setJoint(RIGHT, "w1", -0.1);
				break;
			case 'v':
				setJoint(RIGHT, "w2", 0.1);
				break;
			case 'z':
				setJoint(RIGHT, "w2", -0.1);
				break;
			case 'm':
				gripperAction(LEFT, OPEN);
				break;
			case ',':
				gripperAction(LEFT, CLOSE);
				break;
			case 'x':
				gripperAction(RIGHT, OPEN);
				break;
			case 'c':
				gripperAction(RIGHT, CLOSE);
				break;
			case '/':
				calibrate(LEFT, n);
				break;
			case 'b':
				calibrate(RIGHT, n);
				break;
			case 27:
				ros::shutdown();
				break;
			default:
				printHelp ();
				break;
		}
		current = getch();
	}
}
