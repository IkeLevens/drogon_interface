/* 
 * File:   DrogonControlInterfaceKeyboard.cpp
 * Author: Isaac Yochelson
 *
 * Created on June 24, 2013 12:17 PM
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
#include <DrogonControlInterfaceLibrary.h>
#include <baxter_msgs/GripperCommand.h>	//Headers for messages copied from Baxter RSDK
#include <baxter_msgs/JointCommandMode.h>
#include <baxter_msgs/JointPositions.h>
#include <ros/callback_queue.h>
using namespace std;
using namespace drogon;

void printMap (map<string, double> toPrint);
void printHelp ();
char getch();
void mainKeyboardLoop();
DrogonControlInterface* dci;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Drogon_Keyboard_Node");
	dci = new DrogonControlInterface();
	dci->rosEnable();
	dci->setPositionMode(LEFT);
	dci->setPositionMode(RIGHT);
	mainKeyboardLoop();
	dci->rosDisable();
	return 0;
}
void printHelp ()
{
	cout << "left position: " << dci->getPosition(LEFT).toString() << endl;
	cout << "right position: " << dci->getPosition(RIGHT).toString() << endl;
}
void setJoint(int arm, string joint, float delta)
{
	map<string, double> current = dci->getJointStates(arm);
//	printMap(current);
	current[joint] += delta;
	dci->setJointPosition(arm, current);
}
//	This is a debugging function for printing maps in a more useful manner than their memory location.
void printMap (map<string, double> toPrint)
{
	for (unsigned int i=0; i<7; i++)
		cout << jointNames[i] << ":  " << toPrint[jointNames[i]] << endl;
}
//	This method is not our code.  It was recomened as a replacement for getch in a Linux environment
//	on a C++ developers' forum.
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
void mainKeyboardLoop() 
{
	bool running = true;
	char current ='?';
	while (running)
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
				dci->gripperAction(LEFT, OPEN);
				break;
			case ',':
				dci->gripperAction(LEFT, CLOSE);
				break;
			case 'x':
				dci->gripperAction(RIGHT, OPEN);
				break;
			case 'c':
				dci->gripperAction(RIGHT, CLOSE);
				break;
			case '/':
				dci->calibrate(LEFT);
				break;
			case 'b':
				dci->calibrate(RIGHT);
				break;
			case 27:
				ros::shutdown();
				running = false;
				break;
			default:
				printHelp ();
				break;
		}
		current = getch();
	}
}
