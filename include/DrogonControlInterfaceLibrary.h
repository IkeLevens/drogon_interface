/*
 * File: DrogonControlInterfaceLibrary.h
 * Author: Isaac Yochelson
 *
 * created on June 24, 2013
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

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>	//moveIt! includes
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>

#include <std_msgs/Header.h>//Headers for ros message classes
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>				//Headers for reading joint positions
#include <baxter_msgs/GripperCommand.h>	//Headers for messages from Baxter RSDK
#include <baxter_msgs/JointCommandMode.h>
#include <baxter_msgs/JointPositions.h>
#include <baxter_msgs/JointVelocities.h>
#include <baxter_msgs/EndpointState.h>
#include <baxter_msgs/SolvePositionIK.h>	//Header for service from Baxter RSDK
#ifndef DROGON_INTERFACE_LIBRARY
#define DROGON_INTERFACE_LIBRARY
using namespace std;
namespace drogon {
const int LEFT = 0;
const int RIGHT = 1;
const int OPEN = 2;
const int CLOSE = 3;
// These are the names of the joints for each arm in a Baxter Research Robot
const string jointNames [] = {
	"e0",
	"e1",
	"s0",
	"s1",
	"w0",
	"w1",
	"w2"
};
const int JOINTS = 7;
}
using namespace drogon;
// This class is used to keep a joint state listener on an arm
class State
{
	public:
	map<string, double> data; // This is the state of the joints for this arm
	void stateCallback (const sensor_msgs::JointState& msg); // This is the callback
	//method which will be used in the ros topic subscriber.  It will recieve JointState
	//messages and update data to match the data in the message.
};
// This class is used for kinematics solutions on arm endpoints
class Position
{
	public:
	geometry_msgs::Pose pose; // This pose is the position and orientation of the endpoint
	void positionCallback (const baxter_msgs::EndpointState& msg); // This is the
	//callback method which will be used in the ros topic subscriber.  It will recieve
	//EndpointState messages from the inverse kinematics solver and update pose to match.
	string toString(); // Returns a string representation of the data stored in pose
};
// This class stores the joint angle limits for a Baxter research robot
class Limits
{
	public:
	map <string, double> jointAngleUpperLimits;
	map <string, double> jointAngleLowerLimits;
	Limits(); // This method populates the jointAngleUpperLimits and jointAngleLowerLimits
	//elements with the proper values for a Baxter Research Robot
};
// This is the main class of the library.  It is intended to be used as a control interface
// for a Baxter Research Robot.  This library makes use of Robot Operating System, moveIt,
// and the Baxter Research Software Development Kit.
class DrogonControlInterface
{
	private:
	Limits limits; // Joint angle limits
	State leftState; // current joint angle state for the left arm
	State rightState; // current joint angle state for the right arm
	Position rightPosition; // position of the right end effector
	Position leftPosition; // position of the left end effector
	ros::NodeHandle n;  // keeps a ros nodehandle in scope while the library is in use so
	// that topic and service subscribers can be maintained in the node graph

	// ros topic publishers for motion control
	ros::Publisher leftArmPub;
	ros::Publisher rightArmPub;
	ros::Publisher leftVelocityPub;
	ros::Publisher rightVelocityPub;
	ros::Publisher leftGripPub;
	ros::Publisher rightGripPub;
	ros::Publisher leftCalibratePub;
	ros::Publisher rightCalibratePub;

	// ros topic subscribers for state listeners
	ros::Subscriber leftSub;
	ros::Subscriber rightSub;
	ros::Subscriber leftEndSub;
	ros::Subscriber rightEndSub;

	ros::ServiceClient ikLeftClient; // service client for Baxter RSDK's built in inverse
	// kinematic solver for the left endpoint
	ros::ServiceClient ikRightClient; // service client for Baxter RSDK's built in inverse
	// kinematic solver for the right endpoint
	
	//objects from MoveIt!
	robot_model_loader::RobotModelLoader loader;
	robot_model::RobotModelPtr model;
	robot_state::RobotStatePtr state;
	robot_model::JointModelGroup* leftArmGroup;
	robot_model::JointModelGroup* rightArmGroup;

	move_group_interface::MoveGroup* leftArmPlanner;
	move_group_interface::MoveGroup* rightArmPlanner;

	void setupRobotModel(); // sets up robot model for use in moveit function calls
	void verifyGoalLimits(map<string, double>& goal); // verifies that a goal state's
	// joint angles are within the joint angle limits
	bool closeEnough (map<string, double>& goal, map<string, double>& state); // determines
	// if the current joint state for an arm is close enough to the goal state to be
	// considered a successful motion
	
	// These methods create and return ros topic publishers
	ros::Publisher getArmPublisher(int arm, ros::NodeHandle& n);
	ros::Publisher getCalibratePublisher(int arm, ros::NodeHandle& n);
	ros::Publisher getGripperPublisher(int arm, ros::NodeHandle& n);
	ros::Publisher getVelocityPublisher(int arm);
	ros::Publisher getVelocityPublisher(int arm, ros::NodeHandle& n);

	// These methods create and return ros topic subscribers
	ros::Subscriber getArmSubscriber (int arm, State& state, ros::NodeHandle& n);
	ros::Subscriber getEndSubscriber (int arm, Position& position, ros::NodeHandle& n);

	// This method creates and returns a ros service client
	ros::ServiceClient getIKServiceClient(int arm, ros::NodeHandle& n);
	public:
	DrogonControlInterface(); // constructor for this class, this will be called by executables
	// making use of this library
	void rosEnable(); // enable the Baxter research robot's joint controllers
	void rosDisable(); // disable the Baxter research robot's joint controllers
	void setJointPosition(int arm, map<string, double> goal); // this will move an arm to a goal
	// position using the position mode of the Baxter Research Robot joint controllers
	void setJointVelocities(int arm, map<string, double> velocities, unsigned int time); // this
	// will set joint velocities to move an arm using the velocity mode of the Baxter Research
	// Robot join controllers
	void gripperAction (int arm, int direction); // This moves an end effector
	void calibrate (int arm); // This calibrates an end effector
	void setPositionMode(int arm); // This sets an arm controller of the baxter research robot
	// to position mode
	void setVelocityMode(int arm); // this sets an arm controller of the baxter research robot
	// to velocity mode
	map<string, double> getJointStates(int arm); // this method returns a map of the current
	// angles of the baxter research robot
	bool getRSDKIKSolution (int arm, geometry_msgs::Pose pose, map<string, double> &out); // this
	// retrieves an inverse kinematic solution for a pose of an arm's end effector using the
	// RSDK's built in ik solver
	bool getIKSolution (int arm, geometry_msgs::Pose input, map<string, double> &output);
	// this retrieves an inverse kinematic solution for a pose of an arm's end effector using
	// moveit's kdl ik solver	
	geometry_msgs::Pose getPose(int arm); // this returns the current pose of an end effector
	Position getPosition(int arm); // this returns the position object being used to maintain the
	// cartesian location of an end effector
	void setupMoveGroups(); // this sets move groups for motion planning
	void executePlan(const moveit::planning_interface::MoveGroup::Plan &plan, const int arm); // this executes the plan
	// which is passed by reference as an argument to parameter plan.
	moveit::planning_interface::MoveGroup::Plan getPlan(const map<string, double> &goal, const int arm); // this returns a motion plan from the
	// current robot's current configuration to the goal configuration.
	moveit::planning_interface::MoveGroup::Plan getPlan(const map<string, double> &goal, const map<string, double> &start, const int arm);
	// this returns a motion plan from the start configuration to the goal configuration.
};

#endif
