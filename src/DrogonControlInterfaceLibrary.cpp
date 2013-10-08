/* 
 * File:   DrogonControlInterfaceLibrary.cpp
 * Author: Isaac Yochelson
 *
 * Created on June 24, 2013, 10:08 AM
 */
#include <DrogonControlInterfaceLibrary.h>

using namespace std;
using namespace drogon;

Limits::Limits()
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
void State::stateCallback (const sensor_msgs::JointState& msg)
{
//		cout << "leftStateCallback" << endl;
	for (int i=0; i<7; i++)
	{
		data[jointNames[i]] = msg.position[i];
	}
}
void Position::positionCallback (const baxter_msgs::EndpointState& msg)
{
	pose = msg.pose;
}
string Position::toString ()
{
	stringstream ss;
	ss << "x:";
	ss << pose.position.x;
	ss << "y: ";
	ss << pose.position.y;
	ss << "z: ";
	ss << pose.position.z;
	ss << "orientation: ";
	ss << pose.orientation.x;
	ss <<" ";
	ss << pose.orientation.y;
	ss << " ";
	ss << pose.orientation.z;
	ss << " ";
	ss << pose.orientation.w;
	return ss.str();
}
WebListener::WebListener()
{
	webEnabled = false;
}
void WebListener::enableWeb ()
{
	webEnabled = true;
}
void WebListener::disableWeb ()
{
	webEnabled = false;
}
void WebListener::commandCallback (const std_msgs::String& msg)
{
	if (webEnabled && (msg.data.find(';') == std::string::npos)) {
		system(("rosrun joint_trajectory file_playback.py -f " + msg.data).c_str());
	}
}
DrogonControlInterface::DrogonControlInterface()
{
	leftArmPub = getArmPublisher(LEFT, n);
	rightArmPub = getArmPublisher(RIGHT, n);
	leftVelocityPub = getVelocityPublisher(LEFT, n);
	rightVelocityPub = getVelocityPublisher(RIGHT, n);
	leftGripPub = getGripperPublisher(LEFT, n);
	rightGripPub = getGripperPublisher(RIGHT, n);
	leftCalibratePub = getCalibratePublisher(LEFT, n);
	rightCalibratePub = getCalibratePublisher(RIGHT, n);
	leftSub = getArmSubscriber(LEFT, leftState, n);
	rightSub = getArmSubscriber(RIGHT, rightState, n);
	leftEndSub = getEndSubscriber(LEFT, leftPosition, n);
	rightEndSub = getEndSubscriber(RIGHT, rightPosition, n);
	ikLeftClient = getIKServiceClient(LEFT, n);
	ikRightClient = getIKServiceClient(RIGHT, n);
	webSubscriber = getWebSubscriber(webListener, n);
	while(!(ikLeftClient.exists()&&ikRightClient.exists()))
	{
		ROS_INFO("Waiting for service");
		sleep(1.0);
	}
	ros::AsyncSpinner spinner(2);
	spinner.start();
	setupRobotModel();
}
ros::ServiceClient DrogonControlInterface::getIKServiceClient(int arm, ros::NodeHandle& n)
{
	if (arm == LEFT)
	{
		return n.serviceClient<baxter_msgs::SolvePositionIK>("/sdk/robot/limb/left/solve_ik_position");
	}
	else
	{
		return n.serviceClient<baxter_msgs::SolvePositionIK>("/sdk/robot/limb/right/solve_ik_position");
	}
}
ros::Subscriber DrogonControlInterface::getArmSubscriber (int arm, State& state, ros::NodeHandle& n)
{
	ros::Subscriber sub;
	if (arm == LEFT)
	{
		sub = n.subscribe("robot/limb/left/joint_states", 1, &State::stateCallback, &state);
	}
	else
	{
		sub = n.subscribe("robot/limb/right/joint_states", 1, &State::stateCallback, &state);
	}
	return sub;
}
ros::Subscriber DrogonControlInterface::getEndSubscriber (int arm, Position& position, ros::NodeHandle& n)
{
	ros::Subscriber sub;
	if (arm == LEFT)
	{
		sub = n.subscribe("sdk/robot/limb/left/endpoint/state", 1, &Position::positionCallback, &position);
	}
	else
	{
		sub = n.subscribe("sdk/robot/limb/right/endpoint/state", 1, &Position::positionCallback, &position);
	}
	return sub;
}
ros::Subscriber DrogonControlInterface::getWebSubscriber (WebListener& webListener, ros::NodeHandle& n)
{
	ros::Subscriber sub;
	sub = n.subscribe("/web", 1, &WebListener::commandCallback, &webListener);
	return sub;
}
ros::Publisher DrogonControlInterface::getCalibratePublisher(int arm, ros::NodeHandle& n)
{
	ros::Publisher pub;
	if (arm == LEFT)
	{
		pub = n.advertise<std_msgs::Empty>("robot/limb/left/accessory/gripper/command_calibrate", 3, true);
	}
	else
	{
		pub = n.advertise<std_msgs::Empty>("robot/limb/right/accessory/gripper/command_calibrate", 3, true);
	}
	return pub;
}
ros::Publisher DrogonControlInterface::getGripperPublisher(int arm, ros::NodeHandle& n)
{
	ros::Publisher pub;
	if (arm == LEFT)
	{
		pub = n.advertise<baxter_msgs::GripperCommand>("robot/limb/left/accessory/gripper/command_set", 3, true);
	}
	else
	{
		pub =  n.advertise<baxter_msgs::GripperCommand>("robot/limb/right/accessory/gripper/command_set", 3, true);
	}
	return pub;
}
ros::Publisher DrogonControlInterface::getArmPublisher(int arm, ros::NodeHandle& n)
{
	ros::Publisher pub;
	if (arm == LEFT)
	{
		pub = n.advertise<baxter_msgs::JointPositions>("robot/limb/left/command_joint_angles", 3, true);
	}
	else
	{
		pub = n.advertise<baxter_msgs::JointPositions>("robot/limb/right/command_joint_angles", 3, true);
	}
	return pub;
}
ros::Publisher DrogonControlInterface::getVelocityPublisher(int arm, ros::NodeHandle& n)
{
	ros::Publisher pub;
	if (arm == LEFT)
	{
		pub = n.advertise<baxter_msgs::JointVelocities>("robot/limb/left/command_joint_velocites", 3, true);
	}
	else
	{
		pub = n.advertise<baxter_msgs::JointVelocities>("robot/limb/right/command_joint_velocities", 3, true);
	}
	return pub;
}
void DrogonControlInterface::setupRobotModel()
{
	loader = robot_model_loader::RobotModelLoader("robot_description");
	model = loader.getModel();
	robot_state::RobotStatePtr tempState(new robot_state::RobotState(model));
	state = tempState;
	state->setToDefaultValues();
	leftArmGroup = model->getJointModelGroup("left_arm");
	rightArmGroup = model->getJointModelGroup("right_arm");
	setupMoveGroups();
}
void DrogonControlInterface::rosEnable()
{
	system("rosrun tools enable_robot.py -e");
}
void DrogonControlInterface::rosDisable()
{
	system("rosrun tools enable_robot.py -d");
}
bool DrogonControlInterface::getIKSolution (int arm, geometry_msgs::Pose input, map<string, double> &output)
{
	Eigen::Affine3d endpoint;
	tf::poseMsgToEigen(input, endpoint);
	printf("armgroup set\n");
	bool found_ik;
	if (arm == LEFT) {
		found_ik = state->setFromIK(leftArmGroup ,endpoint, 10, 0.1);
	} else {
		found_ik = state->setFromIK(rightArmGroup, endpoint, 10, 0.1);
	}
	if (found_ik) {
		printf("found ik solution\n");
		vector<double> joint_values;
		if (arm == LEFT) {
			state->copyJointGroupPositions(leftArmGroup, joint_values);
		} else {
			state->copyJointGroupPositions(rightArmGroup, joint_values);
		}
		printf("got variable values\n");
		for (int i=0; i < JOINTS; ++i) {
			output[jointNames[i]] = joint_values[i];
		}
	}
	return found_ik;
}
bool DrogonControlInterface::getRSDKIKSolution (int arm, geometry_msgs::Pose pose, map<string, double> &out)
{
	geometry_msgs::PoseStamped stamped;
	std_msgs::Header hdr;
	hdr.frame_id = "base";
	hdr.stamp = ros::Time::now();
	stamped.pose = pose;
	stamped.header = hdr;
	baxter_msgs::SolvePositionIK srv;
	srv.request.pose_stamp.push_back(stamped);
	if (arm == LEFT)
	{
//		cout << "client : " << ikLeftClient << endl;
		cout << "srv.request: " << srv.request.pose_stamp[0] << endl;
		cout  << "joints size  before : " << srv.response.joints.size() << endl;
		if (ikLeftClient.call(srv))
		{
			//if( srv.response.joints[0].angles.size() == 0)
			//	return false;
			cout  << "joints size : " << srv.response.joints.size() << endl;
			cout  << "angles: " << srv.response.joints[0].angles.size() << endl;
			cout << "names: " << srv.response.joints[0].names.size() << endl;
			baxter_msgs::JointPositions solution = srv.response.joints[0];
			for (unsigned i=0; i<7; i++)
			{
				std::cout << "angles size : " << solution.angles.size() << std::endl;
				out[srv.response.joints[0].names[i]] = srv.response.joints[0].angles[i];
				//out[jointNames[i]] = solution.angles.back();
				//solution.angles.pop_back();
			}
			return true;
		}
		else
		{
			cout << "error in call to IK service" << endl;
			return false;
		}
	}
	else
	{
			if (ikRightClient.call(srv))
		{
			baxter_msgs::JointPositions solution = srv.response.joints.back();
			for (unsigned i=0; i<7; i++)
			{
				out[jointNames[i]] = solution.angles.back();
				solution.angles.pop_back();
			}
			return true;
		}
		else
		{
			cout << "error in call to IK service" << endl;
			return false;
		}
	}
}
// This method call must be preceded by calls to rosEnable() and setVelocityMode(arm).
// @param: velocities must contain an element for each of the seven joints.  0 is to be used if the
// joint is not meant to move, but the joint may not be left out.
// @param: time is the length of time in microseconds the servos should run at these velocities.
// @param: arm is an integer representing the arm to be moved.  Use the constant LEFT or RIGHT.
void DrogonControlInterface::setJointVelocities(int arm, map<string, double> velocities, unsigned int time)
{
	ros::Publisher *pub;
	if (arm == LEFT)
		pub = &leftVelocityPub;
	else
		pub = &rightVelocityPub;
	baxter_msgs::JointVelocities msg;
	for (unsigned int i=0; i<velocities.size(); i++)
	{
		msg.velocities.push_back(velocities[jointNames[i]]);
		msg.names.push_back(jointNames[i]);
	}
	pub->publish(msg);
	usleep(time);
	baxter_msgs::JointVelocities stop;
	for (unsigned int i=0; i<velocities.size(); i++)
	{
		stop.velocities.push_back(0);
		stop.names.push_back(jointNames[i]);
	}
	pub->publish(stop);
}
void DrogonControlInterface::setJointPosition(int arm, map<string, double> goal)
{
	verifyGoalLimits(goal);
	baxter_msgs::JointPositions msg;
//	ros::init(0, NULL, "drogon_interface");
	ros::Publisher *pub;
	State *state;
	if (arm == LEFT)
	{
		pub = &leftArmPub;
		state = &leftState;
	}
	else
	{
		pub = &rightArmPub;
		state = &rightState;
	}
	for (int i=0; i<7; ++i)
	{
//		cout << i << ") " << jointNames[i] << endl;
		msg.names.push_back(jointNames[i]);
//		cout << " msg name :" << msg.names[i] << endl;
		msg.angles.push_back(goal[jointNames[i]]);
//		cout << " msg angle : " << msg.angles[i] << endl;
	}
//	std::cout << " done with names! " << endl;
	while (!closeEnough(goal, state->data))
//	for (int i=0; i<30; i++)
	{
//		cout << "in while loop" << endl;
//		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
		pub->publish(msg);
		usleep(33000);
	}
}
geometry_msgs::Pose DrogonControlInterface::getPose(int arm)
{
	if (arm == LEFT)
	{
		return leftPosition.pose;
	}
	else
	{
		return rightPosition.pose;
	}
}
;
Position DrogonControlInterface::getPosition(int arm)
{
	if (arm == LEFT)
	{
		return leftPosition;
	}
	else
	{
		return rightPosition;
	}
}
void DrogonControlInterface::gripperAction (int arm, int direction)
{
//	ros::init(0, NULL, "drogon_interface");
	ros::Publisher *pub;
	if (arm == LEFT)
	{
		pub = &leftGripPub;
	}
	else
	{
		pub = &rightGripPub;
	}
	baxter_msgs::GripperCommand cmd;
	cmd.force=30.0;
	cmd.velocity=100.0;
	cmd.holding=0.0;
	cmd.deadZone=3.0;
	if (direction == CLOSE)
	{
		cmd.position=0.0;
	}
	else
	{
		cmd.position=100.0;
	}
	pub->publish(cmd);
}
void DrogonControlInterface::calibrate (int arm)
{
//	ros::init(0, NULL, "drogon_interface");
	std_msgs::Bool msg;
	msg.data=true;
	if (arm == LEFT)
	{
		ros::Publisher pub = n.advertise<std_msgs::Bool>("robot/limb/left/accessory/gripper/set_enabled", 1);
		pub.publish(msg);
		leftCalibratePub.publish(std_msgs::Empty());
	}
	else
	{
		ros::Publisher pub = n.advertise<std_msgs::Bool>("robot/limb/right/accessory/gripper/set_enabled", 1);
		pub.publish(msg);
		rightCalibratePub.publish(std_msgs::Empty());
	}
}
void DrogonControlInterface::setPositionMode(int arm)
{
//	ros::init(0, NULL, "drogon_interface");
	baxter_msgs::JointCommandMode msg;
	msg.mode = baxter_msgs::JointCommandMode::POSITION;
	ros::Publisher pub;
	if (arm == LEFT)
	{
		pub = n.advertise<baxter_msgs::JointCommandMode>("robot/limb/left/joint_command_mode", 3, true);
	}
	else
	{
		pub = n.advertise<baxter_msgs::JointCommandMode>("robot/limb/right/joint_command_mode", 3, true);
	pub.publish(msg);
	}
}
void DrogonControlInterface::setVelocityMode(int arm)
{
//	ros::init(0, NULL, "drogon_interface");
	baxter_msgs::JointCommandMode msg;
	msg.mode = baxter_msgs::JointCommandMode::VELOCITY;
	ros::Publisher pub;
	if (arm == LEFT)
	{
		pub = n.advertise<baxter_msgs::JointCommandMode>("robot/limb/left/joint_command_mode", 3, true);
	}
	else
	{
		pub = n.advertise<baxter_msgs::JointCommandMode>("robot/limb/right/joint_command_mode", 3, true);
	}
	pub.publish(msg);
}
map<string, double> DrogonControlInterface::getJointStates(int arm)
{
	map<string, double> current;
	State *state;
	if (arm == LEFT)
	{
		state = &leftState;
	}
	else
	{
		state = &rightState;
	}
//	ros::spinOnce();
	for (int i=0; i<7; i++)
	{
		current[jointNames[i]] = state->data[jointNames[i]];
//	cout << jointNames[i] << ":  " << current[jointNames[i]] << endl;
	}
	return current;
}
void DrogonControlInterface::verifyGoalLimits(map<string, double> &goal)
{
	for (int i=0; i<7; i++)
	{
		if (goal[jointNames[i]] > limits.jointAngleUpperLimits[jointNames[i]])
		{
			goal[jointNames[i]] = limits.jointAngleUpperLimits[jointNames[i]];
		}
		else if (goal[jointNames[i]] < limits.jointAngleLowerLimits[jointNames[i]])
		{
			goal[jointNames[i]] = limits.jointAngleLowerLimits[jointNames[i]];
		}
	}		
}
bool DrogonControlInterface::closeEnough(map<string, double> &goal, map<string, double> &state)
{
//	ros::spinOnce();
//	cout << "begin closeEnough" << endl;
//	cout << "goal: " << goal << endl;
//	cout << "state: " << state << endl;
	for (int i=0; i<7; i++)
	{
//		cout << jointNames[i] << ": " << state->at(jointNames[i]) << endl;
//		cout << jointNames[i] << ": " << goal->at(jointNames[i]) << endl;
		if (goal[jointNames[i]] - state[jointNames[i]] > .025 || goal[jointNames[i]] - state[jointNames[i]] < -.025)
		{
//			cout << "closeEnough: false" << endl;
			return false;
		}
	}
//	cout << "closeEnough: true" << endl;
	return true;
}
void DrogonControlInterface::setupMoveGroups() {
	move_group_interface::MoveGroup temp("left_arm");
	leftArmPlanner = &temp;
	move_group_interface::MoveGroup temp1("right_arm");
	rightArmPlanner = &temp1;
}
moveit::planning_interface::MoveGroup::Plan DrogonControlInterface::getPlan(const map<string, double> &goal, const int arm) {
	moveit::planning_interface::MoveGroup::Plan plan;
	if (arm == LEFT) {
		leftArmPlanner->setStartStateToCurrentState();
		leftArmPlanner->setJointValueTarget(goal);
		leftArmPlanner->plan(plan);
	} else {
		rightArmPlanner->setStartStateToCurrentState();
		rightArmPlanner->setJointValueTarget(goal);
		rightArmPlanner->plan(plan);
	}
	return plan;
}
moveit::planning_interface::MoveGroup::Plan DrogonControlInterface::getPlan(const map<string, double> &goal, const map<string, double> &start, const int arm) {
}
void DrogonControlInterface::executePlan(const moveit::planning_interface::MoveGroup::Plan &plan, const int arm) {
	if (arm == LEFT) {
		leftArmPlanner->execute(plan);
	} else {
		rightArmPlanner->execute(plan);
	}
}
