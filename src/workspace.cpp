/* 
 * File:   iktest.cpp
 * Author: Isaac Yochelson
 *
 * Created on November 13, 2013, 10:48 AM
 */
#include <cstdlib>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

using namespace std;

int main(int argc, char** argv) {
	ros::init(argc, argv, "collision_object_publisher_node");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle;
	ros::Publisher object_pub = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1, false);
	while (object_pub.getNumSubscribers() < 1) {
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
	}
	shape_msgs::SolidPrimitive table;
	table.type = table.BOX;
	table.dimensions.push_back(0.8);
	table.dimensions.push_back(3.0);
	table.dimensions.push_back(0.05);
	geometry_msgs::Pose pose;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 0;
	pose.position.x = 0.82;
	pose.position.y = 0;
	pose.position.z = -0.3;
	shape_msgs::SolidPrimitive cup;
	cup.type = cup.CYLINDER;
	cup.dimensions.push_back(0.25);
	cup.dimensions.push_back(0.094);
	moveit_msgs::CollisionObject ws;
	ws.id = "workspace";
	ws.header.frame_id = "odom_combined";
	ws.primitives.push_back(table);
	ws.primitive_poses.push_back(pose);
	pose.position.z = -0.15;
	for (int count = -7; count < 8; ++count) {
		pose.position.y = 0.104 * count;
		ws.primitives.push_back(cup);
		ws.primitive_poses.push_back(pose);
	}
	ws.header.stamp = ros::Time::now();
	ws.operation = ws.ADD;
	object_pub.publish(ws);
}
