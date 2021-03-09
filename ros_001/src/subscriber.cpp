/*
 * subscriber.cpp
 *
 *  Created on: Apr 29, 2017
 *      Author: tornado
 */

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

// Topic messages callback
void velCallback(const geometry_msgs::Twist msg)
{
	ROS_INFO("[Subscriber] received:\n[x=%f]\n[y=%f]\n[z=%f]\n-------\n", msg.linear.x, msg.linear.y, msg.linear.z);
}

int main(int argc, char **argv)
{
	// Initiate new ROS node named "subscriber"
	ros::init(argc, argv, "subscriber");
	// Create a node handle: it is reference assigned to a new node
	ros::NodeHandle n;

	// subscribe to a given topic, in this case "diffBot/cmd_vel"
	// velCallback: is the name of the callback function that will be executed each time a message is reveived
	ros::Subscriber sub = n.subscribe("diffBot/cmd_vel", 10, velCallback);

	// Enter a loop, pumping callbacks
	ros::spin();

	return 0;
}
