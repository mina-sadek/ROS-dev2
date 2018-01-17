/*
 * publisher.cpp
 *
 *  Created on: Apr 29, 2017
 *      Author: tornado
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <stdlib.h>

int main(int argc, char **argv) {
	// Initialize the ROS node, and assign the node a name
	ros::init(argc, argv, "publisher");
	// Create a node handler
	ros::NodeHandle n;

	// Ceates the publisher, and assign it to publish
	// to the /diffBot/cmd_vel topic, with a queue size of 10
	ros::Publisher pub=n.advertise<geometry_msgs::Twist>("diffBot/cmd_vel", 10);

	// Sets up the random number generator
	srand(time(0));

	// Sets the loop to publish at a rate of 10Hz
	ros::Rate rate(10);

	while(ros::ok()) {
		// Declares the message to be sent
		geometry_msgs::Twist msg;

		// Random x value between -2 and 2
		msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
		// Random y value between -3 and 3
		msg.angular.z=6*double(rand())/double(RAND_MAX)-3;

		// Publish the message
		pub.publish(msg);

		// Delays untill it is time to send another message
		rate.sleep();
	}
}


