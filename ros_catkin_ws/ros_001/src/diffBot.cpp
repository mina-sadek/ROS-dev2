
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#include "diffBotPi/diffBotPi.h"
#define PWM_MIN 500
#define PWM_STEP 10
#define SLEEP_TIME 0.1

using namespace std;

class diffBot
{
 public:
	// Constructor
	diffBot();
	// Destructor
	~diffBot();
	// Main node loop.
	void run();
private:
	// The node handle
	ros::NodeHandle nodeHandle;

	// A publisher member
	// ros::Publisher mPub;

	// A Subscriber member
	ros::Subscriber velocitySub;

	// A Service Server member
	//ros::ServiceServer mServServer;

	// Subscriber callback to the velocity topic.
	// @param vel The new velocity for the diffBot
	void velocitySubCallback(const geometry_msgs::Twist::ConstPtr &vel);
};

diffBot::diffBot()
{
	if (robot_init() <= -1)
	{
		//return 1;
	}

	// Register subscriber and services
	this->velocitySub = this->nodeHandle.subscribe("diffBot/cmd_vel", 1, &diffBot::velocitySubCallback, this);
}

diffBot::~diffBot()
{
	robot_exit();
}

void diffBot::run()
{
	// Sleep till new messages arrives
	ros::spin();
}

void diffBot::velocitySubCallback(const geometry_msgs::Twist::ConstPtr &vel)
{
	ROS_INFO("[Subscriber] received:\n[x=%f]\n[y=%f]\n[z=%f]\n-------\n", vel->linear.x, vel->linear.y, vel->angular.z);
	// Check if direction is set correctly
	if (vel->linear.x > 0)
	{
		int speed = vel->linear.x;
		// move forward
		printf("robot moving FORWARD @ speed = %d\n", speed);
		robot_move(FORWARD, speed);
		ros::Duration(SLEEP_TIME).sleep(); // sleep for a second
                robot_stop();
	}
	else if (vel->linear.x < 0)
	{
		int speed = abs(vel->linear.x);
		// move forward
		printf("robot moving BACKWARD @ speed = %d\n", speed);
		robot_move(BACKWARD, speed);
		ros::Duration(SLEEP_TIME).sleep(); // sleep for a second
                robot_stop();
	}
	if (vel->angular.z > 0)
	{
		int speed = vel->angular.z;
		// move right
		printf("robot moving RIGHT @ speed = %d\n", speed);
		robot_move(RIGHT, speed);
		ros::Duration(SLEEP_TIME).sleep(); // sleep for a second
                robot_stop();
	}
	else if (vel->angular.z < 0)
	{
		int speed = abs(vel->angular.z);
		// move right
		printf("robot moving RIGHT @ speed = %d\n", speed);
		robot_move(LEFT, speed);
		ros::Duration(SLEEP_TIME).sleep(); // sleep for a second
                robot_stop();
	}
	else
	{
		// all values are equal to zero, so stop moving
                robot_stop();
	}
}

int main(int argc, char** argv)
{
	// Register node in the ros environment
	ros::init(argc, argv, "diffBot_node");
	diffBot bot;
	bot.run();
}

