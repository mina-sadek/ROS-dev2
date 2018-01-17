#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#include "diffBotPi/diffBotPi.h"
#define PWM_MIN 500
#define PWM_STEP 10


// Topic messages callback
//void velCallback(const std_msgs::String::ConstPtr& msg)
void velCallback(const geometry_msgs::Twist msg)
{
	ROS_INFO("[Subscriber] received:\n[x=%f]\n[y=%f]\n[z=%f]\n-------\n", msg.linear.x, msg.linear.y, msg.linear.z);
}

int main(int argc, char **argv)
{
  int i, pwm_val;
  if (robot_init() <= -1)
    return 1;
  for (i = 0; i < 2; i++)
  {
    pwm_val = PWM_MIN + (PWM_STEP * (i + 1));
    printf("robot runngin @ speed = %d\n", pwm_val);
    robot_move(FORWARD, pwm_val);
    sleep(2);
    robot_stop();
    sleep(1);
    robot_move(BACKWARD, pwm_val);
    sleep(2);
    robot_stop();
    sleep(1);
    robot_move(RIGHT, pwm_val);
    sleep(2);
    robot_stop();
    sleep(1);
    robot_move(LEFT, pwm_val);
    sleep(2);
    robot_stop();
    sleep(1);
  }

  robot_exit();


	//Initiate new ROS node named "subscriber"
	ros::init(argc, argv, "subscriber");
	// Create a node handle: it is reference assigned to a new node
	ros::NodeHandle n;

	// subscribe to a given topic, in this case "robot/cmd_vel"
	// velCallback: is the name of the callback function that will be executed each time a message is reveived
	ros::Subscriber sub = n.subscribe("diffBot/cmd_vel", 1000, velCallback);

	// Enter a loop, pumping callbacks
	ros::spin();

	return 0;
}

