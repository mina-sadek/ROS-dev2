#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#include <stdio.h>
#include <wiringPi.h>
#define LED 18          // BCM
//#define LED 1         // wiringpi
#define STEP 50

int main(int argc, char **argv) {
	int bright;
	printf ("Raspberry Pi blink\n") ;

	if (wiringPiSetupGpio() == -1)        // BCM [GPIO pin numbering]
	//  if (wiringPiSetup() == -1)          // wiringpi pin numbering
		return 1 ;

	//pinMode (LED, OUTPUT) ;         // aka BCM_GPIO pin 17
	pinMode(LED, PWM_OUTPUT);

	//Initializes ROS, and sets up a node
	ros::init(argc, argv, "publisher_commands");
	ros::NodeHandle nh;

	//Ceates the publisher, and tells it to publish
	//to the robot/cmd_vel topic, with a queue size of 100
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("robot/cmd_vel", 100);

	//Sets up the random number generator
	srand(time(0));

	//Sets the loop to publish at a rate of 10Hz
	ros::Rate rate(10);

	while(ros::ok()) {
		bright = (bright + STEP) % 1024;
		pwmWrite(LED, bright);
		delay(1);

		//Declares the message to be sent
		geometry_msgs::Twist msg;
		//Random x value between -2 and 2
		msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
		//Random y value between -3 and 3
		msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
		//Publish the message
		pub.publish(msg);

		//Delays untill it is time to send another message
		rate.sleep();
	}
}



