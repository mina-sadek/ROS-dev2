/*
 * publisher.cpp
 *
 *  Created on: Apr 29, 2017
 *      Author: tornado
 */

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>
#include <stdio.h>

//#include <string.h>
#include <wiringPi.h>

//#define ENC_PIN 23 // right wheel encoder
#define ENC_PIN 24 // left wheel encoder

void tick_int(void);

// the tick counter 
volatile int tick_count = 0;

// Called every time a tick occurs
void tick_int(void) {
   tick_count++;
}

int main(int argc, char **argv) {

        // sets up the wiringPi library
        if (wiringPiSetupGpio () < 0) {
                fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
                return 1;
        }
        // set interrupt on high-to-low transitions
        // and attach tick_int() to the interrupt
        if ( wiringPiISR (ENC_PIN, INT_EDGE_FALLING, &tick_int) < 0 ) {
                fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
                return 1;
        }

        //Initializes ROS, and sets up a node
        ros::init(argc, argv, "lwheel");
        ros::NodeHandle nh;

        //Ceates the publisher, and tells it to publish
        //to the robot/cmd_vel topic, with a queue size of 100
        ros::Publisher pub=nh.advertise<std_msgs::Int16>("diffBot/lwheel", 10);

        //Sets the loop to publish at a rate of 10Hz
        //ros::Rate rate(10);
        ros::Rate rate(1);      // 1Hz

        //Declares the message to be sent
        //geometry_msgs::Twist msg;
        std_msgs::Int16 msg;

        while(ros::ok()) {
                msg.data = tick_count;
                // tick_count = 0;

                //Publish the message
                pub.publish(msg);

                // Handle ROS events
                // ros::spinOnce() will call all the callbacks waiting to be called at that point of time
                ros::spinOnce();

                //Delays untill it is time to send another message
                rate.sleep();
        }
}

