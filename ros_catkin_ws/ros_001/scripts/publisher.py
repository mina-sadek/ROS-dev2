#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publisher():
    # this part defines the publisher interface
    # husky_velocity_controller/cmd_vel: is the topic name
    # Twist: is the topic type is a Twist message
    # queue_size: limits the amount of queued messages if any subscriber is not receiving them fast enough
    pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist, queue_size=10)

    # publisher: node name, tells rospy the name of your node -- 
    # until rospy has this information, it cannot start communicating with the ROS Master.
    # node name must not include any "/" characters

    # (anonymous = True): In ROS, nodes are uniquely named.
    # If two nodes with the same name are launched, the previous one is kicked off.
    # The anonymous=True flag means that rospy will choose a unique name for our 'subscriber' node,
    # so that multiple nodes can run simultaneously.
    rospy.init_node('publisher', anonymous = True)

    # Creates a Rate object rate.
    # It offers a convenient way for looping at the desired rate, with the help of its method sleep(). 
    # 10: means that, we will go through the loop 10 times per second. (as long as our processing time does not exceed 1/10th of a second!)
    rate = rospy.Rate(10) # 10hz

    # Define an object of the type Twist, preparing for publishing it
    vel = Twist()

    # Standard rospy construct: checking the rospy.is_shutdown() flag and then doing work.
    # You have to check is_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise).
    while not rospy.is_shutdown():
        vel.linear.x = 1;
        vel.angular.z = 1;
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)

        # Perform the publishing action 
        pub.publish(vel)

        # Sleeps just long enough to maintain the desired rate through the loop, here "10 loops/second"
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
