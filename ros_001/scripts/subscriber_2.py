#!/usr/bin/env python
#import roslib; roslib.load_manifest('YOUR_PACKAGE_NAME_HERE')
import rospy
#import tf.transformations
from geometry_msgs.msg import Twist

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands

    #v_l = ...
    #v_r = ...

    # Then set your wheel speeds (using wheel_left and wheel_right as examples)
    #wheel_left.set_speed(v_l)
    #wheel_right.set_speed(v_r)

def subscriber():
    # subscriber: node name, tells rospy the name of your node -- 
    # until rospy has this information, it cannot start communicating with the ROS Master.
    # node name must not include any "/" characters
    
    # (anonymous = True): In ROS, nodes are uniquely named.
    # If two nodes with the same name are launched, the previous one is kicked off.
    # The anonymous=True flag means that rospy will choose a unique name for our 'subscriber' node,
    # so that multiple listeners can run simultaneously.
    rospy.init_node('subscriber', anonymous = True)
    
    # /husky_velocity_controller/cmd_vel: Define name of the topic that the node will subscribe to
    # Twist: is type of the topic
    # callback: is the function to be called when a message is successfully received from that topic 
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()
    
    
    
