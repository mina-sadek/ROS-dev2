#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32
from std_msgs.msg import Int16

import wiringpi

ENC_PIN = 23	# right wheel encoder
#ENC_PIN = 24	# left wheel encoder

class diffBot_rwheel():
    #tick_count = 1
    ###############################################
    def __init__(self):
    ###############################################
	wiringpi.wiringPiSetupGpio()   # initialise wiringpi
	wiringpi.wiringPiISR(ENC_PIN, wiringpi.INT_EDGE_FALLING, self.tick_int)
        rospy.init_node("diffBot_rwheel", anonymous = True);
#        self.nodename = rospy.get_name()
#        rospy.loginfo("%s started" % self.nodename)
        
        self.tick_count = 0
	self.pub_wheel = rospy.Publisher('diffBot/rwheel', Int16, queue_size=10) #1)
	rospy.init_node('diffBot_rwheel', anonymous = True)
	self.rate = rospy.Rate(1) # 1hz

    def tick_int(self):
	self.tick_count = 5

    ###############################################
    def spin(self):
    ###############################################
	while not rospy.is_shutdown():
		self.spinOnce()

		# Sleeps @ rate "1 loop/second"
		self.rate.sleep()

    ###############################################
    def spinOnce(self):
    ###############################################
	self.msg = self.tick_count
	#self.tick_count = 0
	# Perform the publishing action 
        self.pub_wheel.publish(self.msg)

################################################
if __name__ == '__main__':
    try:
	    rwheel = diffBot_rwheel()
	    rwheel.spin()
    except rospy.ROSInterruptException:
        pass 
