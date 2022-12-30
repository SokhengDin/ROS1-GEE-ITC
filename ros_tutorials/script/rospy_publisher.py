#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def ros_talker():
	rospy.init_node('publisher', anonymous=True) ## initialized node
	publisher = rospy.Publisher('chatter', String, queue_size=10) ## create publisher with topic name 'chatter'
	rate = rospy.Rate(10) ## frequency
	counter = 0
	while not rospy.is_shutdown():
		hello_str = "Hello world ! %s" % counter
		rospy.loginfo(hello_str)
		publisher.publish(hello_str)
		counter += 1
		rate.sleep()

if __name__=="__main__":
	try:
		ros_talker()
	except KeyboardInterrupt:
		pass
	
