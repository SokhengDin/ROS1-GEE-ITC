#!/usr/bin/env python


import rospy
from std_msgs.msg import String

def callback(msg):
	rospy.loginfo("I saw %s", msg.data)


def ros_subscriber():
	rospy.init_node('Subscriber', anonymous=True)
	rospy.Subscriber('chatter', String, callback)
	
	rospy.spin()
	
if __name__=="__main__":
	ros_subscriber()
