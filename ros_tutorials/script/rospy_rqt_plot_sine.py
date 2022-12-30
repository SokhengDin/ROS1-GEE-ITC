#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math

if __name__=="__main__":
    rospy.init_node("sinus_publisher")
    publish = rospy.Publisher("/sinus", Float64, queue_size=10)

    x = 0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        x += 0.1
        publish.publish(math.sin(x)*10)
        rate.sleep()