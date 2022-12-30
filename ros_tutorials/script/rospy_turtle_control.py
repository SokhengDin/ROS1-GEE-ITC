#!/usr/bin/env python3

import rospy
import numpy as np
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class turtle():
    def __init__(self):
        rospy.init_node('turtle', anonymous=True)
        self.rate = rospy.Rate(10)
        self.velocity_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.turtle_pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.callback_pose)
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            print("ROS master is not available")
        self.rate.sleep()

    def callback_pose(self, msg_pose):
        msg_twist = Twist()
        if msg_pose.x > 10 or msg_pose.x < 1 or msg_pose.y > 10 or msg_pose.y < 1:
            msg_twist.linear.x = np.random.uniform(low=1, high=2.5)
            msg_twist.angular.z = np.random.uniform(low=0.8, high=3)
        else:
            msg_twist.linear.x = 2
            msg_twist.angular.z = 0.0
        self.velocity_pub.publish(msg_twist)

if __name__=="__main__":
    while not rospy.is_shutdown():
        turtle()