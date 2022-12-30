#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose

class Subsciber():
    def __init__(self):
        rospy.init_node('Subscriber', anonymous=True) ## Initialize node
        self.subscriber_velocity = rospy.Subscriber('cmd_vel', Twist, self.callback_velocity) ## Create subscirber for velocity
        self.subscriber_pose = rospy.Subscriber('Pose', Pose, self.callback_pose) ## Create subscriber for pose
        try:
            rospy.spin() ## Spin to catch data from callback function
        except rospy.ROSInterruptException:
            print("Ros is shutting down!")

    def callback_velocity(self, twist):
        rospy.loginfo("Current velocity '%s'", twist)

    def callback_pose(self, pose):
        rospy.loginfo("Current position '%s'", pose)


if __name__=="__main__":
    Subsciber()