#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose

class Publisher():
    def __init__(self):
        ## Instance attribute
        rospy.init_node('Publisher', anonymous=True) ## Initialize node
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10) ## Instance object for velocity publisher
        self.pose_publisher = rospy.Publisher('Pose', Pose, queue_size=10) ## Instance object for pose publisher
        self.rate = rospy.Rate(10) ## Rate loop 10Hz, T=1/f=1/10=0.1s
        self.generate_velocity() ## Function generate velocity message to publish
        self.generate_pose() ## Function generate pose message to publish
        self.rate.sleep() ## Sleep publishing at rate
    def generate_velocity(self):
        twist = Twist()
        twist.linear.x = np.random.randn()
        twist.linear.y = np.random.randn()
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = np.random.randn()
        self.velocity_publisher.publish(twist)

    def generate_pose(self):
        pose = Pose()
        pose.position.x = np.random.uniform()
        pose.position.y = np.random.uniform()
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = np.random.uniform() ## Yaw
        pose.orientation.w = 0.0
        self.pose_publisher.publish(pose)

if __name__=="__main__":
    while not rospy.is_shutdown():
        Publisher()