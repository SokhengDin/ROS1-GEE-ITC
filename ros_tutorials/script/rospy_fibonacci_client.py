#!/usr/bin/env python

from __future__ import print_function
from msilib import sequence
import sys

import rospy
import actionlib
import ros_tutorials.msg

def fibonacci_client():

    client = actionlib.SimpleActionClient('fibonacci', ros_tutorials.msg.FibonacciAction)

    client.wait_for_server()

    goal = ros_tutorials.msg.FibonacciGoal(order=20)

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

if __name__=="__main__":
    try:
        rospy.init_node("fibonacci_client")
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderrs)
