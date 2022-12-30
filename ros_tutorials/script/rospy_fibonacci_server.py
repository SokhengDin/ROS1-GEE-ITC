#!/usr/bin/env python
import rospy
import actionlib
import ros_tutorials.msg

class FibonacciAction(object):
    feedback = ros_tutorials.msg.FibonacciFeedback()
    result = ros_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        self.action_name = name
        self.action_simple = actionlib.SimpleActionServer(self.action_name, ros_tutorials.msg.FibonacciAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_simple.start()

    def execute_cb(self, goal):
        rate = rospy.Rate(10)
        success = True

        self.feedback.sequence = []
        self.feedback.sequence.append(0)
        self.feedback.sequence.append(1)

        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self.action_name, goal.order,
                      self.feedback.sequence[0], self.feedback.sequence[1]))
        
        for i in range(1, goal.order):
            if self.action_simple.is_preempt_requested():
                rospy.loginfo('%s: Preempt' % self.action_name)
                self.action_simple.set_preempted()
                success = False
                break
            self.feedback.sequence.append(self.feedback.sequence[i]+self.feedback.sequence[i-1])
            self.action_name.publish_feedback(self.feedback)

            rate.sleep()

        if success:
            self.result.sequence = self.feedback.sequence
            rospy.loginfo('%s: Succeeded' % self.action_name)
            self.action_simple.set_succeeded(self.result)

if __name__=="__main__":
    rospy.init_node("fibonacci_server")
    server = FibonacciAction(rospy.get_name())
    rospy.spin()