#!/usr/bin/env python
from __future__ import print_function
import rospy

from ros_tutorials.srv import add_two_ints, add_two_intsResponse

def handle_add_two_ints(request):
    print("Returning [%s + %s = %s]" % (request.a,  request.b, (request.a+request.b)))
    return add_two_intsResponse(request.a + request.b)

def add_two_ints_server():
    global add_two_ints
    rospy.init_node("service_node", anonymous=True)
    service = rospy.Service('add_two_ints', add_two_ints, handle_add_two_ints)
    print("Ready to add two integers")
    rospy.spin()

if __name__=="__main__":
    add_two_ints_server()