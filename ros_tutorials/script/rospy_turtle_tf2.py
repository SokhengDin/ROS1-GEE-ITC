#!/usr/bin/env python3
import rospy 
import sys
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler


if __name__=="__main__":
    if len(sys.argv) < 8:
        rospy.logerr('Invalid number of parameters\n usage: ' 'child_frame_name x y z roll pitch yaw')
        sys.exit(0)
    else:
        if sys.argv[1] == 'world':
            rospy.logerr('Your static turtle name cannot be "world"')
            sys.exit(0)
        rospy.init_node("static_tf2_broadcaster")
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformstamped = geometry_msgs.msg.TransformStamped()

        static_transformstamped.header.stamp = rospy.Time.now()
        static_transformstamped.header.frame_id = "world"
        static_transformstamped.child_frame_id = sys.argv[1]

        static_transformstamped.transform.translation.x = float(sys.argv[2])
        static_transformstamped.transform.translation.y = float(sys.argv[3])
        static_transformstamped.transform.translation.z = float(sys.argv[4])

        quaternion = quaternion_from_euler(float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]))
        static_transformstamped.transform.rotation.x = quaternion[0]
        static_transformstamped.transform.rotation.y = quaternion[1]
        static_transformstamped.transform.rotation.z = quaternion[2]
        static_transformstamped.transform.rotation.w = quaternion[3]

        broadcaster.sendTransform(static_transformstamped)
        rospy.spin()