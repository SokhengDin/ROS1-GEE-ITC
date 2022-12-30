#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        transform.setOrigin(tf::Vector3(0.0, 2.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "turtle3"));
        rate.sleep();
    }

    return 0;
}