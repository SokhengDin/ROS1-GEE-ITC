#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <tf/transform_broadcaster.h>


std::string turtlename;


void poseCallback(const turtlesim::PoseConstPtr &msg)
{
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, msg->theta);
    transform.setRotation(quaternion);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtlename));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_broadcaster");
    if (argc !=2)
    {
        ROS_ERROR("need turtle name for argument");
        return -1;
    }
    turtlename = argv[1];
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(turtlename+"/pose", 10, &poseCallback);
    ros::spin();
    return 0;
}