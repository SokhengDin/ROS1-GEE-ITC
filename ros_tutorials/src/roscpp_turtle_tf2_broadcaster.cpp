#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

std::string turtle_name;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "static_broadcaster");
    if (argc != 8)
    {
        ROS_ERROR("Invalid number of parameters\nusage: static_turtle_tf2_broadcaster child_turtle_name x y z roll pitch yaw");
        return -1;
    }
    if (strcmp(argv[1], "world")==0)
    {
        ROS_ERROR("Your turtlename cannot be 'world'");
        return -1;
    }

    turtle_name = argv[1];
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformstamped;

    static_transformstamped.header.stamp = ros::Time::now();
    static_transformstamped.header.frame_id = "world";
    static_transformstamped.child_frame_id = turtle_name;
    static_transformstamped.transform.translation.x = atof(argv[2]);
    static_transformstamped.transform.translation.y = atof(argv[3]);
    static_transformstamped.transform.translation.z = atof(argv[4]);
    tf2::Quaternion quaternion;
    quaternion.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
    static_transformstamped.transform.rotation.x = quaternion.x();
    static_transformstamped.transform.rotation.y = quaternion.y();
    static_transformstamped.transform.rotation.z = quaternion.z();
    static_transformstamped.transform.rotation.w = quaternion.w();
    static_broadcaster.sendTransform(static_transformstamped);
    ROS_INFO("Spinning unill killed publishing %s to world", turtle_name.c_str());
    ros::spin();
    return 0;
}