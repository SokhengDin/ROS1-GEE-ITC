#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "tf2_listener");
    ros::NodeHandle nh;

    ros::service::waitForService("spawn");
    ros::ServiceClient spawner = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn turtle;
    turtle.request.x = 4;
    turtle.request.y = 2;
    turtle.request.theta = 0;
    turtle.request.name = "turtle2";
    spawner.call(turtle);

    ros::Publisher turtle_vel = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener(buffer);

    ros::Rate rate(10);
    while (nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = buffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
        }

        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
        vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2)+pow(transformStamped.transform.translation.y, 2));
        turtle_vel.publish(vel_msg);
        rate.sleep();
    }
    return 0;
}