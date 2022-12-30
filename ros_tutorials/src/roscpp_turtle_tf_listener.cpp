#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle nh;
    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle2 = nh.serviceClient<turtlesim::Spawn>("spawn");
    ros::ServiceClient add_turtle3 = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn turtle2;
    turtle2.request.name = "turtle2";
    turtle2.request.x = 4;
    turtle2.request.y = 2;
    turtle2.request.theta = 0;
    //turtlesim::Spawn turtle3;
    //turtle3.request.name = "turtle3";
    //turtle3.request.x = 3;
    //turtle3.request.y = 3;
    //turtle3.request.theta = 0;
    add_turtle2.call(turtle2);
    //add_turtle3.call(turtle3);

    ros::Publisher turtle_vel2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    //ros::Publisher turtle_vel3 = nh.advertise<geometry_msgs::Twist>("turtle3/cmd_vel", 10);

    tf::TransformListener listener1;
    //tf::TransformListener listener2;

    ros::Rate rate(10);
    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener1.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
            //listener2.lookupTransform("/turtle3", "/turtle2", ros::Time(0), transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
        turtle_vel2.publish(vel_msg);
        //turtle_vel3.publish(vel_msg);

        rate.sleep();
    }
    return 0;
}