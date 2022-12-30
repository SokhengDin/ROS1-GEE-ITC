#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

int main(int argc, char** argv)
{
    double x;
    double y;
    double delta;
    double vx;
    double vy;
    double vdelta=1.57;
    double time;
    double counter = 0.0;
    double DT = 0.1;
    ros::init(argc, argv, "Publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist msg;
    while (ros::ok())
    {
        delta = 1.57*counter;
        x = 16*pow(sin(delta), 3);
        y = 13*cos(delta)-5*cos(2*delta)-2*cos(3*delta)-cos(4*delta);
        vx = 48*pow(sin(delta), 2)*cos(delta)*vdelta;
        vy = -13*sin(delta)*vdelta+10*sin(2*delta)*vdelta+6*sin(3*delta)*vdelta+4*sin(4*delta)*vdelta;
        msg.linear.x = 3.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 2.0;
        pub.publish(msg);
        counter++;
        loop_rate.sleep();
    }
    return 0;
}