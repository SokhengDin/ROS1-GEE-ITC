#include "ros/ros.h"
#include "ros_tutorials/add_two_ints.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "client_node");
    if (argc != 3)
    {
        ROS_INFO("usage: add_two_ints_client X Y");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<ros_tutorials::add_two_ints>("add_two_ints");

    ros_tutorials::add_two_ints srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if (client.call(srv))
    {
        ROS_INFO("Sum : %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_INFO("Failed to call service add_two_ints");
        return 1;
    }
    return 0;
}