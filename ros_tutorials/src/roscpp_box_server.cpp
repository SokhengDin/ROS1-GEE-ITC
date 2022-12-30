#include "ros/ros.h"
#include "ros_tutorials/rect_volume.h"

bool volume(ros_tutorials::rect_volume::Request &request,
            ros_tutorials::rect_volume::Response &response)
{
    response.volume = request.length * request.width * request.height;
    ROS_INFO("Request: length=%f, width=%f, height=%f", (float)request.length, (float)request.width, (float)request.height);
    ROS_INFO("Sending back response: [%f]" , (float)response.volume);
    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "service_node");
    ros::NodeHandle nh;
    
    ros::ServiceServer service = nh.advertiseService("calculate_volume", volume);
    ROS_INFO("Ready to calculate volume.");
    ros::spin();
    return 0;
}
