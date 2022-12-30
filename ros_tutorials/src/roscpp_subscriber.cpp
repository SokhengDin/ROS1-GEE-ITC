#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)// ros_tutorials::Rectangle::ConstPtr& msg, msg->area;
{
	ROS_INFO("I see: [%s]", msg->data.c_str());
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle nh;
	
	ros::Subscriber subscribe = nh.subscribe("chatter", 1000, chatterCallback);
	
	ros::spin(); // to catch data callback
	
	return 0;
}
