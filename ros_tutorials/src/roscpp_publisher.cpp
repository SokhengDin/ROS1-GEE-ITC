#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker"); // Check if roscore is running
	ros::NodeHandle nh; // Create ros node
	ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000); // Create node publisher
	
	ros::Rate loop_rate(10); // frequency rate in ros
	
	int counter = 0; // variable integer
	
	while (ros::ok()) // check if ros is running
	{
		std_msgs::String msg; // create ros message type String
		std::stringstream ss; // Create stringstream variable
		ss << "Hello world!" << counter;
		msg.data  = ss.str(); // insert stringstream data into ros message
		ROS_INFO("%s", msg.data.c_str());
		pub.publish(msg); // Publisher ros message
		
		ros::spinOnce(); // to catch up callback function
		loop_rate.sleep(); // ros sleep
		counter++; // count up loop
	}
	return 0;
}
