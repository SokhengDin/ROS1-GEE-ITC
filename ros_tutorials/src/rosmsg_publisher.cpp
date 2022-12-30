#include <ros/ros.h>
#include <ros_tutorials/Rectangle.h>


int main(int argc, char ** argv)
{	
	float length = 3;
	float width = 6;
	ros::init(argc, argv, "Publisher");
	ros::NodeHandle nh;
	ros::Publisher publisher = nh.advertise<ros_tutorials::Rectangle>("Area_rectangle", 10); // Publisher with topic name "Area rectangle"
	ros::Rate loop_rate(10);
	ros_tutorials::Rectangle rect1;
	while (ros::ok())
	{
		rect1.name = "Rectangle1";
		rect1.stamp = ros::Time::now();
		rect1.width = width;
		rect1.length = length;
		rect1.area = width * length;
		publisher.publish(rect1);
		ROS_INFO("Area of rectangle: %f", rect1.area);
		loop_rate.sleep();
	}
	return 0;
}	
