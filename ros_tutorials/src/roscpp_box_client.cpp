#include "ros/ros.h"
#include "ros_tutorials/rect_volume.h"

int main(int argc, char ** argv)
{
   ros::init(argc, argv, "client_node");
   if (argc != 4)
   {
      ROS_INFO("usage: calculate rectangle volume length width and height");
      return 1;
    }
   ros::NodeHandle nh;
   ros::ServiceClient client = nh.serviceClient<ros_tutorials::rect_volume>("calculate_volume");
   ros_tutorials::rect_volume srv;
   srv.request.length = atoll(argv[1]);
   srv.request.width = atoll(argv[2]);
   srv.request.height = atoll(argv[3]);
   if (client.call(srv))
   {
      ROS_INFO("Volume: %f", (float)srv.response.volume);
   }
   else
   {
      ROS_ERROR("Falied to call service to calculate volume");
      return 1;
   }
   return 0;
}
