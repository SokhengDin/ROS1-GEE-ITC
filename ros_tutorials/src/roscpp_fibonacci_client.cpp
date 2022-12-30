#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "ros_tutorials/FibonacciAction.h"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "fibonacci_client");

    actionlib::SimpleActionClient<ros_tutorials::FibonacciAction> action_client("fibonacci", true);
    ROS_INFO("Waiting for action server to start");
    action_client.waitForServer();
    ROS_INFO("Action server started, sending goal");

    ros_tutorials::FibonacciGoal goal;
    goal.order = 20;
    action_client.sendGoal(goal); 

    bool finshied_before_timeout = action_client.waitForResult(ros::Duration(30.0));
    if (finshied_before_timeout)
    {
        actionlib::SimpleClientGoalState state = action_client.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out");

    }
    return 0;
}