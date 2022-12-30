#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros_tutorials/TurtleActionAction.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "turtle_action_client");

    actionlib::SimpleActionClient<ros_tutorials::TurtleActionAction> ac("turtle_action", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal");

    ros_tutorials::TurtleActionGoal goal;
    goal.edges = 5;
    goal.radius = 1.5;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }

    return 0;
}