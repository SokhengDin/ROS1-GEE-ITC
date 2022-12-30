#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ros_tutorials/AveragingAction.h>
#include <boost/thread.hpp>

void spinThread()
{
    ros::spin();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "statistic_client");
    actionlib::SimpleActionClient<ros_tutorials::AveragingAction> ac("statistic");
    boost::thread spin_thread(&spinThread);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal");

    ros_tutorials::AveragingGoal goal;
    goal.samples = 100;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finished before the time out.");
    }
    ros::shutdown();
    spin_thread.join();

    return 0;
}