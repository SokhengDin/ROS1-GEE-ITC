#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <ros_tutorials/TurtleActionAction.h>


class TurtleAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<ros_tutorials::TurtleActionAction> as_;
        std::string action_name_;
        double radius_, apothem_, interior_angle_, side_len_;
        double start_x_, start_y_, start_theta_;
        double dis_error_, theta_error_;
        int edges_, edge_progress_;
        bool start_edge_;
        geometry_msgs::Twist cmd_;
        ros_tutorials::TurtleActionResult result_;
        ros::Subscriber sub_;
        ros::Publisher pub_;

    public:
        TurtleAction(std::string name)
        : as_(nh_, name, false), action_name_(name)
        {
            as_.registerGoalCallback(boost::bind(&TurtleAction::goalCB, this));
            as_.registerPreemptCallback(boost::bind(&TurtleAction::preemptCB, this));

            sub_ = nh_.subscribe("/turtle1/pose", 1, &TurtleAction::controlCB, this);
            pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
            as_.start();
        }

        ~TurtleAction(void)
        {
        }

        void goalCB()
        {
            ros_tutorials::TurtleActionGoal goal = *as_.acceptNewGoal();
            edges_ = goal.edges;
            radius_ = goal.radius;

            interior_angle_ = ((edges_ - 2)*M_PI)/edges_;
            apothem_ = radius_ * cos(M_PI/edges_);
            side_len_ = apothem_ * 2 * tan(M_PI/edges_);
            result_.apothem = apothem_;
            result_.interior_angle = interior_angle_;
            edge_progress_ = 0;
            start_edge_ = true;
        }

        void preemptCB()
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
        }

        void controlCB(const turtlesim::Pose::ConstPtr& msg)
        {
            if (!as_.isActive())
            {
                return;
            }

            if (edge_progress_ < edges_)
            {
                double l_scale = 6.0;
                double a_scale = 6.0;
                double error_tol = 0.00001;

                if (start_edge_)
                {
                    start_x_ = msg->x;
                    start_y_ = msg->y;
                    start_theta_ = msg->theta;
                    start_edge_ = false;
                }

                dis_error_ = side_len_ - fabs(sqrt((start_x_-msg->x)*(start_x_-msg->x)+(start_y_-msg->y)+(start_y_-msg->y)));
                theta_error_ = angles::normalize_angle_positive(M_PI - interior_angle_ - (msg->theta - start_theta_));

                if (dis_error_ > error_tol)
                {
                    cmd_.linear.x = l_scale*dis_error_;
                    cmd_.angular.z = 0;
                }
                else if (dis_error_ < error_tol && fabs(theta_error_) > error_tol)
                {
                    cmd_.linear.x = 0;
                    cmd_.angular.z = a_scale*theta_error_;
                }
                else if (dis_error_ < error_tol && fabs(theta_error_) < error_tol)
                {
                    cmd_.linear.x = 0;
                    cmd_.angular.z = 0;
                    start_edge_ = true;
                    edge_progress_++;
                }
                else
                {
                    cmd_.linear.x = l_scale*dis_error_;
                    cmd_.angular.z = a_scale*theta_error_;
                }
                pub_.publish(cmd_);
            }
            else{
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded();
            }
        }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "turtle_action_server");
    TurtleAction action("turtle_action");
    ros::spin();

    return 0;
}