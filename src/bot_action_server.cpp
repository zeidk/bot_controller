#include <bot_action_server.h>
#include <bot_msgs/MoveBotActionFeedback.h>

//constructor
BotActionServer::BotActionServer(ros::NodeHandle *nodehandle, std::string action_name, std::string robot_name) : m_nh{*nodehandle},
                                                                                                                 m_robot_name{robot_name},
                                                                                                                 m_kv{0.2},
                                                                                                                 m_kh{0.26},
                                                                                                                 m_location{0, 0},
                                                                                                                 m_linear_speed{0.7},
                                                                                                                 m_angular_speed{0.6},
                                                                                                                 m_roll{0},
                                                                                                                 m_pitch{0},
                                                                                                                 m_yaw{0},
                                                                                                                 m_action_server(m_nh, action_name, boost::bind(&BotActionServer::action_server_callback, this, _1), false)
{
    m_initialize_subscribers();
    m_initialize_publishers();
    m_action_server.start();
}

double BotActionServer::m_normalize_angle_positive(double angle)
{
    const double result = fmod(angle, 2.0 * M_PI);
    if (result < 0)
        return result + 2.0 * M_PI;
    return result;
}

double BotActionServer::m_normalize_angle(double angle)
{
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0)
        return result + M_PI;
    return result - M_PI;
}

void BotActionServer::m_initialize_subscribers()
{
    m_pose_subscriber = m_nh.subscribe("odom", 1000, &BotActionServer::m_pose_callback, this);
}

void BotActionServer::m_initialize_publishers()
{
    m_velocity_publisher = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    m_feedback_publisher = m_nh.advertise<bot_msgs::MoveBotActionFeedback>("bot_action/feedback", 100);
    //add more publishers here as needed
}

// double BotActionServer::convert_rad_to_deg(double angle) {
//     return (angle * M_PI / 180.0);
// }

void BotActionServer::m_pose_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    m_location.first = odom_msg->pose.pose.position.x;
    m_location.second = odom_msg->pose.pose.position.y;
    m_orientation = odom_msg->pose.pose.orientation;
}

double BotActionServer::m_compute_distance(const std::pair<double, double> &a, const std::pair<double, double> &b)
{
    return sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
}


double BotActionServer::m_compute_distance(double x_1, double y_1, double x_2, double y_2)
{
    return sqrt(pow(x_1 - x_2, 2) + pow(y_1 - y_2, 2));
}

void BotActionServer::m_move(double linear, double angular)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    m_velocity_publisher.publish(msg);
}

void BotActionServer::stop()
{
    ROS_DEBUG_STREAM("Stopping robot");
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    m_velocity_publisher.publish(msg);
    ros::shutdown();
}

double BotActionServer::compute_yaw_rad()
{
    double roll{};
    double pitch{};
    double yaw_rad{};
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(m_orientation, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw_rad);
    return yaw_rad;
}


void BotActionServer::action_server_callback(const actionlib::SimpleActionServer<bot_msgs::MoveBotAction>::GoalConstPtr &goal)
{
    double linear_x{};
    double angular_z{};

    bool success = true;
    //helper variables
    double distance_to_goal = m_compute_distance(m_location.first, m_location.second, goal->goal_x, goal->goal_y);
    ROS_INFO_STREAM("Robot going to goal: [" << goal->goal_x << "," << goal->goal_y << "]");
    bot_msgs::MoveBotFeedback msg;
    

    while (ros::ok())
    {
        if (m_action_server.isPreemptRequested() || !ros::ok())
        {
            m_action_server.setPreempted();
            success = false;
            return;
        }
        if (distance_to_goal > 0.05)
        {
            distance_to_goal = m_compute_distance(m_location.first, m_location.second, goal->goal_x, goal->goal_y);
            m_action_feedback.status = "Robot is " + std::to_string(distance_to_goal) + " m away from goal";
            // ROS_INFO_STREAM("Robot is " << std::to_string(distance_to_goal) << " m away from goal");
            m_action_server.publishFeedback(m_action_feedback);

            double angle_to_goal = std::atan2(goal->goal_y - m_location.second, goal->goal_x - m_location.first);

            if (angle_to_goal < 0)
                // angle_to_goal = 2 * M_PI + angle_to_goal;
                angle_to_goal = m_normalize_angle_positive(angle_to_goal);

            //angle to rotate to face the goal
            double w = angle_to_goal - compute_yaw_rad();

            if (w > M_PI)
            {
                w = w - 2 * M_PI;
            }

            //proportional control for linear velocity
            linear_x = std::min(m_kv * distance_to_goal, m_linear_speed);

            //proportional control for angular velocity
            angular_z = m_kh * w;
            if (angular_z > 0)
                angular_z = std::min(angular_z, m_angular_speed);
            else
                angular_z = std::max(angular_z, -m_angular_speed);

            m_move(linear_x, angular_z);
        }
        else
        {
            if (success)
            {
                m_action_result.result="Goal Reached!!";
                m_action_server.setSucceeded(m_action_result);
            }
            stop();
            break;
        }
        
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "as_node");
    ros::NodeHandle nh;
    BotActionServer bot_action_server(&nh, "bot_action", "waffle");
    // ros::spin();

    while (ros::ok())
        ros::spinOnce();
}