#ifndef BOT_ACTION_SERVER_H
#define BOT_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <bot_msgs/MoveBotAction.h>
#include <string>
#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>     //for nav_msgs::Odometry
#include <sensor_msgs/LaserScan.h> //for laser scans
#include <utility>
#include <tf/transform_datatypes.h> //to manipulate quaternions

class BotActionServer
{
public:
    BotActionServer(ros::NodeHandle *nodehandle, std::string action_name, std::string robot_name);
    void action_server_callback(const actionlib::SimpleActionServer<bot_msgs::MoveBotAction>::GoalConstPtr &goal);
    void stop();
    double compute_yaw_rad();
    void publish_velocities(const geometry_msgs::Twist &msg);

private:
    ros::NodeHandle m_nh;
    std::string m_robot_name;
    ros::Publisher m_velocity_publisher;
    ros::Publisher m_feedback_publisher;
    double m_kv; //gain for linear velocity
    double m_kh; //gain for angular velocity
    std::pair<double, double> m_location;
    geometry_msgs::Quaternion m_orientation;
    double m_linear_speed;
    double m_angular_speed;
    double m_roll;                                                     //rad
    double m_pitch;                                                    //rad
    double m_yaw;                                                      //rad
    void m_pose_callback(const nav_msgs::Odometry::ConstPtr &msg);     // prototype for callback of example subscriber
    void m_initialize_subscribers();
    void m_initialize_publishers();
    double m_compute_distance(const std::pair<double, double> &a, const std::pair<double, double> &b);
    double m_compute_distance(double x_1, double y_1, double x_2, double y_2);
    void m_move(double linear, double angular);
    double m_normalize_angle_positive(double angle);
    double m_normalize_angle(double angle);

    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<bot_msgs::MoveBotAction> m_action_server;
    std::string m_action_name;
    bot_msgs::MoveBotGoal m_action_goal;
    bot_msgs::MoveBotFeedback m_action_feedback;
    bot_msgs::MoveBotResult m_action_result;
    ros::Subscriber m_pose_subscriber;
};

#endif