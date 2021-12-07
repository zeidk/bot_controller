#ifndef BOT_ACTION_SERVER_H
#define BOT_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <bot_msgs/MoveBotAction.h>
#include <bot_controller.h>
#include <string>

class BotActionServer
{
public:
    BotActionServer(ros::NodeHandle *nodehandle, std::string action_name, std::string robot_name);
    void action_server_callback(const actionlib::SimpleActionServer<bot_msgs::MoveBotAction>::GoalConstPtr &goal);
    double compute_distance(double x_1, double y_1, double x_2, double y_2);

private:
    ros::NodeHandle m_nh;
    Bot_Controller m_bot_controller;
    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<bot_msgs::MoveBotAction> m_action_server;
    std::string m_action_name;
    bot_msgs::MoveBotGoal m_action_goal;
    bot_msgs::MoveBotFeedback m_action_feedback;
    bot_msgs::MoveBotResult m_action_result;
};

#endif