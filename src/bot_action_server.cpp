#include <bot_action_server.h>

//constructor
BotActionServer::BotActionServer(ros::NodeHandle *nodehandle, std::string action_name, std::string robot_name):
m_nh{*nodehandle},
m_bot_controller(&m_nh, robot_name),
m_action_server(m_nh, action_name, boost::bind(&BotActionServer::action_server_callback, this, _1), false)
{
    m_action_server.start();
}


double BotActionServer::compute_distance(double x_1, double y_1, double x_2, double y_2) {
    return  sqrt(pow(x_1 - x_2, 2) + pow(y_1 - y_2, 2));
}


void BotActionServer::action_server_callback(const actionlib::SimpleActionServer<bot_msgs::MoveBotAction>::GoalConstPtr &goal){
    //helper variables

    double distance{};
    
    bool action_active = true;

    ROS_INFO_STREAM("Robot going to goal: [" << goal->goal_x << "," << goal->goal_y<<"]");
    
    if (m_action_server.isPreemptRequested() || !ros::ok()){
        m_action_server.setPreempted();
        action_active = false;
    }

    while (!m_bot_controller.go_to_goal(goal->goal_x, goal->goal_y)){
        distance = compute_distance(m_bot_controller.get_current_x(), m_bot_controller.get_current_y(), goal->goal_x, goal->goal_y);
        m_action_feedback.status = "Robot is " + std::to_string(distance)+ " m away from goal";
        m_action_server.publishFeedback(m_action_feedback);
    }
    m_action_result.result = "Goal Reached!!!";
    m_action_server.setSucceeded(m_action_result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_server_node");
    ros::NodeHandle nh;
    BotActionServer bot_action_server(&nh, "action_server", "waffle");

    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }
}