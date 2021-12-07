#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <bot_msgs/MoveBotAction.h>

void done_callback(const actionlib::SimpleClientGoalState &state,
                   const bot_msgs::MoveBotResultConstPtr &result)
{
    ROS_INFO("Action client callback: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char **argv)
{

    double goal_x;
    double goal_y;

    ros::init(argc, argv, "action_client_node");

    // here is a "goal" object compatible with the server, as defined in example_action_server/action
    bot_msgs::MoveBotGoal goal;

    // use the name of our server, which is: example_action (named in example_action_server.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)
    actionlib::SimpleActionClient<bot_msgs::MoveBotAction> action_client("action_server", true);

    if (ros::param::has("~/goal_x"))
    {
        ros::param::get("~/goal_x", goal_x);
    }
    else
    {
        ROS_FATAL_STREAM("Parameter _goal_x not set");
        action_client.cancelAllGoals();
        return 0; 
    }

    if (ros::param::has("~/goal_y"))
    {
        ros::param::get("~/goal_y", goal_y);
    }
    else
    {
        ROS_FATAL_STREAM("Parameter goal_y not set");
        action_client.cancelAllGoals();
        return 0; 
    }
    //stuff the goal
    goal.goal_x = goal_x;
    goal.goal_y = goal_y;
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists)
    {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }

    ROS_INFO("connected to action server"); // if here, then we connected to the server;

    
    action_client.sendGoal(goal, &done_callback); // we could also name additional callback functions here, if desired
    //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
    // while (action_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(120.0)); //wait 2 min
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...

        if (!finished_before_timeout)
        {
            ROS_WARN_STREAM("giving up waiting on result to reach goal"
                            << "[" << goal_x << "," << goal_y << "]");
            return 0;
        }
            
    // }
    // ROS_INFO_STREAM("Awesome, robot reached the goal!!");
    

    return 0;
}