#include <ros/ros.h>
#include "bot_controller.h"

//forward declaration
void print_usage(std::string error = "");

void print_usage(std::string error) {
    if (!error.empty())//if not empty string
        ROS_ERROR_STREAM("Wrong usage of arguments: " << error << "\n");
    ROS_INFO_STREAM("\nusage: rosrun bot_controller bot_controller_node [-d/-r/-s/-g] [<values>] [-f/-b]\n"
        << "  -d: drive straight\n"
        << "  -r: rotate\n"
        << "  -s : stop the robot\n"
        << "  -g : go to goal\n"
        << "  <values>: numeric value(s) (double):\n"
        << "        - distance to drive (m): Only 1 numeric value must be provided\n"
        << "        - relative angle to rotate (deg): Only 1 numeric value must be provided\n"
        << "        - position to reach: 2 numeric values must be provided (x and y)\n"
        << "  -f: drive forward or positive rotation (works only with -s and -r)\n"
        << "  -b: drive backward or negative rotation (works only with -s and -r)\n"
        << "  -h: print this screen\n");
    
    ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_controller");
    ros::NodeHandle nh("~");
    // ROS_INFO_STREAM("Press Ctrl-c to exit.");
    //controller object
    Bot_Controller controller(&nh);

    std::string motion_type;
    if (nh.hasParam("motion")) {
        nh.getParam("motion", motion_type);
        ROS_INFO_STREAM("Motion Type: " << motion_type);
    }
    else {
        print_usage("_motion:= [s/r/g/h]");
        ros::shutdown();
    }
    double drive_value;
    if (nh.hasParam("drive_value")) {
        nh.getParam("drive_value", drive_value);
    }
    bool direction;
    if (nh.hasParam("direction")) {
        nh.getParam("direction", direction);
    }
    double angle;
    if (nh.hasParam("angle")) {
        nh.getParam("angle", angle);
    }
    double goal_x;
    if (nh.hasParam("goal_x")) {
        nh.getParam("goal_x", goal_x);
    }
    double goal_y;
    if (nh.hasParam("goal_y")) {
        nh.getParam("goal_y", goal_y);
    }

    if (motion_type.compare("help") == 0)
        print_usage();
    

    ros::Rate rate(20);
    //wait a bit so odom gets updated with the current pose of the robot
    ros::Duration(2).sleep(); // sleep for 2 s

    static double final_angle{ 0 };
    bool command_sent = false;

    while (ros::ok()) {
        if (motion_type.compare("h") == 0)
                controller.stop();
        else if (motion_type.compare("s") == 0)
                controller.drive_straight(drive_value, direction);
        else if (motion_type.compare("r") == 0) {
                if (final_angle == 0)
                    final_angle = controller.compute_expected_final_yaw(direction, drive_value);
                controller.rotate(drive_value, direction, final_angle);
            }
        else if (motion_type.compare("g") == 0) {
                controller.go_to_goal(goal_x, goal_y);
            }
            // command_sent = true;
            ros::spinOnce();

        rate.sleep();
    }
}