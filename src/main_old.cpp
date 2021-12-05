#include <ros/ros.h>
#include "bot_controller.h"

//forward declaration
void print_usage(std::string error = "");
void handle_inputs(int argc, char** argv, Bot_Controller& controller);

struct controller_options {
    std::string drive_type;
    double value; //m or deg
    bool direction;
    double angle;
    double goal_x;
    double goal_y;
};

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

void handle_inputs(int argc, char** argv, controller_options& options) {
    std::string drive_type{};
    double value{};

    if (argc > 1) {
        std::string arg1{ argv[1] };
        if (argc == 2) {
            if (arg1 == "-s") {
                options.drive_type = "stop";
                // controller.stop();
            }
            else
                print_usage("Only possible argument: -s");
        }
        else if (argc == 4) {//-d val -f
            double arg2{ std::stod(argv[2]) };//distance or angle or goal_x
            options.value = arg2;
            if (arg1 == "-d") {
                options.drive_type = "straight";
                std::string arg3{ argv[3] };//f or b
                if (arg3 == "-f")
                    options.direction = true;
                else if (arg3 == "-b") {
                    options.direction = false;
                }
                else
                    print_usage("-d <val> -f/-b");
            }
            else if (arg1 == "-r") {
                options.drive_type = "rotate";
                std::string arg3{ argv[3] };//f or b
                if (arg3 == "-f")
                    options.direction = true;
                else if (arg3 == "-b")
                    options.direction = false;
                else
                    print_usage("-d <val> -f/-b");
            }
            else if (arg1 == "-g") {
                options.drive_type = "go_to_goal";
                double arg3{ std::stod(argv[3]) }; //goal_y
                options.goal_x = arg2;
                options.goal_y = arg3;
            }
            else
                print_usage("Possible arguments: -s/-d/-r/-g");
        }
        else print_usage("Incorrect number of arguments provided");
    }
    else
        print_usage("Incorrect number of arguments provided");
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "my_controller");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("Press Ctrl-c to exit.");
    //controller object
    Bot_Controller controller(&nh);
    controller_options options;

    if (argc == 2) {
        if (argv[1] == "-h")
            print_usage();
    }
    handle_inputs(argc, argv, options);


    ros::Rate rate(20);
    //wait a bit so odom gets updated with the current pose of the robot
    ros::Duration(2).sleep(); // sleep for 2 s

    static double final_angle{ 0 };

    while (ros::ok()) {
        ros::spinOnce();
        if (options.drive_type == "stop")
            controller.stop();
        else if (options.drive_type == "straight")
            controller.drive_straight(options.value, options.direction);
        else if (options.drive_type == "rotate") {
            if (final_angle == 0)
                final_angle = controller.compute_expected_final_yaw(options.direction, options.value);
            controller.rotate(options.value, options.direction, final_angle);
        }
        else if (options.drive_type == "go_to_goal") {
            controller.go_to_goal(options.goal_x, options.goal_y);
        }

        rate.sleep();
    }
}