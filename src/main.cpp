#include <ros/ros.h>

#include "bot_controller.h"

// forward declaration
void print_usage(std::string error = "");

void print_usage(std::string error)
{
  if (!error.empty())  // if not empty string
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
  // controller object

  std::string robot_name;
  if (!ros::param::has("~/robot_name"))
  {
    print_usage("robot name missing: _robot_name:=<robot_name>");
  }
  else
  {
    ros::param::get("~/robot_name", robot_name);
  }

  Bot_Controller controller(&nh, robot_name);

  std::string motion_type;
  if (nh.hasParam("motion"))
  {
    nh.getParam("motion", motion_type);
    ROS_INFO_STREAM("Motion Type: " << motion_type);
    // ros::shutdown();
  }
  else
  {
    print_usage("_motion:= <s/r/g/h>");
    ros::shutdown();
  }

  double drive_value;

  std::string direction_s;
  bool direction_b;
  if (motion_type == "s" || motion_type == "r")
  {
    if (nh.hasParam("value"))
    {
      nh.getParam("value", drive_value);
    }
    else
    {
      print_usage("_value:= <double>");
      ros::shutdown();
    }

    if (nh.hasParam("direction"))
    {
      nh.getParam("direction", direction_s);
      if (direction_s == "f")
        direction_b = true;
      else if (direction_s == "b")
        direction_b = false;
      else
      {
        print_usage("_direction:=<f/b>");
        ros::shutdown();
      }
    }
    else
    {
      print_usage("_direction:= <f/b>");
      ros::shutdown();
    }
  }

  double goal_x;
  double goal_y;
  if (motion_type == "g")
  {
    if (!nh.hasParam("goal_x"))
      print_usage("_goal_x:=<double>");
    else
      nh.getParam("goal_x", goal_x);

    if (!nh.hasParam("goal_y"))
      print_usage("_goal_y:=<double>");
    else
      nh.getParam("goal_y", goal_y);
  }

  ros::Rate rate(20);
  // wait a bit so odom gets updated with the current pose of the robot
  ros::Duration(2).sleep();  // sleep for 2 s

  static double final_angle{ 0 };

  while (ros::ok())
  {
    if (motion_type == "h")
      controller.stop();
    else if (motion_type == "s")
      controller.drive_straight(drive_value, direction_b);
    else if (motion_type == "r")
    {
      if (final_angle == 0)
        final_angle = controller.compute_expected_final_yaw(direction_b, drive_value);
      controller.rotate(drive_value, direction_b, final_angle);
    }
    else if (motion_type == "g")
    {
      controller.go_to_goal(goal_x, goal_y);
    }
    // command_sent = true;
    ros::spinOnce();

    rate.sleep();
  }
}