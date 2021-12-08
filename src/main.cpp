#include <ros/ros.h>

#include "bot_controller.h"

// forward declaration
void print_usage(std::string error = "");

void print_usage(std::string error)
{
  if (!error.empty())  // if not empty string
    ROS_ERROR_STREAM(error << "\n");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bot_controller_node");
  ros::NodeHandle nh("~");

  std::string robot_name;
  if (nh.hasParam("robot_name")) {
    nh.getParam("robot_name", robot_name);
    ROS_INFO_STREAM("robot name: " << robot_name);
  }
  else {
    print_usage("missing argument: _robot_name:= <name>");
  }

  //initialize Bot_Controller object
  Bot_Controller controller(&nh, robot_name);

  std::string motion_type;
  if (nh.hasParam("motion"))
  {
    nh.getParam("motion", motion_type);
  }
  else
  {
    print_usage("missing argument: _motion:= <s/r/g/h>");
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
      }
    }
    else
    {
      print_usage("_direction:= <f/b>");
    }
  }

  double goal_x;
  double goal_y;
  if (motion_type == "g")
  {
    if (!nh.hasParam("goal_x"))
      print_usage("missing argument: _goal_x:=<double>");
    else
      nh.getParam("goal_x", goal_x);

    if (!nh.hasParam("goal_y"))
      print_usage("missing argument: _goal_y:=<double>");
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
    ros::spinOnce();

    rate.sleep();
  }
}