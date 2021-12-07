#include "ros/ros.h"
#include "bot_msgs/SnapPicture.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "snap_picture_client");
  if (argc != 2)
  {
    ROS_INFO("usage: snap_picture <picture_name>");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<bot_msgs::SnapPicture>("snap_picture");
  bot_msgs::SnapPicture srv;
  srv.request.picture_name = argv[1];
  if (client.call(srv))
  {
    ROS_INFO_STREAM("Picture status: " << srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service snap_picture");
    return 1;
  }

  return 0;
}