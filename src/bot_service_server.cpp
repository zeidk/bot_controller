#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "bot_msgs/SnapPicture.h"
#include <sensor_msgs/Image.h>

//global variable which is updated in camera_image_topic_callback
cv_bridge::CvImagePtr cv_ptr;

bool snap_picture_service_callback(bot_msgs::SnapPicture::Request &request,
                                   bot_msgs::SnapPicture::Response &response)
{
    ROS_INFO("Processing service request...");

    std::string saved_picture_path = "/tmp/" + (std::string) request.picture_name;
    bool write_status = cv::imwrite(saved_picture_path, cv_ptr->image);
    if (write_status){
        response.success = "Picture saved";
        ROS_INFO_STREAM("Picture saved at: " << saved_picture_path);
    }
        
    else
        response.success = "Failed to save picture";
    return true;
}

void camera_image_topic_callback(const sensor_msgs::Image::ConstPtr &img_msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snap_picture_server");
    ros::NodeHandle nh;
    ros::Subscriber camera_topic_sub = nh.subscribe("/camera/rgb/image_raw", 1000, &camera_image_topic_callback);
    ros::ServiceServer service = nh.advertiseService("snap_picture", snap_picture_service_callback);
    ROS_INFO("Ready to snap pictures from robot's camera.");
    ros::spin();
}