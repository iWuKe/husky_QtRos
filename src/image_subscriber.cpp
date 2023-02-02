#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui.hpp"
//#include "rclcpp/logging.hpp"
//#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/Image.h>

//void imageCallback(const sensor_msgs::ImageConstPtr& msg)
//{
//  sensor_msgs::CvBridge bridge;
//  try
//  {
//    cvShowImage("view", bridge.imgMsgToCv(msg, "bgr8"));
//  }
//  catch (sensor_msgs::CvBridgeException& e)
//  {
//    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//  }
//}

void imageCallback2(const sensor_msgs::ImageConstPtr & msg)
{
  try {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    //auto logger = rclcpp::get_logger("my_subscriber");
    //RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cvNamedWindow("view");
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
  cvDestroyWindow("view");
}
