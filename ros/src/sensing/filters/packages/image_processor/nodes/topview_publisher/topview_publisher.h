#ifndef TOPVIEW_PUBLISHER_H_INCLUDED
#define TOPVIEW_PUBLISHER_H_INCLUDED

//headers for ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>

//headers for opencv and image_transport
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

//headers in this package
#include <image_processor/topview_publisherConfig.h>

class topview_publisher
{
public:
  topview_publisher();
  ~topview_publisher();
private:
  //callback functions for image_sub_
  void image_callback_(const sensor_msgs::ImageConstPtr& msg);
  //callback functions for camera_info_sub_
  void camera_info_callback_(const sensor_msgs::CameraInfoConstPtr& msg);
  //callback function from dynamic_reconfigure
  void callback_config_(image_processor::topview_publisherConfig &config, uint32_t level);
  //function generate top_view
  cv::Mat create_top_view_(cv::Mat src,double theta,double height,double simulated_height);
  //hundlers
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  //subscribers and publishers
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber camera_info_sub_;
  //flags
  volatile bool camera_info_recieved_;
  //setting infomation about topic names,etc...
  std::string camera_topic_name_,camera_info_topic_name_;
  //camera parameters
  cv::Matx33d K_matrix_;
  cv::Vec4d D_vec_;
  //dynamic parameters
  double pitch_;
  double camera_height_;
  double simulated_camera_height_;
  dynamic_reconfigure::Server<image_processor::topview_publisherConfig> dynamic_params_server_;
  dynamic_reconfigure::Server<image_processor::topview_publisherConfig>::CallbackType dynamic_params_callback_type_;
};
#endif