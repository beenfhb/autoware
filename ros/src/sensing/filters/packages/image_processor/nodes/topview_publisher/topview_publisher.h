#ifndef TOPVIEW_PUBLISHER_H_INCLUDED
#define TOPVIEW_PUBLISHER_H_INCLUDED

//headers for ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

//headers for opencv and image_transport
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class topview_publisher
{
public:
  topview_publisher();
  ~topview_publisher();
private:
  //callback functions for image_sub
  void image_callback(const sensor_msgs::ImageConstPtr& msg);
  //callback functions for camera_info_sub
  void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg);
  cv::Mat create_top_view(cv::Mat src,double theta,double height);
  //hundlers
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  //subscribers and publishers
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  ros::Subscriber camera_info_sub;
  //flags
  volatile bool camera_info_recieved;
  //setting infomation about topic names,etc...
  std::string camera_topic_name,camera_frame_name,camera_info_topic_name;
  //camera parameters
  cv::Matx33d K_matrix;
  cv::Vec4d D_vec;
  //dynamic parameters
  double pitch;
  double camera_height;
  double simulated_camera_height;
};
#endif