
//headers in robocup_localization package
#include "topview_publisher.h"

//headers for ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

//headers for opencv and image_transport
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//headers for tf
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

//headers for std libs
#include <math.h>

topview_publisher::topview_publisher() : it_(nh_)
{
  camera_info_recieved_ = false;
  nh_.getParam(ros::this_node::getName()+"/camera_topic_name_", camera_topic_name_);
  nh_.getParam(ros::this_node::getName()+"/camera_info_topic_name_", camera_info_topic_name_);
  image_pub_ =  it_.advertise(ros::this_node::getName()+"/top_view", 1);
  camera_info_sub_ = nh_.subscribe(camera_info_topic_name_, 1, &topview_publisher::camera_info_callback_, this);
  image_sub_ = it_.subscribe(camera_topic_name_, 1, &topview_publisher::image_callback_, this);
}

topview_publisher::~topview_publisher()
{
}

void topview_publisher::camera_info_callback_(const sensor_msgs::CameraInfoConstPtr& msg)
{
  K_matrix_ = cv::Matx33d(msg->K[0],msg->K[1],msg->K[2],msg->K[3],msg->K[4],msg->K[5],msg->K[6],msg->K[7],msg->K[8]);
  D_vec_ = cv::Vec4d(msg->D[0],msg->D[1],msg->D[2],msg->D[3]);
}

void topview_publisher::image_callback_(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr->image = create_top_view_(cv_ptr->image,pitch_,camera_height_,simulated_camera_height_);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

cv::Mat topview_publisher::create_top_view_(cv::Mat src,double theta,double height,double simulated_height)
{
  cv::Mat undistorted_image = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
  cv::undistort(src, undistorted_image, K_matrix_, D_vec_);
  cv::Mat top_image = cv::Mat::zeros(undistorted_image.rows, undistorted_image.cols, CV_8UC3);
  double Hvc = simulated_height;
  double Hc = height;
  double Dvc = 0.0;
  double f = K_matrix_(0,0);
  double fp = f;
  double s = sin(theta);
  double c = cos(theta);
  int cx = undistorted_image.cols/2;
  int cy = undistorted_image.rows;
  int cxp = undistorted_image.cols/2;
  int cyp = undistorted_image.rows;
  for (int y = 0; y < top_image.rows; y++)
  {
    for (int x = 0; x < top_image.cols; x++)
    {
      int xOrg = x - cx;
      int yOrg = - y + cy;
      double oldX = 0.5 + (Hvc / Hc) * (f / fp) * c * ( s/c - (yOrg*Hvc*s - fp*Hc*c + fp*Dvc*s) / (fp*Hc*s + Hvc*yOrg*c + fp*Dvc*c) ) * xOrg;
      double oldY = 0.5 + f * ((yOrg*Hvc*s - fp*Hc*c + fp*Dvc*s)/(fp*Hc*s + Hvc*yOrg*c + fp*Dvc*c));
      oldX = oldX + cxp;
      oldY = -oldY + cyp;
      if (oldX < 0 || top_image.cols - 1 < oldX || oldY < 0 || top_image.rows - 1 < oldY )
      {
        continue;
      }
      if((int)oldX + 1 >= top_image.cols || (int)oldY + 1 >= top_image.rows)
      {
        top_image.data[(y * top_image.cols + x) * top_image.channels()] = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX) * top_image.channels()];
        top_image.data[(y * top_image.cols + x) * top_image.channels() + 1] = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX) * top_image.channels() + 1];
        top_image.data[(y * top_image.cols + x) * top_image.channels() + 2] = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX) * top_image.channels() + 2];
        continue;
      }
      for (int i = 0; i < top_image.channels(); i++)
      {
        uchar f11 = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX) * top_image.channels() + i];
        uchar f12 = undistorted_image.data[(((int)oldY + 1) * top_image.cols + (int)oldX) * top_image.channels() + i];
        uchar f21 = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX + 1) * top_image.channels() + i];
        uchar f22 = undistorted_image.data[(((int)oldY + 1) * top_image.cols + (int)oldX + 1) * top_image.channels() + i];
        double dx2 = (int)oldX + 1 - oldX;
        double dx1 = oldX - (int)oldX;
        double dy2 = (int)oldY + 1 - oldY;
        double dy1 = oldY - (int)oldY;
        top_image.data[(y * top_image.cols + x) * top_image.channels() + i] = dy2 * (f11 * dx2 + f21 * dx1) + dy1 * (f12 * dx2 + f22 * dx1);
      }
    }
  }
  return top_image;
}