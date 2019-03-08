#ifndef CALIBRATION_PUBLISHER_PROJECTION_MATRIX_PUBLISHER_H_INCLUDED
#define CALIBRATION_PUBLISHER_PROJECTION_MATRIX_PUBLISHER_H_INCLUDED

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#define EIGEN_MPL2_ONLY

// headers in ROS
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>

// headers in Eigen
#include <Eigen/Core>

// headers in Autoware
#include <autoware_msgs/ProjectionMatrix.h>

class ProjectionMatrixPublisher {
public:
  ProjectionMatrixPublisher(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~ProjectionMatrixPublisher();

private:
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr msg);
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher proj_matrix_pub_;
  ros::Subscriber camera_info_sub_;
  std::string camera_info_topic_;
  std::string lidar_frame_;
  std::string projection_matrix_topic_;
  std::string camera_optical_frame_;
  void getRPY(const geometry_msgs::Quaternion &q, double &roll, double &pitch,
              double &yaw);
};

#endif // CALIBRATION_PUBLISHER_PROJECTION_MATRIX_PUBLISHER_H_INCLUDED