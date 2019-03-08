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

#include <calibration_publisher/projection_matrix_publisher.h>

ProjectionMatrixPublisher::ProjectionMatrixPublisher(ros::NodeHandle nh,
                                                     ros::NodeHandle pnh)
    : tf_listener_(tf_buffer_) {
  nh_ = nh;
  pnh_ = pnh;
  pnh_.param<std::string>("camera_info_topic", camera_info_topic_,
                          "camera_info");
  pnh_.param<std::string>("projection_matrix_topic", projection_matrix_topic_,
                          "/projection_matrix");
  pnh_.param<std::string>("camera_optical_frame", camera_optical_frame_,
                          "camera_optical");
  pnh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
  camera_info_sub_ =
      nh_.subscribe(camera_info_topic_, 1,
                    &ProjectionMatrixPublisher::cameraInfoCallback, this);
  proj_matrix_pub_ = nh_.advertise<autoware_msgs::ProjectionMatrix>(
      projection_matrix_topic_, 1);
}

ProjectionMatrixPublisher::~ProjectionMatrixPublisher() {}

void ProjectionMatrixPublisher::getRPY(const geometry_msgs::Quaternion &q,
                                       double &roll, double &pitch,
                                       double &yaw) {
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return;
}

void ProjectionMatrixPublisher::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr msg) {
  // lookup transform between camera nad lidar
  geometry_msgs::TransformStamped transform_stamped_;
  try {
    transform_stamped_ =
        tf_buffer_.lookupTransform(camera_optical_frame_, lidar_frame_,
                                   msg->header.stamp, ros::Duration(0.1));
    // convert quat -> rpy
    double r, p, y;
    getRPY(transform_stamped_.transform.rotation, r, p, y);
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setRPY(r, p, y);
    Eigen::MatrixXd proj_mat(4, 4);
    for (int i = 0; i < 3; i++) {
      for (int m = 0; m < 3; m++) {
        proj_mat(i, m) = rotation_matrix[i][m];
      }
    }
    proj_mat(0, 3) = transform_stamped_.transform.translation.x;
    proj_mat(1, 3) = transform_stamped_.transform.translation.y;
    proj_mat(2, 3) = transform_stamped_.transform.translation.z;
    proj_mat(3, 0) = 0.0;
    proj_mat(3, 1) = 0.0;
    proj_mat(3, 2) = 0.0;
    proj_mat(3, 3) = 1.0;
    autoware_msgs::ProjectionMatrix proj_mat_msg;
    for (int i = 0; i < 4; i++) {
      for (int m = 0; m < 4; m++) {
        proj_mat_msg.projection_matrix[i * 4 * m] = proj_mat(i, m);
      }
    }
    proj_mat_msg.header = msg->header;
    proj_matrix_pub_.publish(proj_mat_msg);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
}