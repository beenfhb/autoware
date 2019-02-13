#ifndef AUTOWARE_MAP_SIGNAL_PROJECTOR_H_INCLUDED
#define AUTOWARE_MAP_SIGNAL_PROJECTOR_H_INCLUDED

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 * ver 1.0 Masaya Kataoka
 */

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers in boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//headers in STL
#include <mutex>

//headers in Autoware
#include <autoware_map/autoware_map.h>
#include <autoware_map_msgs/SignalLightArray.h>
#include <autoware_msgs/SignalRoi.h>
#include <autoware_msgs/ProjectionMatrix.h>

//headers in Eigen
#include <Eigen/Core>

class AutowareMapSignalProjector
{
public:
    AutowareMapSignalProjector(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~AutowareMapSignalProjector();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber signal_light_sub_;
    ros::Subscriber projection_matrix_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Publisher roi_signal_pub_;
    std::string camera_info_topic_;
    std::string proj_matrix_topic_;
    boost::optional<Eigen::MatrixXd> proj_matrix_;
    boost::optional<Eigen::MatrixXd> p_matrix_;
    boost::optional<autoware_map_msgs::SignalLightArray> target_roi_;
    double publish_rate_;
    std::string camera_frame_;
    std::string map_frame_;
    int signal_light_radius_;
    void targetSignalLightCallback(const autoware_map_msgs::SignalLightArray::ConstPtr msg);
    void projectionMatrixCallback(const autoware_msgs::ProjectionMatrix::ConstPtr msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr msg);
    void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr msg);
    autoware_map::AutowareMap autoware_map_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif  //AUTOWARE_MAP_SIGNAL_PROJECTOR_H_INCLUDED