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
#include <autoware_map_msgs/SignalLightArray.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <autoware_msgs/ProjectionMatrix.h>
#include <sensor_msgs/CameraInfo.h>

//headers in boost
#include <boost/optional.hpp>

//headers in Autoware
#include <autoware_map/autoware_map.h>

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
    ros::Publisher roi_signal_pub_;
    std::string camera_info_topic_;
    std::string proj_matrix_topic_;
    boost::optional<Eigen::MatrixXd> proj_matrix_;
    boost::optional<Eigen::MatrixXd> p_matrix_;
    void targetSignalLightCallback(const autoware_map_msgs::SignalLightArray::ConstPtr msg);
    void projectionMatrixCallback(const autoware_msgs::ProjectionMatrix::ConstPtr msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr msg);
    autoware_map::AutowareMap autoware_map_;
};

#endif  //AUTOWARE_MAP_SIGNAL_PROJECTOR_H_INCLUDED