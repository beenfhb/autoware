/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 */

#ifndef __DETECT_INVALID_NEAR_POINTS_CORE_H
#define __DETECT_INVALID_NEAR_POINTS_CORE_H

#include <vector>

#include <ros/ros.h>

#include <velodyne_msgs/VelodyneScan.h>

#include "rawdata.h"

namespace detect_invalid_near_points
{

  typedef velodyne_rawdata::VPoint VPoint;
  typedef velodyne_rawdata::VPointCloud VPointCloud;

  class DetectInvalidNearPoints
  {
      enum class MethodType {
          UseValidIntensity,
          UseBottomRing,
      };

  public:
    DetectInvalidNearPoints(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~DetectInvalidNearPoints() = default;

  private:
    void callbackPackets(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);
    VPointCloud filteringPointsByLabeling(const std::vector<VPointCloud>& in_points_array, const size_t points_size_threshold);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber velodyne_packets_sub_;

    ros::Publisher invalid_near_points_pub_;
    ros::Publisher invalid_distance_points_pub_;
    ros::Publisher found_invalid_near_points_pub_;

    boost::shared_ptr<velodyne_rawdata::RawData> data_;

    MethodType method_type_;
    int num_points_thresthold_;
    int num_lasers_;
    std::vector<float> invalid_intensity_array_;
  };

}

#endif
