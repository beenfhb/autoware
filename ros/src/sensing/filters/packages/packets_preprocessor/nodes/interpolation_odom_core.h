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

#ifndef __INTERPOLATION_ODOM_CORE_H
#define __INTERPOLATION_ODOM_CORE_H

#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "rawdata.h"

namespace interpolation_odom
{

  typedef velodyne_rawdata::VPoint VPoint;
  typedef velodyne_rawdata::VPointCloud VPointCloud;

  class InterpolationOdom
  {

  public:
    InterpolationOdom(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~InterpolationOdom() = default;

  private:
    void callbackPackets(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);
    void callbackOdometry(const nav_msgs::Odometry::ConstPtr &odomMsg);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber velodyne_packets_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher transformed_points_pub_;
    ros::Publisher points_raw_pub_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;


    boost::shared_ptr<velodyne_rawdata::RawData> data_;
    nav_msgs::Odometry odomMsg_;

  };

}

#endif
