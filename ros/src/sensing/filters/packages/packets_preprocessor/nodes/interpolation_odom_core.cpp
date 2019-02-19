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

#include "interpolation_odom_core.h"

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/image_encodings.h>
// #include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>

// #include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

// #include "rawdata.h"
#include "pointcloudXYZIRADT.h"

namespace interpolation_odom
{
  InterpolationOdom::InterpolationOdom(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh)
    , private_nh_(private_nh)
    , data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh_);

    double view_direction = 0.0;
    private_nh_.getParam("view_direction", view_direction);
    double view_width = 6.28318530718;
    private_nh_.getParam("view_width", view_width);
    data_->setParameters(0.0, 200.0, view_direction, view_width);


    velodyne_packets_sub_ = nh_.subscribe("/velodyne_packets", 10, &InterpolationOdom::callbackPackets, this);
    odom_sub_ = nh_.subscribe("/vehicle/odom", 10, &InterpolationOdom::callbackOdometry, this);

    points_raw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_raw", 10);
    transformed_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("transformed_points", 10);
  }

  void InterpolationOdom::callbackOdometry(const nav_msgs::Odometry::ConstPtr &odomMsg)
  {
      odomMsg_ = *odomMsg;

      //TODO Remove
      // geometry_msgs::TransformStamped odom_trans;
      // odom_trans.header.stamp = odomMsg->header.stamp;
      // odom_trans.header.frame_id = "odom";
      // odom_trans.child_frame_id = "base_link";
      // odom_trans.transform.translation.x = odomMsg->pose.pose.position.x;
      // odom_trans.transform.translation.y = odomMsg->pose.pose.position.y;
      // odom_trans.transform.translation.z = odomMsg->pose.pose.position.z;
      // odom_trans.transform.rotation = odomMsg->pose.pose.orientation;
      // tf_broadcaster_.sendTransform(odom_trans);
  }

  void InterpolationOdom::callbackPackets(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (points_raw_pub_.getNumSubscribers() == 0 && transformed_points_pub_.getNumSubscribers() == 0){
      return;
    }

    static bool is_set_tf = false;
    static tf::StampedTransform tf_base_link_to_sensor_;

    if(!is_set_tf) {
        std::string base_link_frame_ = "/base_link";
        std::string sensor_frame_ = pcl_conversions::toPCL(scanMsg->header).frame_id;
        try {
            ros::Duration(0.1).sleep();  //wait for tf. especially use sim_time
            tf_listener_.waitForTransform(base_link_frame_, sensor_frame_, ros::Time(0), ros::Duration(1.0));
            tf_listener_.lookupTransform(base_link_frame_, sensor_frame_, ros::Time(0), tf_base_link_to_sensor_);
            is_set_tf = true;
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ROS_ERROR("Please publish TF %s to %s", base_link_frame_.c_str(), sensor_frame_.c_str());
            //exit(1);
        }
    }

    double v = odomMsg_.twist.twist.linear.x;
    double w = odomMsg_.twist.twist.angular.z;

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> transformed_points;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> points_raw;

    for (size_t next = 0; next < scanMsg->packets.size(); ++next)
    {
        double packet_time_offset = (scanMsg->packets[next].stamp - scanMsg->packets[0].stamp).toSec();
        velodyne_pointcloud::PointcloudXYZIRADT scan;
        data_->unpack(scanMsg->packets[next], scan);

        for(const auto& pc : *(scan.pc)) {
            velodyne_pointcloud::PointXYZIR p;
            double theta = w * pc.time_offset;

            tf::Transform trans;
            trans.setOrigin(tf::Vector3(0, 0, 0));
            tf::Quaternion q;
            q.setRPY(0, 0, theta);
            q = q*tf_base_link_to_sensor_.getRotation().inverse();
            trans.setRotation(q);

            tf::Point point(v * (packet_time_offset+pc.time_offset), 0, 0);
            tf::Point tf_p = trans * point;
            p.x = pc.x + tf_p.getX();
            p.y = pc.y + tf_p.getY();
            p.z = pc.z + tf_p.getZ();

            p.intensity = pc.intensity;
            p.ring = pc.ring;
            transformed_points.push_back(p);

            p.x = pc.x;
            p.y = pc.y;
            p.z = pc.z;
            points_raw.push_back(p);
        }
    }
    transformed_points.header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    transformed_points.header.frame_id = pcl_conversions::toPCL(scanMsg->header).frame_id;
    transformed_points.height = 1;
    transformed_points.width = transformed_points.points.size();
    transformed_points_pub_.publish(transformed_points);

    points_raw.header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    points_raw.header.frame_id = pcl_conversions::toPCL(scanMsg->header).frame_id;
    points_raw.height = 1;
    points_raw.width = points_raw.points.size();
    points_raw_pub_.publish(points_raw);

  }

}
