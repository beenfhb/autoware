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

#include "detect_invalid_near_points_core.h"

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
#include "pointcloudXYZIRAD.h"

namespace detect_invalid_near_points
{
  DetectInvalidNearPoints::DetectInvalidNearPoints(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh)
    , private_nh_(private_nh)
    , data_(new velodyne_rawdata::RawData())
    , method_type_(MethodType::UseValidIntensity)
    , num_points_thresthold_(300)
    , num_lasers_(0)
  {
    data_->setup(private_nh_);
    num_lasers_ = data_->getNumLasers();

    if(num_lasers_ != 16){
        ROS_WARN("This node is debugged only VLP16. Other sensors may not operate properly.");
    }

    double view_direction = 0.0;
    private_nh_.getParam("view_direction", view_direction);
    double view_width = 6.28318530718;
    private_nh_.getParam("view_width", view_width);
    data_->setParameters(0.0, 200.0, view_direction, view_width);

    int tmp_method_type = static_cast<int>(method_type_);
    private_nh_.getParam("method_type", tmp_method_type);
    method_type_ = static_cast<MethodType>(tmp_method_type);

    private_nh_.getParam("num_points_thresthold", num_points_thresthold_);

    std::string invalid_intensity;
    private_nh_.getParam("invalid_intensity", invalid_intensity);
    YAML::Node topics = YAML::Load(invalid_intensity);
    invalid_intensity_array_ = std::vector<float>(num_lasers_, 0);
    for(size_t i = 0; i < topics.size(); ++i) {
        invalid_intensity_array_.at(i) = topics[i].as<float>();
    }
    // for(auto v : invalid_intensity_array_) {
    //     std::cout << v << " ";
    // }
    // std::cout << std::endl;


    velodyne_packets_sub_ = nh_.subscribe("/velodyne_packets", 10, &DetectInvalidNearPoints::callbackPackets, this);

    invalid_near_points_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("invalid_near_points", 10);
    invalid_distance_points_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("invalid_distance_points", 10);
    found_invalid_near_points_pub_ = private_nh_.advertise<std_msgs::Bool>("found_invalid_near_points", 10);
  }

  void DetectInvalidNearPoints::callbackPackets(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (invalid_near_points_pub_.getNumSubscribers() == 0 && invalid_distance_points_pub_.getNumSubscribers() == 0 && found_invalid_near_points_pub_.getNumSubscribers() == 0){
        return;
    }

    size_t zero_index = 0;
    std::vector< pcl::PointCloud<velodyne_pointcloud::PointXYZIRAD> > invalid_distane_points_array;
    for (size_t next = 0; next < scanMsg->packets.size(); ++next)
    {
        velodyne_pointcloud::PointcloudXYZIRAD invalid_distane_points;
        data_->unpack_invalid(scanMsg->packets[next], invalid_distane_points);

        pcl::PointCloud<velodyne_pointcloud::PointXYZIRAD> segment_points;
        for(size_t i = 0; i < invalid_distane_points.pc->size(); ++i) {
            segment_points.points.push_back(invalid_distane_points.pc->at(i));
            if(i % num_lasers_ == (num_lasers_-1)) {
                //Sort by ring number
                std::sort(std::begin(segment_points), std::end(segment_points),
                  [](const velodyne_pointcloud::PointXYZIRAD &lhs, const velodyne_pointcloud::PointXYZIRAD &rhs){
                      return lhs.ring < rhs.ring;
                  });
                invalid_distane_points_array.push_back(segment_points);
                segment_points.points.clear();
            }

            //find zero index
            if(invalid_distane_points.pc->at(i).ring == (num_lasers_-1)/2){
                int azimuth = invalid_distane_points.pc->at(i).azimuth;
                static int last_azimuth = azimuth;
                if(last_azimuth <= 18000 && azimuth > 18000) {
                    zero_index = invalid_distane_points_array.size();
                    std::cout << zero_index << " " << last_azimuth << " " << azimuth << std::endl;
                }
                last_azimuth = azimuth;
            }
        }

    }
    std::cout << std::endl;

    std::vector< pcl::PointCloud<velodyne_pointcloud::PointXYZIRAD> > sorted_points_array;
    sorted_points_array.insert(std::end(sorted_points_array), std::begin(invalid_distane_points_array)+zero_index, std::end(invalid_distane_points_array));
    sorted_points_array.insert(std::end(sorted_points_array), std::begin(invalid_distane_points_array), std::begin(invalid_distane_points_array)+zero_index);

    std::vector<VPointCloud> near_points_array;
    for(const auto& cloud : sorted_points_array) {
        VPointCloud tmp_cloud;
        for(auto p : cloud.points) {
            VPoint point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            point.intensity = p.intensity;
            point.ring = p.ring;
            if((method_type_ == MethodType::UseValidIntensity && p.distance == 0 && point.intensity <= 100 && point.intensity != invalid_intensity_array_[point.ring])
            || (method_type_  == MethodType::UseBottomRing && p.distance == 0  && point.intensity <= 100 && point.ring < (num_lasers_-1)/2)){
                tmp_cloud.push_back(point);
            }
            else {
                point.intensity = 0;
                tmp_cloud.push_back(point);
            }
        }
        near_points_array.push_back(tmp_cloud);
    }

    // std::cout << num_points_thresthold_ << std::endl;
    auto filtered_near_points = filteringPointsByLabeling(near_points_array, num_points_thresthold_);

    filtered_near_points.header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    filtered_near_points.header.frame_id = pcl_conversions::toPCL(scanMsg->header).frame_id;
    filtered_near_points.height = 1;
    filtered_near_points.width = filtered_near_points.points.size();
    invalid_near_points_pub_.publish(filtered_near_points);

    if (invalid_distance_points_pub_.getNumSubscribers() > 0){
        VPointCloud invalid_distance_points;
        for(const auto& cloud : sorted_points_array) {
            VPointCloud tmp_cloud;
            for(auto p : cloud.points) {
                VPoint point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;
                point.intensity = p.intensity;
                point.ring = p.ring;
                if(p.distance == 0 && point.intensity <= 100){
                    invalid_distance_points.push_back(point);
                }
            }
        }

        invalid_distance_points.header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
        invalid_distance_points.header.frame_id = pcl_conversions::toPCL(scanMsg->header).frame_id;
        invalid_distance_points.height = 1;
        invalid_distance_points.width = invalid_distance_points.points.size();
        invalid_distance_points_pub_.publish(invalid_distance_points);
    }

    std_msgs::Bool found_invalid_near_points_msg;
    found_invalid_near_points_msg.data = !filtered_near_points.empty();
    found_invalid_near_points_pub_.publish(found_invalid_near_points_msg);

    // visualization_msgs::MarkerArray marker_array_msg;
    // auto velodyne_model_marker = createVelodyneModelMakerMsg();
    // marker_array_pub_.publish(velodyne_model_marker);

  }


  VPointCloud DetectInvalidNearPoints::filteringPointsByLabeling(const std::vector<VPointCloud>& in_points_array, const size_t points_size_threshold)
  {
      cv::Mat image = cv::Mat::zeros(cv::Size(in_points_array.size(), num_lasers_), CV_8UC1);
      for(size_t y = 0; y < image.rows; ++y) {
          for(size_t x = 0; x < image.cols; ++x) {
              image.at<unsigned char>(y, x) = in_points_array.at(x).points.at(y).intensity == 0 ? 0 : 255;
          }
      }

      cv::Mat element(3, 3, CV_8UC1, cv::Scalar::all(255));
      cv::morphologyEx(image, image, cv::MORPH_OPEN, element, cv::Point(-1, -1), 3);
      cv::morphologyEx(image, image, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 3);

      cv::Mat label_image(image.size(), CV_32S);
      cv::Mat stats;
      cv::Mat centroids;
      int label_n = cv::connectedComponentsWithStats(image, label_image, stats, centroids, 8);

      std::vector<int> stat_area;
      for(size_t label = 0; label < label_n; ++label) {
          int* param = stats.ptr<int>(label);
          stat_area.push_back(param[cv::ConnectedComponentsTypes::CC_STAT_AREA]);
          // std::cout << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << " ";
      }
      // std::cout << std::endl;

      VPointCloud out_points;
      for(size_t y = 0; y < image.rows; ++y) {
          for(size_t x = 0; x < image.cols; ++x) {
              const int label = label_image.at<int>(y, x);
              const VPoint point = in_points_array.at(x).points.at(y);
              if(label != 0 && point.intensity != 0 && stat_area.at(label) >= points_size_threshold) {
                  out_points.points.push_back(point);
              }
          }
      }
      return out_points;
  }

  // visualization_msgs::MarkerArray DetectInvalidNearPoints::createVelodyneModelMakerMsg()
  // {
  //   visualization_msgs::MarkerArray marker_array_msg;
  //
  //   auto generateColor = [](float r, float g, float b, float a){
  //       std_msgs::ColorRGBA color;
  //       color.r = r;
  //       color.g = g;
  //       color.b = b;
  //       color.a = a;
  //       return color;
  //   };
  //
  //   const double radius = 0.1033;
  //   const std::array<double, 3> height = {0.020, 0.037, 0.015};
  //   const std::array<double, 3> pos_z = {-0.0285, 0.0, 0.0255};
  //   const std::array<std_msgs::ColorRGBA, 3> color = {generateColor(0.8, 0.8, 0.8, 0.8), generateColor(0.1, 0.1, 0.1, 0.98), generateColor(0.8, 0.8, 0.8, 0.8)};
  //
  //
  //   geometry_msgs::Quaternion orientation_msg;
  //   quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0), orientation_msg);
  //
  //   double prev_height = 0;
  //   for(size_t i = 0; i < 3; ++i){
  //       visualization_msgs::Marker marker;
  //       marker.header.frame_id = "velodyne";
  //       marker.header.stamp = ros::Time::now();
  //       marker.ns = "velodyne_model";
  //       marker.id = i;
  //       marker.type = visualization_msgs::Marker::CYLINDER;
  //       marker.action = visualization_msgs::Marker::ADD;
  //       marker.pose.position.x = 0;
  //       marker.pose.position.y = 0;
  //       marker.pose.position.z = pos_z[i];
  //       marker.pose.orientation = orientation_msg;
  //       marker.scale.x = radius;
  //       marker.scale.y = radius;
  //       marker.scale.z = height[i];
  //       marker.color = color[i];
  //       marker_array_msg.markers.push_back(marker);
  //   }
  //
  //   return marker_array_msg;
  // }

}
