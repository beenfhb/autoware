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
#include <Eigen/Core>
#include <Eigen/LU>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <ros/ros.h>

namespace waypoint_maker {
class WaypointBezierReplannerNode {
public:
  WaypointBezierReplannerNode();
  ~WaypointBezierReplannerNode();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher lane_pub_;
  ros::Subscriber lane_sub_, config_sub_;
  bool replanning_mode_;
  autoware_msgs::LaneArray lane_array_;
  std::string src_lane_;
  std::string dist_lane_;
  void replan(autoware_msgs::LaneArray &lane_array);
  void publishLaneArray();
  void laneCallback(const autoware_msgs::LaneArray::ConstPtr &lane_array);
};

WaypointBezierReplannerNode::WaypointBezierReplannerNode()
    : private_nh_("~"), replanning_mode_(false) {
  private_nh_.param<string>("srcLane", src_lane_, "/based/lane_waypoints_raw");
  private_nh_.param<string>("dstLane", dst_lane_,
                            "/based/lane_waypoints_array");
  private_nh_.param<int>("step", step_, 5);
  private_nh_.param<int>("resampling_num", resampling_num_, 10);
  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>(dist_lane_, 10, true);
  lane_sub_ =
      nh_.subscribe(src_lane_, 1, &WaypointReplannerNode::laneCallback, this);
}
WaypointReplannerNode::~WaypointReplannerNode() {}

void WaypointBezierReplannerNode::replan(autoware_msgs::LaneArray &lane_array) {
  for (auto &el : lane_array.lanes) {
  }
}

void WaypointBezierReplannerNode::publishLaneArray() {
  autoware_msgs::LaneArray array(lane_array_);
  if (replanning_mode_) {
    replan(array);
  }
  lane_pub_.publish(array);
}

void WaypointBezierReplannerNode::laneCallback(
    const autoware_msgs::LaneArray::ConstPtr &lane_array) {
  lane_array_ = *lane_array;
  publishLaneArray();
}

void WaypointBezierReplannerNode::run() { ros::spin(); }
