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
 */

#include <gtest/gtest.h>
#include <ros/connection_manager.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

#include "lane_bypass_planner/lane_bypass_planner.h"

class LaneBypassPlannerTestClass {
public:
  LaneBypassPlannerTestClass() {}

  LaneBypassPlanner lbp_;
  autoware_msgs::Lane lbp_lane_;

  ros::NodeHandle nh_;
  ros::Publisher pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("/current_velocity", 0);
  ros::Publisher pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 0);
  ros::Publisher pub_costmap_ = nh_.advertise<nav_msgs::OccupancyGrid>("/semantics/costmap_generator/occupancy_grid", 0);
  ros::Publisher pub_lane_ = nh_.advertise<autoware_msgs::Lane>("/lane_bypass_planner/in_path", 0);
  ros::Publisher pub_num_ = nh_.advertise<std_msgs::Int32>("/force_lane_change_number", 0);

  ros::Subscriber sub_lane_ = nh_.subscribe("/lane_bypass_planner/out_path", 1, &LaneBypassPlannerTestClass::laneCallback, this);


  void publishTwist(double vx, double wz) {
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = vx;
    msg.twist.angular.z = wz;
    pub_twist_.publish(msg);
  }
  void publishPose(double x, double y, double yaw) {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
    quaternionTFToMsg(quaternion, msg.pose.orientation);
    pub_pose_.publish(msg);
  }
  void publishCostmap(double all_cost) {
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "velodyne";
    msg.info.resolution = 1.0;
    msg.info.width = 10;
    msg.info.height = 10;
    msg.info.origin.position.x = -5.0;
    msg.info.origin.position.y = -5.0;
    msg.info.origin.position.z = 0.0;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
    quaternionTFToMsg(quaternion, msg.info.origin.orientation);
    for (int i = 0; i < 100; ++i) {
      msg.data.push_back(all_cost);
    }
    pub_costmap_.publish(msg);
  }

  void publisLane(int size) {
    autoware_msgs::Lane lane;
    lane.header.stamp = ros::Time::now();
    lane.header.frame_id = "map";
    for (int idx = 0; idx < size; idx++) {
      static autoware_msgs::Waypoint wp;
      wp.gid = idx;
      wp.lid = idx;
      wp.pose.pose.position.x = 0.0 + (double)idx;
      wp.pose.pose.position.y = 0.0;
      wp.pose.pose.position.z = 0.0;
      wp.twist.twist.linear.x = 5.0;
      wp.twist.twist.angular.z = 0.0;

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
      quaternionTFToMsg(quaternion, wp.pose.pose.orientation);

      lane.waypoints.push_back(wp);
    }
    pub_lane_.publish(lane);
  }

  void laneCallback(const autoware_msgs::Lane &msg) {
    lbp_lane_ = msg;
  }

  void publisLaneNum(int num) {
    std_msgs::Int32 msg;
    msg.data = num;
    pub_num_.publish(msg);
  }

  int getBestLaneNum() { return lbp_.best_lane_num_; }
  int getCenterLaneNum() { return lbp_.center_lane_num_; }
  int getSubLaneNum() { return lbp_.sub_lane_num_odd_; }
  void enableForceLaneSelect() { lbp_.enable_force_lane_select_ = true; };
  std::shared_ptr<nav_msgs::OccupancyGrid> getCurrentCostmap() {return lbp_.current_costmap_ptr_; };
  std::shared_ptr<autoware_msgs::Lane> getCurrentLane() {return lbp_.current_lane_ptr_; };
  std::shared_ptr<geometry_msgs::TwistStamped> getCurrentTwist() {return lbp_.current_selftwist_ptr_; };
  std::shared_ptr<geometry_msgs::PoseStamped> getCurrentPose() {return lbp_.current_selfpose_ptr_; };
};

