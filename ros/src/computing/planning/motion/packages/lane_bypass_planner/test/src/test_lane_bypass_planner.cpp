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
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "test_lane_bypass_planner.h"


class LaneBypassPlannerTestSuite : public ::testing::Test {
public:
  LaneBypassPlannerTestSuite() {
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "map";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "velodyne";
    br.sendTransform(transformStamped);

    transformStamped.header.frame_id = "velodyne";
    transformStamped.child_frame_id = "base_link";
    br.sendTransform(transformStamped);
  }
  ~LaneBypassPlannerTestSuite() {}

  LaneBypassPlannerTestClass test_obj_;
};

TEST_F(LaneBypassPlannerTestSuite, publishBypassLanetest) {

  test_obj_.publishTwist(5.0, 0.0);
  test_obj_.publishPose(0.0, 0.0, 0.0);
  test_obj_.publishCostmap(0.0);
  test_obj_.publisLane(100);

  ros::spinOnce();
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
    
  std::shared_ptr<geometry_msgs::PoseStamped> p1 = test_obj_.getCurrentPose();
  std::shared_ptr<geometry_msgs::TwistStamped> p2 = test_obj_.getCurrentTwist();
  std::shared_ptr<nav_msgs::OccupancyGrid> p3 = test_obj_.getCurrentCostmap();
  std::shared_ptr<autoware_msgs::Lane> p4 = test_obj_.getCurrentLane();
  EXPECT_EQ(1, p1 != nullptr) << "pose ptr should not be null";
  EXPECT_EQ(1, p2 != nullptr) << "twist ptr should not be null";
  EXPECT_EQ(1, p3 != nullptr) << "costmap ptr should not be null";
  EXPECT_EQ(1, p4 != nullptr) << "lane ptr should not be null";


  ASSERT_EQ(100, test_obj_.lbp_lane_.waypoints.size()) << "size should be same as received lane size";
}


TEST_F(LaneBypassPlannerTestSuite, forceLanenumChange) {

  test_obj_.enableForceLaneSelect();
  test_obj_.publishTwist(5.0, 0.0);
  test_obj_.publishPose(0.0, 0.0, 0.0);
  test_obj_.publishCostmap(0.0);
  test_obj_.publisLane(100);
  test_obj_.publisLaneNum(5);
  test_obj_.enableForceLaneSelect();

  ros::spinOnce();
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(5, test_obj_.getBestLaneNum());
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "LaneBypassPlannerTestNode");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}