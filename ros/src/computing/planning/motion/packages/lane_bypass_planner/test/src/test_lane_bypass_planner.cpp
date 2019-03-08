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

/*
テスト項目
０．すべてトピックが来ていて、ちゃんとpathがpubされるか
１．強制変更でちゃんと変わるか確かめる
２．generateSubLaneで指定した本数分の軌道ができているか確認
３．smoothTransitionで、開始点が全て同じ位置か確認
４．すべて障害物コストorすべて0コストで、コストがちゃんと計算されるか確認
*/
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test_lane_bypass_planner.h"


class LaneBypassPlannerTestSuite : public ::testing::Test {
public:
  LaneBypassPlannerTestSuite() {}
  ~LaneBypassPlannerTestSuite() {}

  LaneBypassPlannerTestClass test_obj_;

};

TEST_F(LaneBypassPlannerTestSuite, publishBypassLanetest) {

  test_obj_.publishTwist(5.0, 0.0);
  test_obj_.publishPose(0.0, 0.0, 0.0);
  test_obj_.publishCostmap(0.0);
  test_obj_.publisLane(100);
  ros::spinOnce();
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();
  ros::WallDuration(0.5).sleep();

  ASSERT_EQ(100, test_obj_.lbp_lane_.waypoints.size()) << "size should be same as received lane size";
}


TEST_F(LaneBypassPlannerTestSuite, forceLanenumChange) {

  test_obj_.publishTwist(5.0, 0.0);
  test_obj_.publishPose(0.0, 0.0, 0.0);
  test_obj_.publishCostmap(0.0);
  test_obj_.publisLane(100);
  test_obj_.publisLaneNum(3);
  ros::spinOnce();
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();
  ros::WallDuration(0.5).sleep();

  ASSERT_EQ(3, test_obj_.getBestLaneNum());

  test_obj_.publisLaneNum(5);
  ros::spinOnce();
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();
  ros::WallDuration(0.5).sleep();

  ASSERT_EQ(3, test_obj_.getBestLaneNum());

}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "LaneBypassPlannerTestNode");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}