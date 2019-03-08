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

#include <iostream>
#include <memory>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <autoware_msgs/Lane.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>

#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class LaneBypassPlanner
{
  public:
    LaneBypassPlanner();
    ~LaneBypassPlanner(){};

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher lane_pub_;
    ros::Subscriber cost_map_sub_, lane_sub_, selfpose_sub_, selftwist_sub_, lane_num_sub_;
    ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;


    std::shared_ptr<nav_msgs::OccupancyGrid> current_costmap_ptr_;
    std::shared_ptr<autoware_msgs::Lane> current_lane_ptr_;
    std::shared_ptr<geometry_msgs::PoseStamped> current_selfpose_ptr_;
    std::shared_ptr<geometry_msgs::TwistStamped> current_selftwist_ptr_;

    int force_lane_change_num_;

    /* params */
    bool enable_smooth_transition_;
    bool enable_force_lane_select_;
    bool enable_replan_when_moving_;
    bool enable_smooth_transition_only_for_cost_calculation_and_vizualization_;
    int sub_lane_num_odd_;
    int cost_check_num_max_;
    double sub_lane_width_;
    double smooth_transit_dist_;

    const double INVALID_COST_;
    const double COSTMAP_OBSTACLE_COST_;

    /* cost function parameters*/
    struct CostWeight
    {
        double be_center;
        double stay_there;
        double stay_there_while;
    };
    CostWeight cost_weight_;
    int best_lane_num_;
    int center_lane_num_;
    ros::Time stay_start_time_;

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void laneCallback(const autoware_msgs::Lane::ConstPtr &msg);
    void lanenumCallback(const std_msgs::Int32 &msg);
    void selfposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void selftwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent &e);


    bool coordinateTransformLane(const autoware_msgs::Lane &in_lane_ptr,
                                 const std::string &target_frame, autoware_msgs::Lane &out_lane_ptr);
    void generateSubLane(const autoware_msgs::Lane &base_lane, const int sub_lane_num,
                         const double sub_lane_width, std::vector<autoware_msgs::Lane> &v_sub_lane);
    void smoothingTransition(const geometry_msgs::PoseStamped &selfpose, const double &smooth_transit_dist,
                             std::vector<autoware_msgs::Lane> &v_sub_lane);
    bool calculateBypassLane(const std::vector<autoware_msgs::Lane> &v_sub_lane,
                             const std::shared_ptr<nav_msgs::OccupancyGrid> current_costmap_ptr_,
                             autoware_msgs::Lane &final_lane, std::vector<double> &v_cost, int &min_index);
    void calculateLaneCost(const std::vector<autoware_msgs::Lane> &v_sub_lane,
                           const std::shared_ptr<nav_msgs::OccupancyGrid> current_costmap_ptr_,
                           const int check_num_max, std::vector<double> &costs);
    bool transformRosPose(const geometry_msgs::PoseStamped &in_pose, const std::string &target_frame, geometry_msgs::PoseStamped &out_pose);

    /* for TEST */
    friend class LaneBypassPlannerTestClass;

    /* debug */
    ros::Publisher debug_sublane_pub_;
    void debugPublishSubLane(const std::vector<autoware_msgs::Lane> &v_sub_lane, const std::vector<double> &v_cost);

};
