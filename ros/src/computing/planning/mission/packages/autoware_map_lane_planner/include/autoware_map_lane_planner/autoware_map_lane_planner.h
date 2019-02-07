#ifndef AUTOWARE_MAP_LANE_PLANNER_H_INCLUDED
#define AUTOWARE_MAP_LANE_PLANNER_H_INCLUDED

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
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Int32.h>

//headers in Autoware
#include <autoware_map/autoware_map.h>
#include <autoware_map_lane_planner/lane_network.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/LaneArray.h>

//headers in Boost
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

class AutowareMapLanePlanner
{
public:
    AutowareMapLanePlanner(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~AutowareMapLanePlanner();
private:
    autoware_map::AutowareMap autoware_map_;
    /* ROS related members */
    void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr msg);
    void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr msg);
    ros::Subscriber goal_pose_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Subscriber lane_lane_waypoints_array_sub_;
    ros::Publisher closest_waypoint_pub_;
    ros::Publisher base_waypoints_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string goal_pose_topic_;
    /* values used in Autoware Map Lane Planner */
    geometry_msgs::PoseStamped current_pose_;
    boost::optional<geometry_msgs::PoseStamped> goal_pose_;
    boost::optional<autoware_map_msgs::Waypoint> goal_waypoint_;
    boost::shared_ptr<LaneNetwork> lane_network_ptr_;
    /* parameters for Autoware Map Lane Planner*/
    double search_radius_;
    bool allow_lane_change_;
    /* functions */
    bool findClosestWaypointCandidates(autoware_map_msgs::Waypoint waypoint);
    boost::optional<autoware_map_msgs::Waypoint> findClosestWaypoint();
    bool findGoalWaypointCandidates(autoware_map_msgs::Waypoint waypoint);
    boost::optional<autoware_map_msgs::Waypoint> findGoalWaypoint();
    double getDiffAngle(double from,double to);
    void getRPY(geometry_msgs::Quaternion q, double &roll, double &pitch, double &yaw);
    void plan(geometry_msgs::PoseStamped goal);
    void plan(autoware_msgs::LaneArray goal);
};

#endif