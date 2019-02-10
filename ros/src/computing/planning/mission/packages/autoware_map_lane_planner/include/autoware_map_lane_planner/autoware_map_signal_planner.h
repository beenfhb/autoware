#ifndef AUTOWARE_MAP_SIGNAL_PLANNER_H_INCLUDED
#define AUTOWARE_MAP_SIGNAL_PLANNER_H_INCLUDED

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

//headers in Autoware
#include <autoware_map/autoware_map.h>
#include <autoware_msgs/TrafficLight.h>

class AutowareMapSignalPlanner
{
public:
    AutowareMapSignalPlanner(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~AutowareMapSignalPlanner();
    std::vector<autoware_map_msgs::Waypoint> plan(std::vector<autoware_map_msgs::Lane> lanes,autoware_map_msgs::Waypoint from ,autoware_map_msgs::Waypoint to);
private:
    std::vector<autoware_map_msgs::Waypoint> planRedSignalWaypoints(std::vector<autoware_map_msgs::Waypoint> base_waypoints);
    std::vector<autoware_map_msgs::Waypoint> planBaseWaypoints(std::vector<autoware_map_msgs::Lane> lanes,autoware_map_msgs::Waypoint from ,autoware_map_msgs::Waypoint to);
    autoware_map::AutowareMap autoware_map_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void trafficLightCallback(const autoware_msgs::TrafficLight::ConstPtr msg);
    autoware_msgs::TrafficLight traffic_light_;
    double deceleration_;
};

#endif