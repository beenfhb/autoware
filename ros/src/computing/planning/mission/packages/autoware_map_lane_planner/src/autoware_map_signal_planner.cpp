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

#include <autoware_map_lane_planner/autoware_map_signal_planner.h>

AutowareMapSignalPlanner::AutowareMapSignalPlanner(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<double>("deceleration", deceleration_, 5.0);
    traffic_light_.traffic_light = traffic_light_.COLOR_UNKNOWN;
    autoware_map_.subscribe(nh_, autoware_map::Category::LANE);
    autoware_map_.subscribe(nh_, autoware_map::Category::WAYPOINT);
    autoware_map_.subscribe(nh_, autoware_map::Category::LANE_SIGNAL_LIGHT_RELATION);
    autoware_map_.subscribe(nh_, autoware_map::Category::WAYPOINT_LANE_RELATION);
    autoware_map_.subscribe(nh_, autoware_map::Category::WAYPOINT_SIGNAL_RELATION);
}

AutowareMapSignalPlanner::~AutowareMapSignalPlanner()
{

}

void AutowareMapSignalPlanner::trafficLightCallback(const autoware_msgs::TrafficLight::ConstPtr msg)
{
    traffic_light_ = *msg;
    return;
}

std::vector<autoware_map_msgs::Waypoint> AutowareMapSignalPlanner::planRedSignalWaypoints(std::vector<autoware_map_msgs::Waypoint> base_waypoints)
{
    std::vector<autoware_map_msgs::Waypoint> waypoints;
    bool stop_waypoint_found = false;
    autoware_map::WaypointSignalRelation stop_relation;
    if(traffic_light_.traffic_light == traffic_light_.COLOR_RED)
    {
        std::vector<autoware_map::Key<autoware_map::WaypointSignalRelation> > relation_keys;
        autoware_map_.getAllKeys(relation_keys);
        std::vector<autoware_map::WaypointSignalRelation> relations;
        for(auto itr = relation_keys.begin(); itr != relation_keys.end(); itr++)
        {
            relations.push_back(autoware_map_.findByKey(*itr));
        }
        for(auto waypoint_itr = base_waypoints.begin(); waypoint_itr != base_waypoints.end(); waypoint_itr++)
        {
            for(auto relation_itr = relations.end(); relation_itr != relations.end(); relation_itr++)
            {
                if(relation_itr->waypoint_id == waypoint_itr->waypoint_id)
                {
                    stop_waypoint_found = true;
                    stop_relation = *relation_itr;
                }
            }
        }
    }
    if(stop_waypoint_found)
    {
        waypoints = base_waypoints;
        bool stopline_passed = false;
        int stop_index;
        for(int i=(base_waypoints.size()-1); i>=0; i--)
        {
            if(!stopline_passed)
            {
                if(waypoints[i].waypoint_id != stop_relation.waypoint_id)
                {
                    waypoints[i].velocity = 0.0;
                }
                else
                {
                    waypoints[i].velocity = 0.0;
                    stopline_passed = true;
                    stop_index = i;
                }
            }
            else
            {
                // update target velocity for stop lines;
                double velocity = (waypoints[i].velocity - std::sqrt(std::pow(waypoints[i].velocity,2)) - 2*deceleration_*(stop_index-i))/deceleration_;
                if(velocity > waypoints[i].velocity)
                {
                    waypoints[i].velocity = velocity;
                }
                else
                {
                    break;
                }
            }
        }
    }
    else
    {
        waypoints = base_waypoints;
    }
    return waypoints;
}

std::vector<autoware_map_msgs::Waypoint> AutowareMapSignalPlanner::plan(std::vector<autoware_map_msgs::Lane> lanes,autoware_map_msgs::Waypoint from ,autoware_map_msgs::Waypoint to)
{
    std::vector<autoware_map_msgs::Waypoint> base_waypoints;
    std::vector<autoware_map_msgs::Waypoint> waypoints;
    base_waypoints = planBaseWaypoints(lanes, from, to);
    waypoints = planRedSignalWaypoints(base_waypoints);
    return waypoints;
}

std::vector<autoware_map_msgs::Waypoint> AutowareMapSignalPlanner::planBaseWaypoints(std::vector<autoware_map_msgs::Lane> lanes,autoware_map_msgs::Waypoint from ,autoware_map_msgs::Waypoint to)
{
    std::vector<autoware_map_msgs::Waypoint> waypoints;
    for(int i=0; i<lanes.size(); i++)
    {
        if(i == 0)
        {
            for(int waypoint_id=from.waypoint_id; waypoint_id<=lanes[i].end_waypoint_id; waypoint_id++)
            {
                autoware_map::Key<autoware_map_msgs::Waypoint> key(waypoint_id);
                waypoints.push_back(autoware_map_.findByKey(key));
            }
        }
        else if(i == (lanes.size()-1))
        {
            for(int waypoint_id=lanes[i].start_waypoint_id; waypoint_id<=to.waypoint_id; waypoint_id++)
            {
                autoware_map::Key<autoware_map_msgs::Waypoint> key(waypoint_id);
                waypoints.push_back(autoware_map_.findByKey(key));
            }
        }
        else
        {
            for(int waypoint_id=lanes[i].start_waypoint_id; waypoint_id<=lanes[i].end_waypoint_id; waypoint_id++)
            {
                autoware_map::Key<autoware_map_msgs::Waypoint> key(waypoint_id);
                waypoints.push_back(autoware_map_.findByKey(key));
            }
        }
    }
    return waypoints;
}