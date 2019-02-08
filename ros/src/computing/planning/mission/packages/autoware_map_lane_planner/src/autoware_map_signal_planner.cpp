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
    traffic_light_.traffic_light = traffic_light_.COLOR_UNKNOWN;
    autoware_map_.subscribe(nh_, autoware_map::Category::LANE_SIGNAL_LIGHT_RELATION);
    autoware_map_.subscribe(nh_, autoware_map::Category::WAYPOINT_LANE_RELATION);
}

AutowareMapSignalPlanner::~AutowareMapSignalPlanner()
{

}

void AutowareMapSignalPlanner::trafficLightCallback(const autoware_msgs::TrafficLight::ConstPtr msg)
{
    traffic_light_ = *msg;
    return;
}

std::vector<autoware_map_msgs::Waypoint> AutowareMapSignalPlanner::plan(std::vector<autoware_map_msgs::Lane> lanes,autoware_map_msgs::Waypoint from ,autoware_map_msgs::Waypoint to)
{
    bool find_stop_lane_ = false;
    int stop_lane_id;
    std::vector<autoware_map_msgs::Waypoint> waypoints;
    if(traffic_light_.traffic_light == traffic_light_.COLOR_RED)
    {
        for(auto lane_itr = lanes.begin(); lane_itr != lanes.end(); lane_itr++)
        {
            autoware_map::Key<autoware_map_msgs::LaneSignalLightRelation> key(lane_itr->lane_id);
            autoware_map_msgs::LaneSignalLightRelation relation = autoware_map_.findByKey(key);
            //failed to find lane_id
            if(relation.lane_id != 0 && relation.signal_light_id != 0)
            {
                stop_lane_id = relation.lane_id;
                find_stop_lane_ = true;
            }
        }
    }
    if(find_stop_lane_)
    {

    }
    else
    {
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
    }
    return waypoints;
}