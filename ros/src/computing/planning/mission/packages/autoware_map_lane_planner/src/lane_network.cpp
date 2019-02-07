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

#include <autoware_map_lane_planner/lane_network.h>

LaneNetwork::LaneNetwork(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    allow_lane_change_ = false;
    std::function<void(const autoware_map_msgs::LaneArray&)> lane_func_
        = std::bind(&LaneNetwork::updateLane,this,std::placeholders::_1);
    map_.registerCallback(lane_func_);
    std::function<void(const autoware_map_msgs::LaneRelationArray&)> lane_relation_func_
        = std::bind(&LaneNetwork::updateLaneRelation,this,std::placeholders::_1);
    map_.registerCallback(lane_relation_func_);
    std::function<void(const autoware_map_msgs::LaneChangeRelationArray&)> lane_change_relation_func_
        = std::bind(&LaneNetwork::updateLaneChangeRelation,this,std::placeholders::_1);
    map_.registerCallback(lane_change_relation_func_);
    map_.subscribe(nh_, autoware_map::Category::LANE);
    map_.subscribe(nh_, autoware_map::Category::LANE_RELATION);
    map_.subscribe(nh_, autoware_map::Category::LANE_CHANGE_RELATION);
}

LaneNetwork::~LaneNetwork()
{

}

void LaneNetwork::updateLane(autoware_map_msgs::LaneArray lane)
{
    lane_ = lane;
    map_.getAllKeys(lane_keys_);
    generateLaneNetwork();
    return;
}

void LaneNetwork::updateLaneRelation(autoware_map_msgs::LaneRelationArray relations)
{
    lane_relation_ = relations;
    map_.getAllKeys(lane_relation_keys_);
    generateLaneNetwork();
    return;
}

void LaneNetwork::updateLaneChangeRelation(autoware_map_msgs::LaneChangeRelationArray relations)
{
    lane_change_relation_ = relations;
    map_.getAllKeys(lane_change_relation_keys_);
    generateLaneNetwork();
    return;
}

void LaneNetwork::enableLaneChange()
{
    allow_lane_change_ = true;
    generateLaneNetwork();
    return;
}

void LaneNetwork::disableLaneChange()
{
    allow_lane_change_ = false;
    generateLaneNetwork();
    return;
}

void LaneNetwork::generateLaneNetwork()
{
    graph_.clear();
    if(lane_relation_ && lane_change_relation_ && lane_)
    {
        //Edge : lane_id -> lane_id
        std::vector<Edge> edges;
        //distance (number of waypoints in the each lane)
        std::vector<int> distance;
        for(auto itr = lane_relation_keys_.begin(); itr != lane_relation_keys_.end(); itr++)
        {
            autoware_map_msgs::LaneRelation relation = map_.findByKey(*itr);
            Edge edge(relation.lane_id,relation.next_lane_id);
            edges.push_back(edge);
            autoware_map::Key<autoware_map_msgs::Lane> lane_key(relation.lane_id);
            autoware_map_msgs::Lane lane = map_.findByKey(lane_key);
            distance.push_back(std::abs(lane.end_waypoint_id-lane.start_waypoint_id));
        }
        if(allow_lane_change_)
        {
            for(auto itr = lane_change_relation_keys_.begin(); itr != lane_change_relation_keys_.end(); itr++)
            {
                autoware_map_msgs::LaneChangeRelation relation = map_.findByKey(*itr);
                Edge edge(relation.lane_id,relation.next_lane_id);
                edges.push_back(edge);
                autoware_map::Key<autoware_map_msgs::Lane> lane_key(relation.lane_id);
                autoware_map_msgs::Lane lane = map_.findByKey(lane_key);
                distance.push_back(std::abs(lane.end_waypoint_id-lane.start_waypoint_id));
            }
        }
        graph_ = Graph(edges.begin(), edges.end(), distance.begin(), distance.size());
    }
}