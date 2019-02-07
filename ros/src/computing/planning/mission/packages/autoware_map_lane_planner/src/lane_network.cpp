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
    std::function<void(const autoware_map_msgs::LaneRelation&)> relation_func_
        = std::bind(&LaneNetwork::updateLaneRelation,this,std::placeholders::_1);
    map_.subscribe(nh_, autoware_map::Category::LANE);
    map_.subscribe(nh_, autoware_map::Category::LANE_RELATION);
    map_.subscribe(nh_, autoware_map::Category::LANE_CHANGE_RELATION);
    //map_.registerCallback(relation_func_);
}

LaneNetwork::~LaneNetwork()
{

}

void LaneNetwork::updateLane(autoware_map_msgs::Lane lane)
{
    lane_ = lane;
    map_.getAllKeys(lane_keys_);
    if(lane_relation_ && lane_change_relation_ && lane_)
    {
        generateLaneNetwork();
    }
}

void LaneNetwork::updateLaneRelation(autoware_map_msgs::LaneRelation relations)
{
    lane_relation_ = relations;
    map_.getAllKeys(lane_relation_keys_);
    if(lane_relation_ && lane_change_relation_ && lane_)
    {
        generateLaneNetwork();
    }
}

void LaneNetwork::updateLaneChangeRelation(autoware_map_msgs::LaneChangeRelation relations)
{
    lane_change_relation_ = relations;
    map_.getAllKeys(lane_change_relation_keys_);
    if(lane_relation_ && lane_change_relation_ && lane_)
    {
        generateLaneNetwork();
    }
}

void LaneNetwork::generateLaneNetwork()
{
}