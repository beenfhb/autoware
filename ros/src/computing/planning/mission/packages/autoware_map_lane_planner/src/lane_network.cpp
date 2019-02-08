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
    map_.subscribe(nh_, autoware_map::Category::WAYPOINT);
    map_.subscribe(nh_, autoware_map::Category::WAYPOINT_LANE_RELATION);
}

LaneNetwork::~LaneNetwork()
{

}

boost::optional<std::vector<autoware_map_msgs::Waypoint> > LaneNetwork::plan(autoware_map_msgs::Waypoint from, autoware_map_msgs::Waypoint to)
{
    boost::optional<std::vector<autoware_map_msgs::Lane> > lanes = planLane(from, to);
    if(!lanes)
    {
        return boost::none;
    }
    std::vector<autoware_map_msgs::Waypoint> waypoints;
    for(int i=0; i<lanes->size(); i++)
    {
        if(i == 0)
        {
            for(int waypoint_id=from.waypoint_id; waypoint_id<=lanes.get()[i].end_waypoint_id; waypoint_id++)
            {
                autoware_map::Key<autoware_map_msgs::Waypoint> key(waypoint_id);
                waypoints.push_back(map_.findByKey(key));
            }
        }
        else if(i == (lanes->size()-1))
        {
            for(int waypoint_id=lanes.get()[i].start_waypoint_id; waypoint_id<=to.waypoint_id; waypoint_id++)
            {
                autoware_map::Key<autoware_map_msgs::Waypoint> key(waypoint_id);
                waypoints.push_back(map_.findByKey(key));
            }
        }
        else
        {
            for(int waypoint_id=lanes.get()[i].start_waypoint_id; waypoint_id<=lanes.get()[i].end_waypoint_id; waypoint_id++)
            {
                autoware_map::Key<autoware_map_msgs::Waypoint> key(waypoint_id);
                waypoints.push_back(map_.findByKey(key));
            }
        }
    }
    return waypoints;
}

boost::optional<std::vector<autoware_map_msgs::Lane> > LaneNetwork::planLane(autoware_map_msgs::Waypoint from, autoware_map_msgs::Waypoint to)
{
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<autoware_map_msgs::Lane> lanes;
    std::vector<autoware_map::Key<autoware_map_msgs::WaypointLaneRelation> > waypoint_lane_relation_keys;
    map_.getAllKeys(waypoint_lane_relation_keys);
    bool from_found = false;
    bool to_found = false;
    Vertex from_vertex;
    Vertex to_vertex;
    //iterate waypoint relation and search lanes in the from and to waypoint
    for(auto itr = waypoint_lane_relation_keys.begin(); itr != waypoint_lane_relation_keys.end(); itr++)
    {
        autoware_map_msgs::WaypointLaneRelation relation = map_.findByKey(*itr);
        if(relation.waypoint_id == from.waypoint_id)
        {
            from_vertex = relation.lane_id;
            from_found = true;
        }
        if(relation.waypoint_id == to.waypoint_id)
        {
            to_vertex = relation.lane_id;
            to_found = true;
        }
    }
    if(from_found && to_found)
    {
        if(previous_to_lane_id_ && previous_from_lane_id_)
        {
            if(previous_to_lane_id_.get() == to_vertex && previous_from_lane_id_.get() == from_vertex)
            {
                return previous_lane_plan_;
            }
        }
        std::vector<Vertex> parents(boost::num_vertices(graph_));
        std::vector<std::size_t> distance(boost::num_vertices(graph_));
        boost::dijkstra_shortest_paths(graph_, from_vertex, boost::predecessor_map(&parents[0]).distance_map(&distance[0]));
        if(parents[to_vertex] == to_vertex)
        {
            ROS_ERROR_STREAM("failed to find path.");
            return boost::none;
        }
        std::deque<Vertex> route;
        for (Vertex v = to_vertex; v != from_vertex; v = parents[v]) 
        {
            route.push_front(v);
        }
        route.push_front(from_vertex);
        for (const Vertex v : route)
        {
            autoware_map::Key<autoware_map_msgs::Lane> key(v);
            lanes.push_back(map_.findByKey(key));
        }
        previous_from_lane_id_ = from_vertex;
        previous_to_lane_id_ = to_vertex;
        previous_lane_plan_ = lanes;
        return lanes;
    }
    ROS_ERROR_STREAM("failed to find start or goal waypoint from lane/waypoint relation");
    return boost::none;
}

void LaneNetwork::updateLane(autoware_map_msgs::LaneArray lane)
{
    std::lock_guard<std::mutex> lock(mtx_);
    lane_ = lane;
    map_.getAllKeys(lane_keys_);
    generateLaneNetwork();
    return;
}

void LaneNetwork::updateLaneRelation(autoware_map_msgs::LaneRelationArray relations)
{
    std::lock_guard<std::mutex> lock(mtx_);
    lane_relation_ = relations;
    map_.getAllKeys(lane_relation_keys_);
    generateLaneNetwork();
    return;
}

void LaneNetwork::updateLaneChangeRelation(autoware_map_msgs::LaneChangeRelationArray relations)
{
    std::lock_guard<std::mutex> lock(mtx_);
    lane_change_relation_ = relations;
    map_.getAllKeys(lane_change_relation_keys_);
    generateLaneNetwork();
    return;
}

void LaneNetwork::enableLaneChange()
{
    std::lock_guard<std::mutex> lock(mtx_);
    allow_lane_change_ = true;
    generateLaneNetwork();
    return;
}

void LaneNetwork::disableLaneChange()
{
    std::lock_guard<std::mutex> lock(mtx_);
    allow_lane_change_ = false;
    generateLaneNetwork();
    return;
}

void LaneNetwork::generateLaneNetwork()
{
    graph_.clear();
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