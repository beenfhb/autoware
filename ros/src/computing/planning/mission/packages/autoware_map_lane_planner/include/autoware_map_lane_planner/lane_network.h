#ifndef LANE_NETWORK_H_INCLUDED
#define LANE_NETWORK_H_INCLUDED

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

//headers in boost
#include <boost/optional.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

//headers in Autoware
#include <autoware_map/autoware_map.h>

//Lane Network Graph
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
    boost::no_property, boost::property<boost::edge_weight_t, int> > Graph;
//Edge : lane_id -> lane_id
typedef std::pair<int, int> Edge;
//Vertex : lane_id
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

class LaneNetwork
{
public:
    LaneNetwork(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~LaneNetwork();
    void updateLane(autoware_map_msgs::Lane lane);
    void updateLaneRelation(autoware_map_msgs::LaneRelation relations);
    void updateLaneChangeRelation(autoware_map_msgs::LaneChangeRelation relations);
    void generateLaneNetwork();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    autoware_map::AutowareMap map_;
    boost::optional<autoware_map_msgs::Lane> lane_;
    boost::optional<autoware_map_msgs::LaneRelation> lane_relation_;
    boost::optional<autoware_map_msgs::LaneChangeRelation> lane_change_relation_;
    //Edge : lane_id -> lane_id
    std::vector<Edge> edges_;
    //distance (number of waypoints in the each lane)
    std::vector<int> distance_;
};

#endif  //LANE_NETWORK_H_INCLUDED