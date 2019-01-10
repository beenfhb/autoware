/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf/transform_datatypes.h>
#include <autoware_map/autoware_map.h>
#include <autoware_map/util.h>

using autoware_map::Key;
namespace autoware_map
{
namespace
{
void updateLane(std::map<Key<Lane>, Lane>& map, const LaneArray& msg)
{
    map = std::map<Key<Lane>, Lane>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(Key<Lane>(item.lane_id), item));
    }
}
void updateLaneAttrRelation(std::map<Key<LaneAttrRelation>, LaneAttrRelation>& map, const LaneAttrRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<Key<LaneAttrRelation>, LaneAttrRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(Key<LaneAttrRelation>(id++), item));
    }
}
void updateLaneRelation(std::map<Key<LaneRelation>, LaneRelation>& map, const LaneRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<Key<LaneRelation>, LaneRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(Key<LaneRelation>(id++), item));
    }
}
void updateLaneSignalLightRelation(std::map<Key<LaneSignalLightRelation>, LaneSignalLightRelation>& map, const LaneSignalLightRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<Key<LaneSignalLightRelation>, LaneSignalLightRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(Key<LaneSignalLightRelation>(id++), item));
    }
}
void updateLaneChangeRelation(std::map<Key<LaneChangeRelation>, LaneChangeRelation>& map, const LaneChangeRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<Key<LaneChangeRelation>, LaneChangeRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(Key<LaneChangeRelation>(id++), item));
    }
}
void updateOppositeLaneRelation(std::map<Key<OppositeLaneRelation>, OppositeLaneRelation>& map, const OppositeLaneRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<Key<OppositeLaneRelation>, OppositeLaneRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(Key<OppositeLaneRelation>(id++), item));
    }
}
void updatePoint(std::map<Key<Point>, Point>& map, const PointArray& msg)
{
    map = std::map<Key<Point>, Point>();
    for (const auto& item : msg.data)
    {
        if (item.point_id == 0)
            continue;
        map.insert(std::make_pair(Key<Point>(item.point_id), item));
    }
}
void updateArea(std::map<Key<Area>, Area>& map, const AreaArray& msg)
{
    map = std::map<Key<Area>, Area>();
    for (const auto& item : msg.data)
    {
        if (item.area_id == 0)
            continue;
        map.insert(std::make_pair(Key<Area>(item.area_id), item));
    }
}
void updateRoute(std::map<Key<Route>, Route>& map, const RouteArray& msg)
{
    map = std::map<Key<Route>, Route>();
    for (const auto& item : msg.data)
    {
        if (item.route_id == 0)
            continue;
        map.insert(std::make_pair(Key<Route>(item.route_id), item));
    }
}
void updateSignal(std::map<Key<Signal>, Signal>& map, const SignalArray& msg)
{
    map = std::map<Key<Signal>, Signal>();
    for (const auto& item : msg.data)
    {
        if (item.signal_id == 0)
            continue;
        map.insert(std::make_pair(Key<Signal>(item.signal_id), item));
    }
}
void updateSignalLight(std::map<Key<SignalLight>, SignalLight>& map, const SignalLightArray& msg)
{
    map = std::map<Key<SignalLight>, SignalLight>();
    for (const auto& item : msg.data)
    {
        if (item.signal_light_id == 0)
            continue;
        map.insert(std::make_pair(Key<SignalLight>(item.signal_light_id), item));
    }
}
void updateWayarea(std::map<Key<Wayarea>, Wayarea>& map, const WayareaArray& msg)
{
    map = std::map<Key<Wayarea>, Wayarea>();
    for (const auto& item : msg.data)
    {
        if (item.wayarea_id == 0)
            continue;
        map.insert(std::make_pair(Key<Wayarea>(item.wayarea_id), item));
    }
}
void updateWaypoint(std::map<Key<Waypoint>, Waypoint>& map, const WaypointArray& msg)
{
    map = std::map<Key<Waypoint>, Waypoint>();
    for (const auto& item : msg.data)
    {
        if (item.waypoint_id == 0)
            continue;
        map.insert(std::make_pair(Key<Waypoint>(item.waypoint_id), item));
    }
}
void updateWaypointRelation(std::map<Key<WaypointRelation>, WaypointRelation>& map, const WaypointRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<Key<WaypointRelation>, WaypointRelation>();
    for (const auto& item : msg.data)
    {
        if (item.waypoint_id == 0)
            continue;
        map.insert(std::make_pair(Key<WaypointRelation>(id++), item));
    }
}
void updateWaypointLaneRelation(std::map<Key<WaypointLaneRelation>, WaypointLaneRelation>& map, const WaypointLaneRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<Key<WaypointLaneRelation>, WaypointLaneRelation>();
    for (const auto& item : msg.data)
    {
        if (item.waypoint_id == 0)
            continue;
        map.insert(std::make_pair(Key<WaypointLaneRelation>(id++), item));
    }
}
void updateWaypointSignalRelation(std::map<Key<WaypointSignalRelation>, WaypointSignalRelation>& map, const WaypointSignalRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<Key<WaypointSignalRelation>, WaypointSignalRelation>();
    for (const auto& item : msg.data)
    {
        if (item.waypoint_id == 0)
            continue;
        map.insert(std::make_pair(Key<WaypointSignalRelation>(id++), item));
    }
}

}   // namespace
bool AutowareMap::hasSubscribed(category_t category) const
{
    if (category & LANE)
    {
        if (lane_.empty())
            return false;
    }
    if (category & LANE_ATTR_RELATION)
    {
        if (lane_attr_relation_.empty())
            return false;
    }
    if (category & LANE_RELATION)
    {
        if (lane_relation_.empty())
            return false;
    }
    if (category & LANE_SIGNAL_LIGHT_RELATION)
    {
        if (lane_signal_light_relation_.empty())
            return false;
    }
    if (category & LANE_CHANGE_RELATION)
    {
        if (lane_change_relation_.empty())
            return false;
    }
    if (category & OPPOSITE_LANE_RELATION)
    {
        if (opposite_lane_relation_.empty())
            return false;
    }
    if (category & POINT)
    {
        if (point_.empty())
            return false;
    }
    if (category & AREA)
    {
        if (area_.empty())
            return false;
    }
    if (category & ROUTE)
    {
        if (route_.empty())
            return false;
    }
    if (category & SIGNAL)
    {
        if (signal_.empty())
            return false;
    }
    if (category & SIGNAL_LIGHT)
    {
        if (signal_light_.empty())
            return false;
    }
    if (category & WAYAREA)
    {
        if (wayarea_.empty())
            return false;
    }
    if (category & WAYPOINT)
    {
        if (waypoint_.empty())
            return false;
    }
    if (category & WAYPOINT_LANE_RELATION)
    {
        if (waypoint_lane_relation_.empty())
            return false;
    }
    if (category & WAYPOINT_RELATION)
    {
        if (waypoint_relation_.empty())
            return false;
    }
    if (category & WAYPOINT_SIGNAL_RELATION)
    {
        if (waypoint_signal_relation_.empty())
            return false;
    }
    return true;
}
category_t AutowareMap::hasSubscribed() const
{
    category_t category=NONE;
    if (!lane_.empty())
        category |= LANE;
    if (!lane_attr_relation_.empty())
        category |= LANE_ATTR_RELATION;
    if (!lane_relation_.empty())
        category |= LANE_RELATION;
    if (!lane_signal_light_relation_.empty())
        category |= LANE_SIGNAL_LIGHT_RELATION;
    if (!lane_change_relation_.empty())
        category |= LANE_CHANGE_RELATION;
    if (!opposite_lane_relation_.empty())
        category |= OPPOSITE_LANE_RELATION;
    if (!point_.empty())
        category |= POINT;
    if (!area_.empty())
        category |= AREA;
    if (!route_.empty())
        category |= ROUTE;
    if (!signal_.empty())
        category |= SIGNAL;
    if (!signal_light_.empty())
        category |= SIGNAL_LIGHT;
    if (!wayarea_.empty())
        category |= WAYAREA;
    if (!waypoint_.empty())
        category |= WAYPOINT;
    if (!waypoint_lane_relation_.empty())
        category |= WAYPOINT_LANE_RELATION;
    if (!waypoint_relation_.empty())
        category |= WAYPOINT_RELATION;
    if (!waypoint_signal_relation_.empty())
        category |= WAYPOINT_SIGNAL_RELATION;

    return category;
}

void AutowareMap::registerSubscriber(ros::NodeHandle& nh, category_t category)
{
    if (category & LANE)
    {
        lane_.registerSubscriber(nh, "/autoware_map_info/lane");
        lane_.registerUpdater(updateLane);
    }
    if (category & LANE_ATTR_RELATION)
    {
        lane_attr_relation_.registerSubscriber(nh, "/autoware_map_info/lane_attr_relation");
        lane_attr_relation_.registerUpdater(updateLaneAttrRelation);
    }
    if (category & LANE_RELATION)
    {
        lane_relation_.registerSubscriber(nh, "/autoware_map_info/lane_relation");
        lane_relation_.registerUpdater(updateLaneRelation);
    }
    if (category & LANE_SIGNAL_LIGHT_RELATION)
    {
        lane_signal_light_relation_.registerSubscriber(nh, "/autoware_map_info/lane_signal_light_relation");
        lane_signal_light_relation_.registerUpdater(updateLaneSignalLightRelation);
    }
    if (category & LANE_CHANGE_RELATION)
    {
        lane_change_relation_.registerSubscriber(nh, "/autoware_map_info/lane_change_relation");
        lane_change_relation_.registerUpdater(updateLaneChangeRelation);
    }
    if (category & OPPOSITE_LANE_RELATION)
    {
        opposite_lane_relation_.registerSubscriber(nh, "/autoware_map_info/opposite_lane_relation");
        opposite_lane_relation_.registerUpdater(updateOppositeLaneRelation);
    }
    if (category & POINT)
    {
        point_.registerSubscriber(nh, "/autoware_map_info/point");
        point_.registerUpdater(updatePoint);
    }
    if (category & AREA)
    {
        area_.registerSubscriber(nh, "/autoware_map_info/area");
        area_.registerUpdater(updateArea);
    }
    if (category & ROUTE)
    {
        route_.registerSubscriber(nh, "/autoware_map_info/route");
        route_.registerUpdater(updateRoute);
    }
    if (category & SIGNAL)
    {
        signal_.registerSubscriber(nh, "/autoware_map_info/signal");
        signal_.registerUpdater(updateSignal);
    }
    if (category & SIGNAL_LIGHT)
    {
        signal_light_.registerSubscriber(nh, "/autoware_map_info/signal_light");
        signal_light_.registerUpdater(updateSignalLight);
    }
    if (category & WAYAREA)
    {
        wayarea_.registerSubscriber(nh, "/autoware_map_info/wayarea");
        wayarea_.registerUpdater(updateWayarea);
    }
    if (category & WAYPOINT)
    {
        waypoint_.registerSubscriber(nh, "/autoware_map_info/waypoint");
        waypoint_.registerUpdater(updateWaypoint);
    }
    if (category & WAYPOINT_LANE_RELATION)
    {
        waypoint_lane_relation_.registerSubscriber(nh, "/autoware_map_info/waypoint_lane_relation");
        waypoint_lane_relation_.registerUpdater(updateWaypointLaneRelation);
    }
    if (category & WAYPOINT_RELATION)
    {
        waypoint_relation_.registerSubscriber(nh, "/autoware_map_info/waypoint_relation");
        waypoint_relation_.registerUpdater(updateWaypointRelation);
    }
    if (category & WAYPOINT_SIGNAL_RELATION)
    {
        waypoint_signal_relation_.registerSubscriber(nh, "/autoware_map_info/waypoint_signal_relation");
        waypoint_signal_relation_.registerUpdater(updateWaypointSignalRelation);
    }

}

AutowareMap::AutowareMap()
{
}


void AutowareMap::subscribe(ros::NodeHandle& nh, category_t category)
{
    registerSubscriber(nh, category);
    ros::Rate rate(10);
    while (ros::ok() && !hasSubscribed(category))
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void AutowareMap::subscribe(ros::NodeHandle& nh, category_t category, const ros::Duration& timeout)
{
    registerSubscriber(nh, category);
    ros::Rate rate(10);
    ros::Time end = ros::Time::now() + timeout;
    while (ros::ok() && !hasSubscribed(category) && ros::Time::now() < end)
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void AutowareMap::subscribe(ros::NodeHandle& nh, category_t category, const size_t max_retries)
{
    size_t tries = 0;
    registerSubscriber(nh, category);
    ros::Rate rate(10);
    while (ros::ok() && !hasSubscribed(category) && tries++ < max_retries)
    {
        ros::spinOnce();
        rate.sleep();
    }
}

template<typename T> // for the member template
T AutowareMap::findById(const int id) const
{
    return findByKey(Key<T>(id));
}
template Lane AutowareMap::findById<Lane>(int) const;
template LaneAttrRelation AutowareMap::findById<LaneAttrRelation>(int) const;
template LaneRelation AutowareMap::findById<LaneRelation>(int) const;
template LaneSignalLightRelation AutowareMap::findById<LaneSignalLightRelation>(int) const;
template LaneChangeRelation AutowareMap::findById<LaneChangeRelation>(int) const;
template OppositeLaneRelation AutowareMap::findById<OppositeLaneRelation>(int) const;
template Point AutowareMap::findById<Point>(int) const;
template Area AutowareMap::findById<Area>(int) const;
template Route AutowareMap::findById<Route>(int) const;
template Signal AutowareMap::findById<Signal>(int) const;
template SignalLight AutowareMap::findById<SignalLight>(int) const;
template Wayarea AutowareMap::findById<Wayarea>(int) const;
template Waypoint AutowareMap::findById<Waypoint>(int) const;
template WaypointRelation AutowareMap::findById<WaypointRelation>(int) const;
template WaypointLaneRelation AutowareMap::findById<WaypointLaneRelation>(int) const;
template WaypointSignalRelation AutowareMap::findById<WaypointSignalRelation>(int) const;


Lane AutowareMap::findByKey(const Key<Lane>& key) const
{
    return lane_.findByKey(key);
}
LaneAttrRelation AutowareMap::findByKey(const Key<LaneAttrRelation>& key) const
{
    return lane_attr_relation_.findByKey(key);
}
LaneRelation AutowareMap::findByKey(const Key<LaneRelation>& key) const
{
    return lane_relation_.findByKey(key);
}
LaneSignalLightRelation AutowareMap::findByKey(const Key<LaneSignalLightRelation>& key) const
{
    return lane_signal_light_relation_.findByKey(key);
}
LaneChangeRelation AutowareMap::findByKey(const Key<LaneChangeRelation>& key) const
{
    return lane_change_relation_.findByKey(key);
}
OppositeLaneRelation AutowareMap::findByKey(const Key<OppositeLaneRelation>& key) const
{
    return opposite_lane_relation_.findByKey(key);
}
Point AutowareMap::findByKey(const Key<Point>& key) const
{
    return point_.findByKey(key);
}
Area AutowareMap::findByKey(const Key<Area>& key) const
{
    return area_.findByKey(key);
}
Route AutowareMap::findByKey(const Key<Route>& key) const
{
    return route_.findByKey(key);
}
Signal AutowareMap::findByKey(const Key<Signal>& key) const
{
    return signal_.findByKey(key);
}
SignalLight AutowareMap::findByKey(const Key<SignalLight>& key) const
{
    return signal_light_.findByKey(key);
}
Wayarea AutowareMap::findByKey(const Key<Wayarea>& key) const
{
    return wayarea_.findByKey(key);
}
Waypoint AutowareMap::findByKey(const Key<Waypoint>& key) const
{
    return waypoint_.findByKey(key);
}
WaypointLaneRelation AutowareMap::findByKey(const Key<WaypointLaneRelation>& key) const
{
    return waypoint_lane_relation_.findByKey(key);
}
WaypointRelation AutowareMap::findByKey(const Key<WaypointRelation>& key) const
{
    return waypoint_relation_.findByKey(key);
}
WaypointSignalRelation AutowareMap::findByKey(const Key<WaypointSignalRelation>& key) const
{
    return waypoint_signal_relation_.findByKey(key);
}
std::vector<Lane> AutowareMap::findByFilter(const Filter<Lane>& filter) const
{
    return lane_.findByFilter(filter);
}
std::vector<LaneAttrRelation> AutowareMap::findByFilter(const Filter<LaneAttrRelation>& filter) const
{
    return lane_attr_relation_.findByFilter(filter);
}
std::vector<LaneRelation> AutowareMap::findByFilter(const Filter<LaneRelation>& filter) const
{
    return lane_relation_.findByFilter(filter);
}
std::vector<LaneSignalLightRelation> AutowareMap::findByFilter(const Filter<LaneSignalLightRelation>& filter) const
{
    return lane_signal_light_relation_.findByFilter(filter);
}
std::vector<LaneChangeRelation> AutowareMap::findByFilter(const Filter<LaneChangeRelation>& filter) const
{
    return lane_change_relation_.findByFilter(filter);
}
std::vector<OppositeLaneRelation> AutowareMap::findByFilter(const Filter<OppositeLaneRelation>& filter) const
{
    return opposite_lane_relation_.findByFilter(filter);
}
std::vector<Point> AutowareMap::findByFilter(const Filter<Point>& filter) const
{
    return point_.findByFilter(filter);
}
std::vector<Area> AutowareMap::findByFilter(const Filter<Area>& filter) const
{
    return area_.findByFilter(filter);
}
std::vector<Route> AutowareMap::findByFilter(const Filter<Route>& filter) const
{
    return route_.findByFilter(filter);
}
std::vector<Signal> AutowareMap::findByFilter(const Filter<Signal>& filter) const
{
    return signal_.findByFilter(filter);
}
std::vector<SignalLight> AutowareMap::findByFilter(const Filter<SignalLight>& filter) const
{
    return signal_light_.findByFilter(filter);
}
std::vector<Wayarea> AutowareMap::findByFilter(const Filter<Wayarea>& filter) const
{
    return wayarea_.findByFilter(filter);
}
std::vector<Waypoint> AutowareMap::findByFilter(const Filter<Waypoint>& filter) const
{
    return waypoint_.findByFilter(filter);
}
std::vector<WaypointLaneRelation> AutowareMap::findByFilter(const Filter<WaypointLaneRelation>& filter) const
{
    return waypoint_lane_relation_.findByFilter(filter);
}
std::vector<WaypointRelation> AutowareMap::findByFilter(const Filter<WaypointRelation>& filter) const
{
    return waypoint_relation_.findByFilter(filter);
}
std::vector<WaypointSignalRelation> AutowareMap::findByFilter(const Filter<WaypointSignalRelation>& filter) const
{
    return waypoint_signal_relation_.findByFilter(filter);
}

void AutowareMap::clear()
{
    lane_.clear();
    lane_attr_relation_.clear();
    lane_relation_.clear();
    lane_signal_light_relation_.clear();
    lane_change_relation_.clear();
    opposite_lane_relation_.clear();
    point_.clear();
    area_.clear();
    route_.clear();
    signal_.clear();
    signal_light_.clear();
    wayarea_.clear();
    waypoint_.clear();
    waypoint_lane_relation_.clear();
    waypoint_relation_.clear();
    waypoint_signal_relation_.clear();
}

} //namespace autoware_map
