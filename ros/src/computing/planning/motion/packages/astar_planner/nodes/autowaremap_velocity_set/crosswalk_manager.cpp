#include "crosswalk_manager.hpp"

using Area = autoware_map_msgs::Area;
using Lane = autoware_map_msgs::Lane;
using LaneAttrRelation = autoware_map_msgs::LaneAttrRelation;
using Waypoint = autoware_map_msgs::Waypoint;
using Point = autoware_map_msgs::Point;

CrossWalkManager::CrossWalkManager()
    : enable_multiple_crosswalk_detection_(false)
{
    map_sub_ = nh.subscribe("awmap_stat", 1, &CrossWalkManager::autowareMapCallback, this);
}

void CrossWalkManager::addRelevantCrossWalkIDs(const int &id)
{
    auto itr = std::find(relevant_crosswalk_ids_.begin(), relevant_crosswalk_ids_.end(), id);
    if ( itr == relevant_crosswalk_ids_.end() )
    {
        relevant_crosswalk_ids_.push_back(id);
    }
}

void CrossWalkManager::findRelevantCrossWalks(const int closest_waypoint, const autoware_msgs::Lane &lane,
                                              const int search_distance)
{
    relevant_crosswalk_ids_.clear();
    if (closest_waypoint < 0)
        return;
    double matching_distance_thresh = pow(0.25, 2);

    for (auto &pair : crosswalks_)
    {
        CrossWalkHandler &crosswalk = pair.second;
        crosswalk.clearStoppingWaypointIds();
    }

    // Find nearest cross walk
    for (int num = closest_waypoint; num < closest_waypoint + search_distance && num < (int)lane.waypoints.size(); num++)
    {
        geometry_msgs::Point waypoint = lane.waypoints.at(num).pose.pose.position;
        waypoint.z = 0.0; // ignore Z axis
        for (auto &pair : crosswalks_)
        {
            CrossWalkHandler &crosswalk = pair.second;
            // crosswalk.stopping_waypoint_ids_.clear();
            for(const auto &stopping_point : crosswalk.getStoppingPoints())
            {
                if( calcSquareOfLength(stopping_point, waypoint) <= matching_distance_thresh) {
                    crosswalk.addStoppingWaypointId(num);
                    relevant_crosswalk_ids_.push_back(crosswalk.getId());
                    if (!useMultipleDetection())
                    {
                        return;
                    }
                }
            }
        }
    }
}

std::vector<CrossWalkHandler> CrossWalkManager::getRelevantCrossWalks(const int waypoint_id) const
{
    std::vector<CrossWalkHandler> relevant_crosswalks;
    for(const auto &crosswalk_id : relevant_crosswalk_ids_)
    {
        for( const auto &stopping_id : crosswalks_.at(crosswalk_id).getStoppingIds())
        {
            if(stopping_id == waypoint_id) {
                relevant_crosswalks.push_back(crosswalks_.at(crosswalk_id));
            }
        }
    }
    return relevant_crosswalks;
}
std::vector<CrossWalkHandler> CrossWalkManager::getRelevantCrossWalks() const
{
    std::vector<CrossWalkHandler> relevant_crosswalks;
    for(const auto &crosswalk_id : relevant_crosswalk_ids_)
    {
        relevant_crosswalks.push_back(crosswalks_.at(crosswalk_id));
    }
    return relevant_crosswalks;
}



void CrossWalkManager::autowareMapCallback(const std_msgs::UInt64::ConstPtr &available_category)
{
    autoware_map::category_t required_category = autoware_map::Category::AREA |
                                                 autoware_map::Category::POINT |
                                                 autoware_map::Category::LANE |
                                                 autoware_map::Category::LANE_ATTR_RELATION |
                                                 autoware_map::Category::WAYPOINT;
    //get autoware_map
    autoware_map_.subscribe(nh, available_category->data, ros::Duration(5));
    if(autoware_map_.hasSubscribed(required_category) == false)
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & autoware_map_.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "<< missing_category << std::endl <<
                        "Did not subscribe all required category! Converted vector map might lack some data!");
    }

    crosswalks_.clear();
    relevant_crosswalk_ids_.clear();

    //set up crosswalks
    for( const auto &relation : autoware_map_.findByFilter( [] (LaneAttrRelation r){return r.attribute_type == LaneAttrRelation::CROSS_WALK; }) )
    {
        Area area = autoware_map_.findById<Area>(relation.area_id);
        Lane lane = autoware_map_.findById<Lane>(relation.lane_id);
        Waypoint waypoint = autoware_map_.findById<Waypoint>(lane.start_waypoint_id);
        Point point = autoware_map_.findById<Point>(waypoint.point_id);

        int id = area.area_id;
        geometry_msgs::Point stopping_point;
        stopping_point.x = point.x;
        stopping_point.y = point.y;
        stopping_point.z = 0; //ignore height

        auto crosswalk = crosswalks_.find(id);
        if(crosswalk == crosswalks_.end()) {
            //create new crosswalk object
            std::vector<geometry_msgs::Point> vertices;
            for( auto vertex_id : area.point_ids)
            {
                Point vertex = autoware_map_.findById<Point>(vertex_id);
                geometry_msgs::Point geom_vertex;
                geom_vertex.x = vertex.x;
                geom_vertex.y = vertex.y;
                geom_vertex.z = 0;
                vertices.push_back(geom_vertex);
            }
            CrossWalkHandler c(id, vertices);
            c.addStoppingPoint(stopping_point);
            crosswalks_.insert(std::make_pair(id, c));
        }
        else{
            crosswalk->second.addStoppingPoint(stopping_point);
        }
    }

}
