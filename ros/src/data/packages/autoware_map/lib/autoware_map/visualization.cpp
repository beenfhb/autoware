#ifndef __AUTOWARE_MAP_VISUALIZATION_HPP__
#define __AUTOWARE_MAP_VISUALIZATION_HPP__

#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
// #include <vector_map/vector_map.h>
#include <autoware_map/autoware_map.h>
#include <autoware_map/visualization.h>
#include <autoware_map/util.h>
// void enableMarker(visualization_msgs::Marker& marker);
// void disableMarker(visualization_msgs::Marker& marker);
// bool isValidMarker(const visualization_msgs::Marker& marker);
// void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2);
using autoware_map::AutowareMap;
std_msgs::ColorRGBA createColorRGBA(Color color)
{
    const double COLOR_VALUE_MIN = 0.0;
    const double COLOR_VALUE_MAX = 1.0;
    const double COLOR_VALUE_MEDIAN = 0.5;
    const double COLOR_VALUE_LIGHT_LOW = 0.56;
    const double COLOR_VALUE_LIGHT_HIGH = 0.93;

    std_msgs::ColorRGBA color_rgba;
    color_rgba.r = COLOR_VALUE_MIN;
    color_rgba.g = COLOR_VALUE_MIN;
    color_rgba.b = COLOR_VALUE_MIN;
    color_rgba.a = COLOR_VALUE_MAX;

    switch (color)
    {
        case BLACK:
            break;
        case GRAY:
            color_rgba.r = COLOR_VALUE_MEDIAN;
            color_rgba.g = COLOR_VALUE_MEDIAN;
            color_rgba.b = COLOR_VALUE_MEDIAN;
            break;
        case LIGHT_RED:
            color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.g = COLOR_VALUE_LIGHT_LOW;
            color_rgba.b = COLOR_VALUE_LIGHT_LOW;
            break;
        case LIGHT_GREEN:
            color_rgba.r = COLOR_VALUE_LIGHT_LOW;
            color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.b = COLOR_VALUE_LIGHT_LOW;
            break;
        case LIGHT_BLUE:
            color_rgba.r = COLOR_VALUE_LIGHT_LOW;
            color_rgba.g = COLOR_VALUE_LIGHT_LOW;
            color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
            break;
        case LIGHT_YELLOW:
            color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.b = COLOR_VALUE_LIGHT_LOW;
            break;
        case LIGHT_CYAN:
            color_rgba.r = COLOR_VALUE_LIGHT_LOW;
            color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
            break;
        case LIGHT_MAGENTA:
            color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.g = COLOR_VALUE_LIGHT_LOW;
            color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
            break;
        case RED:
            color_rgba.r = COLOR_VALUE_MAX;
            break;
        case GREEN:
            color_rgba.g = COLOR_VALUE_MAX;
            break;
        case BLUE:
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case YELLOW:
            color_rgba.r = COLOR_VALUE_MAX;
            color_rgba.g = COLOR_VALUE_MAX;
            break;
        case CYAN:
            color_rgba.g = COLOR_VALUE_MAX;
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case MAGENTA:
            color_rgba.r = COLOR_VALUE_MAX;
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case WHITE:
            color_rgba.r = COLOR_VALUE_MAX;
            color_rgba.g = COLOR_VALUE_MAX;
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        default:
            color_rgba.a = COLOR_VALUE_MIN; // hide color from view
            break;
    }

    return color_rgba;
}


void enableMarker(visualization_msgs::Marker& marker)
{
    marker.action = visualization_msgs::Marker::ADD;
}

void disableMarker(visualization_msgs::Marker& marker)
{
    marker.action = visualization_msgs::Marker::DELETE;
}

bool isValidMarker(const visualization_msgs::Marker& marker)
{
    return marker.action == visualization_msgs::Marker::ADD;
}

void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2)
{
    a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}

visualization_msgs::Marker createMarker(const std::string& ns, int id, int type)
{
    visualization_msgs::Marker marker;
    // NOTE: Autoware want to use map messages with or without /use_sim_time.
    // Therefore we don't set marker.header.stamp.
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.lifetime = ros::Duration();
    marker.frame_locked = true;
    disableMarker(marker);
    return marker;
}

// visualization_msgs::Marker createAreaMarkerArray(const std::string& ns, int id, Color color, const AutowareMap& amap,
//   const Area& area){
//     visualization_msgs::MarkerArray marker_array;
//     for (const auto& area : amap.findByFilter([] (const autoware_map_msgs::Area& a){return true; }))
//     {
//         if (area.area_id == 0 || area.point_ids.empty())
//         continue;
//         visualization_msgs::Marker marker = createMarker(ns, area.area_id, visualization_msgs::Marker::LINE_STRIP);
//
//         for(auto point_id : area.point_ids){
//             Point point = amap.findByKey(Key<Point>(point_id));
//             marker.points.push_back(convertPointToGeomPoint(point));
//         }
//         //close area
//         Point first_point = amap.findByKey(Key<Point>(area.point_ids.front()));
//         marker.points.push_back(first_point);
//
//         marker.scale.x = MAKER_SCALE_AREA;
//         marker.color = createColorRGBA(color);
//         enableMarker(marker);
//         marker_array.markers.push_back(marker);
//     }
//     return marker_array;
// }
//
// visualization_msgs::Marker createLaneMarker(const std::string& ns, int id, Color color, const AutowareMap& amap,
//                                             const Lane& lane)
// {
//     visualization_msgs::MarkerArray marker_array;
//     for (const auto lane : amap.findByFilter([] (const autoware_map_msgs::Lane& a){return true; }))
//     {
//         if (area.area_id == 0 || area.point_ids.empty())
//           continue;
//         visualization_msgs::Marker marker = createMarker(ns, lane.lane_id, visualization_msgs::Marker::LINE_STRIP);
//
//     }
//     return marker_array;
// }
//
// visualization_msgs::Marker createLaneSignalLightRelationMarker(const std::string& ns, int id, Color color, const AutowareMap& amap,
//                                             const LaneSignalLightRelation& lane_signal_light_relation){
//   }
// visualization_msgs::Marker createRouteMarker(const std::string& ns, int id, Color color, const AutowareMap& amap,
//                                            const Route& route);
// visualization_msgs::MarkerArray createSignalMarkerArray(const AutowareMap& amap, Color red_color, Color blue_color,
//                                              Color yellow_color, Color other_color);
// visualization_msgs::Marker createWayareaMarker(const std::string& ns, int id, Color color, const AutowareMap& amap,
//                                                  const Wayarea& wayarea);
// visualization_msgs::MarkerArray createWaypointMarkerArray(const AutowareMap& amap, Color color);




visualization_msgs::MarkerArray createWaypointMarkerArray(const AutowareMap& autoware_map, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 1;
    visualization_msgs::Marker marker = createMarker("waypoints", id++, visualization_msgs::Marker::POINTS);
    for ( auto waypoint : autoware_map.findByFilter([] (const autoware_map_msgs::Waypoint &){return true; }))
    {

        marker.points.push_back(convertPointToGeomPoint(getPointFromWaypointId(waypoint.waypoint_id,autoware_map)));
    }
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color = createColorRGBA(color);
    enableMarker(marker);
    marker_array.markers.push_back(marker);
    return marker_array;
    return marker_array;
}

visualization_msgs::MarkerArray createWaypointSignalRelationMarkerArray(const AutowareMap& autoware_map, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 1;
    for ( auto relation : autoware_map.findByFilter([] (const autoware_map_msgs::WaypointSignalRelation &){return true; }))
    {
        autoware_map_msgs::Waypoint waypoint = autoware_map.findByKey(autoware_map::Key<autoware_map_msgs::Waypoint>(relation.waypoint_id));
        autoware_map_msgs::Point waypoint_point = autoware_map.findByKey(autoware_map::Key<autoware_map_msgs::Point>(waypoint.point_id));
        std::vector<autoware_map_msgs::SignalLight> signals = autoware_map.findByFilter([&](autoware_map_msgs::SignalLight light){return light.signal_id == relation.signal_id; });
        if(!signals.empty())
        {
            autoware_map_msgs::Point signal_point = autoware_map.findByKey(autoware_map::Key<autoware_map_msgs::Point>(signals.front().point_id));
            visualization_msgs::Marker marker = createMarker("waypoint_signal_relation", id++, visualization_msgs::Marker::ARROW );
            marker.points.push_back(convertPointToGeomPoint(signal_point));
            marker.points.push_back(convertPointToGeomPoint(waypoint_point));
            marker.scale.x = 0.1; // shaft diameter
            marker.scale.y = 1; //head_diameter
            marker.scale.z = 1; //head_length
            marker.color = createColorRGBA(color);
            enableMarker(marker);
            marker_array.markers.push_back(marker);
        }
    }
    return marker_array;
}
visualization_msgs::MarkerArray createWaypointRelationMarkerArray(const AutowareMap& autoware_map, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 1;
    for ( auto relation : autoware_map.findByFilter([] (const autoware_map_msgs::WaypointRelation &){return true; }))
    {
        autoware_map_msgs::Point point = getPointFromWaypointId(relation.waypoint_id, autoware_map);
        autoware_map_msgs::Point next_point = getPointFromWaypointId(relation.next_waypoint_id, autoware_map);

        visualization_msgs::Marker marker = createMarker("waypoint_relations", id++, visualization_msgs::Marker::ARROW );
        marker.points.push_back(convertPointToGeomPoint(point));
        marker.points.push_back(convertPointToGeomPoint(next_point));
        marker.scale.x = 0.1;     // shaft diameter
        marker.scale.y = 0.3;     //head_diameter
        marker.scale.z = 0.3;     //head_length
        marker.color = createColorRGBA(color);
        enableMarker(marker);
        marker_array.markers.push_back(marker);

    }
    return marker_array;
}
visualization_msgs::MarkerArray createPointMarkerArray(const AutowareMap& autoware_map, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 1;
    visualization_msgs::Marker marker = createMarker("points", id++, visualization_msgs::Marker::POINTS);
    for ( auto point : autoware_map.findByFilter([] (const autoware_map_msgs::Point &){return true; }))
    {
        marker.points.push_back(convertPointToGeomPoint(point));
    }
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color = createColorRGBA(color);
    enableMarker(marker);
    marker_array.markers.push_back(marker);
    return marker_array;
}

#endif
