#include <autoware2vectormap_converter/autoware2vectormap_converter.h>

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

using vector_map::isValidMarker;
using vector_map::createVectorMarker;
using vector_map::createAreaMarker;
using vector_map::Color;
using vector_map::VectorMap;

void insertMarkerArray(MarkerArray& a1, const MarkerArray& a2)
{
    a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}

Marker createLinkedLineMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                                  const vector_map_msgs::Line& line)
{
    vector_map_msgs::Area area;
    area.aid = 1; // must set valid aid
    area.slid = line.lid;
    return createAreaMarker(ns, id, color, vmap, area);
}

MarkerArray createAreaMarkerArray(const VectorMap& vmap, Color color)
{
    MarkerArray marker_array;
    int id = 0;
    for (const auto& area : vmap.findByFilter([] (const vector_map_msgs::Area & a){return true; }))
    {
        marker_array.markers.push_back(createAreaMarker("area", id++, color, vmap, area));
    }
    return marker_array;
}

MarkerArray createLaneMarkerArray(const VectorMap& vmap, Color color)
{
    MarkerArray marker_array;
    int id = 1;
    for (const auto& lane : vmap.findByFilter([] (const vector_map_msgs::Lane & l){return true; }))
    {
        vector_map_msgs::Node node1 = vmap.findByKey(vector_map::Key<vector_map_msgs::Node>(lane.bnid));
        vector_map_msgs::Node node2 = vmap.findByKey(vector_map::Key<vector_map_msgs::Node>(lane.fnid));
        vector_map_msgs::Line line;
        line.lid = id;
        line.bpid = node1.pid;
        line.fpid = node2.pid;
        line.blid = 0;
        line.flid = 0;

        Marker marker = createLineMarker("lane", id++, color, vmap, line);
        if (isValidMarker(marker))
            marker_array.markers.push_back(marker);
    }
    return marker_array;
}

MarkerArray createStopLineMarkerArray(const VectorMap& vmap, Color color)
{
    MarkerArray marker_array;
    int id = 0;
    for (const auto& stop_line : vmap.findByFilter([] (const vector_map_msgs::StopLine & stop_line){return true; }))
    {

        if (stop_line.lid == 0)
        {
            ROS_ERROR_STREAM("[createStopLineMarkerArray] invalid stop_line: " << stop_line);
            continue;
        }

        vector_map_msgs::Line line = vmap.findByKey(vector_map::Key<vector_map_msgs::Line>(stop_line.lid));
        if (line.lid == 0)
        {
            ROS_ERROR_STREAM("[createStopLineMarkerArray] invalid line: " << line);
            continue;
        }

        if (line.blid == 0) // if beginning line
        {
            Marker marker = createLinkedLineMarker("stop_line", id++, color, vmap, line);
            if (isValidMarker(marker))
                marker_array.markers.push_back(marker);
            else
                ROS_ERROR_STREAM("[createStopLineMarkerArray] failed createLinkedLineMarker: " << line);
        }
    }
    return marker_array;
}

MarkerArray createCrossWalkMarkerArray(const VectorMap& vmap, Color color)
{
    MarkerArray marker_array;
    int id = 0;
    for (const auto& cross_walk : vmap.findByFilter([] (const vector_map_msgs::CrossWalk & cross_walk){return true; }))
    {
        if (cross_walk.aid == 0)
        {
            ROS_ERROR_STREAM("[createCrossWalkMarkerArray] invalid cross_walk: " << cross_walk);
            continue;
        }

        vector_map_msgs::Area area = vmap.findByKey(vector_map::Key<vector_map_msgs::Area>(cross_walk.aid));
        if (area.aid == 0)
        {
            ROS_ERROR_STREAM("[createCrossWalkMarkerArray] invalid area: " << area);
            continue;
        }

        Marker marker = createAreaMarker("cross_walk", id++, color, vmap, area);
        if (isValidMarker(marker))
            marker_array.markers.push_back(marker);
        else
            ROS_ERROR_STREAM("[createCrossWalkMarkerArray] failed createAreaMarker: " << area);
    }
    return marker_array;
}

MarkerArray createSignalMarkerArray(const VectorMap& vmap, Color red_color, Color blue_color,
                                                        Color yellow_color, Color other_color, Color pole_color)
{
    MarkerArray marker_array;
    int id = 0;
    for (const auto& signal : vmap.findByFilter([] (const vector_map_msgs::Signal & signal){return true; }))
    {
        if (signal.vid == 0)
        {
            ROS_ERROR_STREAM("[createSignalMarkerArray] invalid signal: " << signal);
            continue;
        }

        vector_map_msgs::Vector vector = vmap.findByKey(vector_map::Key<vector_map_msgs::Vector>(signal.vid));
        if (vector.vid == 0)
        {
            ROS_ERROR_STREAM("[createSignalMarkerArray] invalid vector: " << vector);
            continue;
        }

        vector_map_msgs::Pole pole;
        if (signal.plid != 0)
        {
            pole = vmap.findByKey(vector_map::Key<vector_map_msgs::Pole>(signal.plid));
            if (pole.plid == 0)
            {
                ROS_ERROR_STREAM("[createSignalMarkerArray] invalid pole: " << pole);
                continue;
            }
        }

        Marker vector_marker;
        switch (signal.type)
        {
            case vector_map_msgs::Signal::RED:
            case vector_map_msgs::Signal::PEDESTRIAN_RED:
                vector_marker = createVectorMarker("signal", id++, red_color, vmap, vector);
                break;
            case vector_map_msgs::Signal::BLUE:
            case vector_map_msgs::Signal::PEDESTRIAN_BLUE:
                vector_marker = createVectorMarker("signal", id++, blue_color, vmap, vector);
                break;
            case vector_map_msgs::Signal::YELLOW:
                vector_marker = createVectorMarker("signal", id++, yellow_color, vmap, vector);
                break;
            case vector_map_msgs::Signal::RED_LEFT:
                vector_marker = createVectorMarker("signal", id++, Color::LIGHT_RED, vmap, vector);
                break;
            case vector_map_msgs::Signal::BLUE_LEFT:
                vector_marker = createVectorMarker("signal", id++, Color::LIGHT_GREEN, vmap, vector);
                break;
            case vector_map_msgs::Signal::YELLOW_LEFT:
                vector_marker = createVectorMarker("signal", id++, Color::LIGHT_YELLOW, vmap, vector);
                break;
            case vector_map_msgs::Signal::OTHER:
                vector_marker = createVectorMarker("signal", id++, other_color, vmap, vector);
                break;
            default:
                ROS_WARN_STREAM("[createSignalMarkerArray] unknown signal.type: " << signal.type << " Creating Marker as OTHER.");
                vector_marker = createVectorMarker("signal", id++, Color::GRAY, vmap, vector);
                break;
        }
        if (isValidMarker(vector_marker))
            marker_array.markers.push_back(vector_marker);
        else
            ROS_ERROR_STREAM("[createSignalMarkerArray] failed createVectorMarker: " << vector);
    }
    return marker_array;
}

MarkerArray createCrossRoadMarkerArray(const VectorMap& vmap, Color color)
{
    MarkerArray marker_array;
    int id = 0;
    for (const auto& cross_road : vmap.findByFilter([] (const vector_map_msgs::CrossRoad & cross_road){return true; }))
    {
        if (cross_road.aid == 0)
        {
            ROS_ERROR_STREAM("[createCrossRoadMarkerArray] invalid cross_road: " << cross_road);
            continue;
        }

        vector_map_msgs::Area area = vmap.findByKey(vector_map::Key<vector_map_msgs::Area>(cross_road.aid));
        if (area.aid == 0)
        {
            ROS_ERROR_STREAM("[createCrossRoadMarkerArray] invalid area: " << area);
            continue;
        }

        Marker marker = createAreaMarker("cross_road", id++, color, vmap, area);
        if (isValidMarker(marker))
            marker_array.markers.push_back(marker);
        else
            ROS_ERROR_STREAM("[createCrossRoadMarkerArray] failed createAreaMarker: " << area);
    }
    return marker_array;
}
