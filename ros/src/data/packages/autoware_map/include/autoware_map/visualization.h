#ifndef __AUTOWARE_MAP_VISUALIZATION_H__
#define __AUTOWARE_MAP_VISUALIZATION_H__


#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_map/autoware_map.h>
#include <autoware_map/visualization.h>

enum Color : int
{
    BLACK,
    GRAY,
    LIGHT_RED,
    LIGHT_GREEN,
    LIGHT_BLUE,
    LIGHT_YELLOW,
    LIGHT_CYAN,
    LIGHT_MAGENTA,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    CYAN,
    MAGENTA,
    WHITE
};

std_msgs::ColorRGBA createColorRGBA(Color color);
void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2);
void enableMarker(visualization_msgs::Marker& marker);
void disableMarker(visualization_msgs::Marker& marker);
bool isValidMarker(const visualization_msgs::Marker& marker);
visualization_msgs::Marker createMarker(const std::string& ns, int id, int type);
visualization_msgs::Marker createVectorMarkerFromSignal(const std::string& ns, int id, Color color, const autoware_map::AutowareMap& autoware_map,
                                              const autoware_map_msgs::SignalLight signal_light);
visualization_msgs::MarkerArray createSignalMarkerArray(const autoware_map::AutowareMap& autoware_map, Color red_color = Color::RED, Color blue_color = Color::GREEN,
                                                        Color yellow_color = Color::YELLOW, Color other_color = Color::GRAY);
visualization_msgs::MarkerArray createAreaMarkerArray(const autoware_map::AutowareMap& autoware_map, Color color);
visualization_msgs::MarkerArray createWaypointSignalRelationMarkerArray(const autoware_map::AutowareMap& amap, Color color);
visualization_msgs::MarkerArray createPointMarkerArray(const autoware_map::AutowareMap& amap, Color color);
visualization_msgs::MarkerArray createWaypointMarkerArray(const autoware_map::AutowareMap& autoware_map, Color color, Color stop_color = Color::RED);
visualization_msgs::MarkerArray createWaypointRelationMarkerArray(const autoware_map::AutowareMap& autoware_map, Color color);
#endif
