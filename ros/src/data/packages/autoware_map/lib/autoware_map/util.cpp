#include <autoware_map/autoware_map.h>
#include <autoware_map/util.h>

using autoware_map::Key;

autoware_map_msgs::Point getPointFromWaypointId(int waypoint_id, autoware_map::AutowareMap autoware_map)
{
  autoware_map_msgs::Waypoint waypoint = autoware_map.findByKey(Key<autoware_map_msgs::Waypoint>(waypoint_id));
  return autoware_map.findByKey(Key<autoware_map_msgs::Point>(waypoint.point_id));
}

bool isJapaneseCoordinate(int epsg)
{
    //EPSG CODE 2443~2461 for JGD2000
    //EPSG CODE 6669~6687 for JGD2011
  return (epsg >= 2443 && epsg <= 2461) || (epsg >= 6669 && epsg <= 6687);
}

geometry_msgs::Point convertPointToGeomPoint(const autoware_map_msgs::Point& autoware_point)
{
  // NOTE:
  // We swap x and y axis if
  // Japan Plane Rectangular Coordinate System is used.
  geometry_msgs::Point geom_point;
  if(isJapaneseCoordinate(autoware_point.epsg))
  {
    geom_point.x = autoware_point.y;
    geom_point.y = autoware_point.x;
    geom_point.z = autoware_point.z;
  }else
  {
    geom_point.x = autoware_point.x;
    geom_point.y = autoware_point.y;
    geom_point.z = autoware_point.z;
  }
  return geom_point;
}


std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Lane& obj)
{
    os << obj.lane_id << ","
    << obj.start_waypoint_id << ","
    << obj.end_waypoint_id << ","
    << obj.lane_number << ","
    << obj.num_of_lanes << ","
    << obj.speed_limit << ","
    << obj.length << ","
    << obj.width_limit << ","
    << obj.height_limit << ","
    << obj.weight_limit;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneAttrRelation& obj)
{
    os << obj.lane_id << ","
    << obj.attribute_type << ","
    << obj.area_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneRelation& obj)
{
    os << obj.lane_id << ","
    << obj.next_lane_id << ","
    << obj.blinker;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneSignalLightRelation& obj)
{
    os << obj.lane_id << ","
    << obj.signal_light_id;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneChangeRelation& obj)
{
    os << obj.lane_id << ","
    << obj.next_lane_id << ","
    << obj.blinker;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::OppositeLaneRelation& obj)
{
    os << obj.lane_id << ","
    << obj.opposite_lane_id;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Point& obj)
{
    os << obj.point_id << ","
    << obj.x << ","
    << obj.y << ","
    << obj.z << ","
    << obj.lat << ","
    << obj.lng << ","
    << obj.pcd << ","
    << obj.mgrs << ","
    << obj.epsg;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Area& obj)
{
    os << obj.area_id << ",";
    for ( auto id = obj.point_ids.begin(); id != obj.point_ids.end(); id++)
    {
        os << *id;
        if( id+1 != obj.point_ids.end() )
            os << ":";
    }
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Route& obj)
{
    os << obj.route_id << ","
    << obj.start_waypoint_id << ","
    << obj.end_waypoint_id << ","
    << obj.begin_lane_id << ","
    << obj.finish_lane_id << ","
    << obj.min_lane_width << ","
    << obj.max_lane_width << ","
    << obj.length << ","
    << obj.max_weight;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Signal& obj)
{
    os << obj.signal_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::SignalLight& obj)
{
    os << obj.signal_light_id << ","
    << obj.signal_id << ","
    << obj.point_id << ","
    << obj.horizontal_angle << ","
    << obj.vertical_angle << ","
    << obj.color_type << ","
    << obj.arrow_type;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Wayarea& obj)
{
    os << obj.wayarea_id << ","
    << obj.area_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Waypoint& obj)
{
    os << obj.waypoint_id << ","
    << obj.point_id << ","
    << obj.velocity << ","
    << obj.stop_line << ","
    << obj.width << ","
    << obj.height;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointLaneRelation& obj)
{
    os << obj.waypoint_id << ","
    << obj.lane_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointRelation& obj)
{
    os << obj.waypoint_id << ","
    << obj.next_waypoint_id << ","
    << obj.yaw << ","
    << obj.blinker << ","
    << obj.distance;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointSignalRelation& obj)
{
    os << obj.waypoint_id << ","
    << obj.signal_id;
    return os;
}

std::istream& operator>>(std::istream& is, autoware_map_msgs::Lane& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.start_waypoint_id = std::stoi(columns[1]);
    obj.end_waypoint_id = std::stoi(columns[2]);
    obj.lane_number = std::stoi(columns[3]);
    obj.num_of_lanes = std::stoi(columns[4]);
    obj.speed_limit = std::stod(columns[5]);
    obj.length = std::stod(columns[6]);
    obj.width_limit = std::stod(columns[7]);
    obj.height_limit = std::stod(columns[8]);
    obj.weight_limit = std::stod(columns[9]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneAttrRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.attribute_type = std::stoi(columns[1]);
    obj.area_id = std::stoi(columns[2]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.next_lane_id = std::stoi(columns[1]);
    obj.blinker = std::stoi(columns[2]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneSignalLightRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.signal_light_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneChangeRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.next_lane_id = std::stoi(columns[1]);
    obj.blinker = std::stoi(columns[2]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::OppositeLaneRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.opposite_lane_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Point& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.point_id = std::stoi(columns[0]);
    obj.x = std::stod(columns[1]);
    obj.y = std::stod(columns[2]);
    obj.z = std::stod(columns[3]);
    obj.lat = std::stod(columns[4]);
    obj.lng = std::stod(columns[5]);
    obj.pcd = columns[6];
    try{
        obj.mgrs = std::stoi(columns[7]);
    }
    catch (const std::invalid_argument& e)
    {
        ROS_WARN_STREAM("invalid argument for mgrs: " << e.what());
        obj.mgrs = 0;
    }
    obj.epsg = std::stoi(columns[8]);

    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Area& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.area_id = std::stoi(columns[0]);
    std::stringstream ss(columns[1]);
    while (std::getline(ss, column, ':' )) {
        obj.point_ids.push_back( std::stoi(column) );
    }

    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Route& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.route_id = std::stoi(columns[0]);
    obj.start_waypoint_id = std::stoi(columns[1]);
    obj.end_waypoint_id = std::stoi(columns[2]);
    obj.begin_lane_id = std::stoi(columns[3]);
    obj.finish_lane_id = std::stoi(columns[4]);
    obj.min_lane_width = std::stod(columns[5]);
    obj.max_lane_width = std::stod(columns[6]);
    obj.length = std::stod(columns[7]);
    obj.max_weight = std::stod(columns[8]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Signal& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.signal_id = std::stoi(columns[0]);
    // obj.signal_light_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::SignalLight& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.signal_light_id = std::stoi(columns[0]);
    obj.signal_id = std::stoi(columns[1]);
    obj.point_id = std::stoi(columns[2]);
    obj.horizontal_angle = std::stod(columns[3]);
    obj.vertical_angle = std::stod(columns[4]);
    obj.color_type = std::stoi(columns[5]);
    obj.arrow_type = std::stoi(columns[6]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Wayarea& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.wayarea_id = std::stoi(columns[0]);
    obj.area_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Waypoint& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns[0]);
    obj.point_id = std::stoi(columns[1]);
    obj.velocity = std::stod(columns[2]);
    obj.stop_line = std::stoi(columns[3]);
    obj.width = std::stod(columns[4]);
    obj.height = std::stod(columns[5]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::WaypointLaneRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns[0]);
    obj.lane_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::WaypointRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns[0]);
    obj.next_waypoint_id = std::stoi(columns[1]);
    obj.yaw = std::stod(columns[2]);
    obj.blinker = std::stoi(columns[3]);
    obj.distance = std::stod(columns[4]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::WaypointSignalRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns[0]);
    obj.signal_id = std::stoi(columns[1]);
    return is;
}
