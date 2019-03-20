#ifndef __CROSSWALK_HPP__
#define __CROSSWALK_HPP__

#include <autoware_map/autoware_map.h>
#include <waypoint_follower/libwaypoint_follower.h>

class CrossWalkHandler {
private:
    const int id_;
    const std::vector<geometry_msgs::Point> vertices_; //vertices of crosswalk area
    std::vector<int> stopping_waypoint_ids_; //waypoint where vehicle must stop
    std::vector<geometry_msgs::Point> stopping_points_; //position where vehicle must stop (first point of the lane with crosswalk attribute)

public:
    CrossWalkHandler(int id, std::vector<geometry_msgs::Point> vertices);

    //getters
    int getId() const;
    std::vector<geometry_msgs::Point> getStoppingPoints() const;
    std::vector<geometry_msgs::Point> getVertices() const;
    std::vector<int> getStoppingIds() const;

    //setters
    void addStoppingWaypointId(const int id);
    void clearStoppingWaypointIds();
    void addStoppingPoint(const geometry_msgs::Point stopping_point);
};

//adds angles. result stays within (M_PI, -M_PI]
double addAngles(double angle1, double angle2);

//determine whether point lies within an area by counting winding number
bool isWithinArea(geometry_msgs::Point point, const std::vector<geometry_msgs::Point> vertices);

inline std::vector<geometry_msgs::Point> calcRelativeCoordinates(std::vector<geometry_msgs::Point> input_points, geometry_msgs::Pose current_pose)
{
    std::vector<geometry_msgs::Point> transformed_points;
    for(auto point : input_points)
    {
        transformed_points.push_back(calcRelativeCoordinate(point, current_pose));
    }
    return transformed_points;
}

#endif
