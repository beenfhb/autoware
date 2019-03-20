#include "crosswalk.hpp"

CrossWalkHandler::CrossWalkHandler(int id, std::vector<geometry_msgs::Point> vertices) :
    id_(id),vertices_(vertices)
{
}

int CrossWalkHandler::getId() const
{
    return id_;
}

std::vector<geometry_msgs::Point> CrossWalkHandler::getStoppingPoints() const
{
    return stopping_points_;
}
std::vector<int> CrossWalkHandler::getStoppingIds() const
{
    return stopping_waypoint_ids_;
}

std::vector<geometry_msgs::Point> CrossWalkHandler::getVertices() const
{
    return vertices_;
}

void CrossWalkHandler::addStoppingWaypointId(const int id)
{
    auto pos = std::find(stopping_waypoint_ids_.begin(), stopping_waypoint_ids_.end(), id);
    if(pos == stopping_waypoint_ids_.end())
        stopping_waypoint_ids_.push_back(id);
}

void CrossWalkHandler::clearStoppingWaypointIds()
{
    stopping_waypoint_ids_.clear();
}

void CrossWalkHandler::addStoppingPoint(const geometry_msgs::Point stopping_point)
{
    stopping_points_.push_back(stopping_point);
}

double addAngles(double angle1, double angle2)
{
    double sum = angle1 + angle2;
    while( sum > M_PI ) sum -= 2 * M_PI;
    while( sum <= -M_PI ) sum += 2 * M_PI;
    return sum;
}

bool isWithinArea(geometry_msgs::Point point, const std::vector<geometry_msgs::Point> vertices)
{
    std::vector<double> angles;
    for (auto pt : vertices)
    {
        if(pt.x == point.x && pt.y== point.y) return false;
        angles.push_back( atan2(pt.y - point.y, pt.x - point.x));
    }

    double angle_sum = 0;
    for (unsigned int idx = 0; idx < vertices.size(); idx++)
    {
        double angle1, angle2;
        angle1 = angles.at(idx);
        if(idx + 1 >= vertices.size())
        {
            angle2 = angles.front();
        }else
        {
            angle2 = angles.at(idx + 1);
        }

        double angle_diff = addAngles(angle1, -angle2);
        angle_sum += angle_diff;
    }

    //allow some precision error
    if(fabs(angle_sum) < 1e-3) return false;
    else return true;
}
