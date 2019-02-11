/// \file internal_road_network.h
/// \brief Autoware Road Network Type
/// \author Hatem Darweesh
/// \date Febraury 10, 2019


#ifndef INTERNALROADNETWORK
#define INTERNALROADNETWORK

#include "op_utility/UtilityH.h"
#include <memory>
#include <string>
#include <vector>
#include <sstream>


namespace autoware_map
{

enum ACTION_TYPE {F_ACTION, L_ACTION, R_ACTION, S_ACTION, U_ACTION, NO_ACTION};

class Lane;
class Point;
class WayPoint;

class Point
{
public:
	double lat, x;
	double lon, y;
	double alt, z;
	double dir, a;

	Point()
	{
		lat = x = 0;
		lon = y = 0;
		alt = z = 0;
		dir = a = 0;
	}

	Point(const double& x, const double& y, const double& z, const double& a)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->a = a;

		lat = 0;
		lon = 0;
		alt = 0;
		dir = 0;
	}

	std::string ToString()
	{
		std::stringstream str;
		str.precision(12);
		str << "X:" << x << ", Y:" << y << ", Z:" << z << ", A:" << a << std::endl;
		str << "Lon:" << lon << ", Lat:" << lat << ", Alt:" << alt << ", Dir:" << dir << std::endl;
		return str.str();
	}
};

class WayPoint
{
public:
	int id;
	Point pose;
	double velocity;
	double width;
	double cost;
	int lane_id;
	int map_area_id;
	int	stop_line_id;
	int left_wp_id;
	int right_wp_id;
	int left_lane_id;
	int right_lane_id;
	std::shared_ptr<Lane> p_lane;
	std::shared_ptr<WayPoint> p_left;
	std::shared_ptr<WayPoint> p_right;
	std::vector<int> 	to_wps;
	std::vector<int> 	from_wps;
	std::vector<std::shared_ptr<WayPoint> > p_to_wps;
	std::vector<std::shared_ptr<WayPoint> > p_from_wps;
	std::vector<std::pair<ACTION_TYPE, double> > action_cost;

	WayPoint()
	{
		id = -1;
		velocity = 0;
		width = 0;
		cost = 0;
		lane_id = -1;
		map_area_id = -1;
		stop_line_id = -1;
		left_wp_id = -1;
		right_wp_id = -1;
		left_lane_id = -1;
		right_lane_id = -1;
	}

	WayPoint(const double& x, const double& y, const double& z, const double& a)
	{
		pose.x = x;
		pose.y = y;
		pose.z = z;
		pose.a = a;

		id = -1;
		velocity = 0;
		width = 0;
		cost = 0;
		lane_id = -1;
		map_area_id = -1;
		stop_line_id = -1;
		left_wp_id = -1;
		right_wp_id = -1;
		left_lane_id = -1;
		right_lane_id = -1;
	}
};

class Lane
{

};

class InternalRoadNet
{
public:

};

}

#endif
