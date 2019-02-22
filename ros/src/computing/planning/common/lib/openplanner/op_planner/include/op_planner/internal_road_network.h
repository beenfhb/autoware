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

enum ACTION_TYPE {F_ACTION, L_ACTION, R_ACTION, S_ACTION, U_ACTION, NO_ACTION };

//enum LANE_TYPE {PLANE_LANE, INTERSECTION_LANE, CROSSWALK_LANE, LANE_CHANGE_PROHIBITION_LANE, PARKING_PROHIBITION_LANE,
//	RAILROAD_LANE, SIDEWALK_LANE, PARKING_AREA_LANE, UNKNOWN_LANE
//};

enum LANE_TYPE { NONE_LANE, DRIVING_LANE, STOP_LANE, SHOULDER_LANE, BIKING_LANE,
	SIDEWALK_LANE, BORDER_LANE, RESTRICTED_LANE, PARKING_LANE, BIDIRECTIONAL_LANE,
    MEDIAN_LANE, SPECIAL1_LANE, SPECIAL2_LANE, SPECIAL3_LANE, ROADWORKS_LANE,
    TRAM_LANE, RAIL_LANE, ENTRY_LANE, EXIT_LANE, OFFRAMP_LANE, ONRAMP_LANE, PLANE_LANE};

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
	std::shared_ptr<WayPoint> p_left_wp;
	std::shared_ptr<WayPoint> p_right_wp;
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
public:
	int id;
	int start_wp_id;
	int end_wp_id;
	int road_id;
	int map_area_id;
	std::vector<int> from_lanes;
	std::vector<int> to_lanes;
	int number; //lane number in the road segment from left to right
	double speed_limit;
	double width_limit;
	double height_limit;
	double weight_limit;
	double length;
	ACTION_TYPE direction;
	LANE_TYPE type;
	double width;

	std::vector<WayPoint> points;

	std::vector<std::shared_ptr<Lane> > p_to_lanes;
	std::vector<std::shared_ptr<Lane> > p_from_lanes;

	std::shared_ptr<Lane> p_left_lane;
	std::shared_ptr<Lane> p_right_lane;

	Lane()
	{
		id = 0;
		start_wp_id = 0;
		end_wp_id = 0;
		road_id	= 0;
		map_area_id = 0;
		number 	= 0;
		speed_limit = 0;
		width_limit = 0;
		height_limit = 0;
		weight_limit = 0;
		length = 0;
		direction = F_ACTION;
		type = PLANE_LANE;
		width = 0;
	}

};

class InternalRoadNet
{
public:

	std::vector<Lane> lanes;

};

}

#endif
