/*
 * opendrive2autoware_converter_core.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: hatem
 */

#include "opendrive2autoware_converter/opendrive2autoware_converter_core.h"
#include <fstream>

namespace autoware_map
{



OpenDrive2AutoConv::OpenDrive2AutoConv()
{
}

OpenDrive2AutoConv::~OpenDrive2AutoConv()
{
}

void OpenDrive2AutoConv::loadOpenDRIVE(const std::string& xodr_file,
		autoware_map::InternalRoadNet& map)
{
	//First, Get the main element
	std::vector<TiXmlElement*> elements;

	std::ifstream f(xodr_file.c_str());
	if(!f.good())
	{
		std::cout << "Can't Open OpenDRIVE Map File: (" << xodr_file << ")" << std::endl;
		return;
	}

	std::cout << " >> Loading OpenDRIVE Map file ... " << std::endl;

	TiXmlDocument doc(xodr_file);
	try
	{
		doc.LoadFile();
	}
	catch(std::exception& e)
	{
		std::cout << "OpenDRIVE Custom Reader Error, Can't Load .xodr File, path is: "<<  xodr_file << std::endl;
		std::cout << e.what() << std::endl;
		return;
	}


	std::cout << " >> Reading Header Data from OpenDRIVE map file ... " << std::endl;
	elements.clear();
	PlannerHNS::MappingHelpers::FindFirstElement("header", doc.FirstChildElement(), elements);

	std::cout << "Final Results, Num:" << elements.size() <<std::endl;

	if(elements.size() > 0)
	{
		OpenDriveHeader header(elements.at(0));
		std::cout << "Final Results, Num:" << elements.size() << ", main element: " <<  elements.at(0)->Value() << std::endl;
	}

	std::cout << " >> Reading Data from OpenDRIVE map file ... " << std::endl;
	elements.clear();
	PlannerHNS::MappingHelpers::FindElements("road", doc.FirstChildElement(), elements);
	std::cout << "Final Results, Num:" << elements.size() << std::endl;

	roads_list_.clear();
	for(unsigned int i=0; i < elements.size(); i++)
	{
		roads_list_.push_back(OpenDriveRoad(elements.at(i)));
	}

}

void OpenDrive2AutoConv::GetReferenceLanes(std::vector<PlannerHNS::Lane>& all_lanes, const double& resolution)
{
	std::vector<PlannerHNS::Lane> lanes;
	for(unsigned int i=0; i < 1; i++)
	{
		lanes.clear();
		roads_list_.at(i).GetRoadLanes(lanes, resolution);
		if(lanes.size() > 0)
			all_lanes.insert(all_lanes.end(), lanes.begin(), lanes.end());
	}
}

void OpenDrive2AutoConv::GetReferenceGeometry(std::vector<PlannerHNS::Lane>& road_lanes, const double& resolution)
{
	std::vector<PlannerHNS::WayPoint> wp_list;
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		wp_list.clear();
		roads_list_.at(i).GetReferenceWaypoints(wp_list);

		PlannerHNS::Lane l;
		l.id = 0;
		for(unsigned int j=0; j < wp_list.size(); j++)
		{
			PlannerHNS::WayPoint wp(wp_list.at(j).pos.x,
					wp_list.at(j).pos.y,
					wp_list.at(j).pos.z,
					wp_list.at(j).pos.a);
			wp.id  = wp_list.at(j).id;
			wp.laneId  = wp_list.at(j).laneId;
			if(wp.laneId != l.id)
			{
				if(l.points.size() > 0)
					road_lanes.push_back(l);

				l.points.clear();
				l.id = wp.laneId;
			}

			l.points.push_back(wp);
		}

		if(l.points.size() > 0)
			road_lanes.push_back(l);
	}
}

void OpenDriveRoad::GetRoadLanes(std::vector<PlannerHNS::Lane>& lanes_list, const double& resolution)
{
	lanes_list.clear();
	for(unsigned int i=0; i < lanes_.size(); i++)
	{
		OpenDriveLane* p_l = &lanes_.at(i);
		if(p_l->type == DRIVING_LANE)
		{
			PlannerHNS::Lane op_lane;
			op_lane.id = (id_*1000 + 500) + p_l->id *10;

			int n_waypoints = floor(p_l->length / resolution);
			double s_inc = 0;

			for(int j=0; j< n_waypoints; j++)
			{
				Geometry* p_geom = GetMatchingGeometry(p_l->section_offset + s_inc);

				if(p_geom != nullptr)
				{
					PlannerHNS::WayPoint p;
					p_geom->GetPoint(s_inc, p);
					LaneWidth* p_width = p_l->GetMatchingWidth(s_inc);
					if(p_width != nullptr)
					{
						double edge_w = p_width->a + (p_width->b * s_inc) + (p_width->c * pow(s_inc,2)) + (p_width->d * pow(s_inc,3));
						double center_line_w = edge_w / 2.0;
						if(p_l->dir == LEFT_LANE)
						{
							p.pos.x += center_line_w * cos(p.pos.a+M_PI_2);
							p.pos.y += center_line_w * sin(p.pos.a+M_PI_2);
						}
						else if(p_l->dir == RIGHT_LANE)
						{
							p.pos.x += center_line_w * cos(p.pos.a-M_PI_2);
							p.pos.y += center_line_w * sin(p.pos.a-M_PI_2);
						}

						op_lane.points.push_back(p);
						// add the point
					}
				}
				s_inc += resolution;
			}

			if(op_lane.points.size() > 0)
				lanes_list.push_back(op_lane);
		}
	}
}

Geometry* OpenDriveRoad::GetMatchingGeometry(const double& sOffset)
{
	for(unsigned int i=0; i < geometries_.size(); i++)
	{
		if(sOffset >= geometries_.at(i).s && sOffset < geometries_.at(i).length)
		{
			return &geometries_.at(i);
		}
	}

	return nullptr;
}

void OpenDriveRoad::GetReferenceWaypoints(std::vector<PlannerHNS::WayPoint>& road_wps, const double& resolution)
{

	for(unsigned int i=0; i < geometries_.size(); i++)
	{
		PlannerHNS::WayPoint p;
		g_lane_id++;
		int n_waypoints = floor(geometries_.at(i).length / resolution);
		double s_inc = geometries_.at(i).s;
//		std::cout << " Road ID: " << id_ << "Geometry Index: " << i << ", L: " << geometries_.at(i).length << ", n: " << n_waypoints << std::endl;
		for(int j=0; j< n_waypoints; j++)
		{
			if(geometries_.at(i).GetPoint(s_inc, p))
			{
				p.laneId = g_lane_id;
				p.id = j + 1;
				road_wps.push_back(p);
				//std::cout << "Point No: " << road_wps.size() << ", X= " << p.pos.x << ", Y= " << p.pos.y <<std::endl;
			}
			s_inc+=resolution;
		}

		double remaining_distance = geometries_.at(i).s+geometries_.at(i).length - s_inc;
		if(remaining_distance > 0)
		{
			s_inc += remaining_distance;
			if(geometries_.at(i).GetPoint(s_inc, p))
			{
				road_wps.push_back(p);
	//			std::cout << "Point No: " << road_wps.size() << ", X= " << p.pos.x << ", Y= " << p.pos.y <<std::endl;
			}
		}

//		std::cout <<  " -------------- " << std::endl;
//		std::cout <<  " Remaining Distance : " << (geometries_.at(i).s+geometries_.at(i).length) - s_inc << std::endl;
//		std::cout <<  " -------------- " << std::endl;
	}
}


LaneWidth* 	OpenDriveLane::GetMatchingWidth(const double& sOffset)
{
	if(width_list.size() == 0)
		return nullptr;

	if(width_list.size() == 1)
		return &width_list.at(0);

	for(unsigned int i=1; i < width_list.size(); i++)
	{
		if(sOffset >= width_list.at(i-1).sOffset && sOffset < width_list.at(i).sOffset)
		{
			return &width_list.at(i-1);
		}
	}

	return &width_list.at(width_list.size()-1);
}

}
