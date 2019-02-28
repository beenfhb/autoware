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
	TiXmlElement* p_head_element = nullptr;
	TiXmlElement* p_element = nullptr;
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

void OpenDrive2AutoConv::GetReferenceLanes(std::vector<PlannerHNS::Lane>& road_lanes, const double& resolution)
{
	std::vector<WayPoint> wp_list;
	int total_count = 0;
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		wp_list.clear();
		roads_list_.at(i).GetReferenceWaypoints(wp_list);

		PlannerHNS::Lane l;
		l.id = 0;
		for(unsigned int j=0; j < wp_list.size(); j++)
		{
			PlannerHNS::WayPoint wp(wp_list.at(j).pose.x,
					wp_list.at(j).pose.y,
					wp_list.at(j).pose.z,
					wp_list.at(j).pose.a);
			wp.id  = wp_list.at(j).id;
			wp.laneId  = wp_list.at(j).lane_id;
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

void OpenDriveRoad::GetRoadLanes(std::vector<Lane>& lanes_list, const double& resolution)
{
	lanes_list.clear();
	for(unsigned int i=0; i < lanes_.size(); i++)
	{
		OpenDriveLane* p_l = &lanes_.at(i);
		if(p_l->type == DRIVING_LANE)
		{
			int n_waypoints = floor(p_l->length / resolution);

			for(int j=0; j< n_waypoints; j++)
			{

			}
		}
	}
}

void OpenDriveRoad::GetReferenceWaypoints(std::vector<WayPoint>& road_wps, const double& resolution)
{

	for(unsigned int i=0; i < geometries_.size(); i++)
	{
		WayPoint p;
		g_lane_id++;
		int n_waypoints = floor(geometries_.at(i).length / resolution);
		double s_inc = geometries_.at(i).s;
		std::cout << " Road ID: " << id_ << "Geometry Index: " << i << ", L: " << geometries_.at(i).length << ", n: " << n_waypoints << std::endl;
		for(int j=0; j< n_waypoints; j++)
		{
			if(geometries_.at(i).GetPoint(s_inc, p))
			{
				p.lane_id = g_lane_id;
				p.id = j + 1;
				road_wps.push_back(p);
				//std::cout << "Point No: " << road_wps.size() << ", X= " << p.pose.x << ", Y= " << p.pose.y <<std::endl;
			}
			s_inc+=resolution;
		}

		double remaining_distance = geometries_.at(i).s+geometries_.at(i).length - s_inc;
		s_inc += remaining_distance;
//		if(geometries_.at(i).GetPoint(s_inc, p))
//		{
//			road_wps.push_back(p);
//			std::cout << "Point No: " << road_wps.size() << ", X= " << p.pose.x << ", Y= " << p.pose.y <<std::endl;
//		}

//		std::cout <<  " -------------- " << std::endl;
//		std::cout <<  " Remaining Distance : " << (geometries_.at(i).s+geometries_.at(i).length) - s_inc << std::endl;
//		std::cout <<  " -------------- " << std::endl;
	}
}

}
