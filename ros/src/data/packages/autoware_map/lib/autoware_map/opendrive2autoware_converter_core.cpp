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
	std::vector<TiXmlElement*> elements;
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
	std::cout << "Final Results Roads, Num:" << elements.size() << std::endl;

	roads_list_.clear();
	for(unsigned int i=0; i < elements.size(); i++)
	{
		roads_list_.push_back(OpenDriveRoad(elements.at(i)));
	}

	elements.clear();
	PlannerHNS::MappingHelpers::FindElements("junction", doc.FirstChildElement(), elements);
	std::cout << "Final Results Junctions, Num:" << elements.size() << std::endl;

	junctions_list_.clear();
	for(unsigned int i=0; i < elements.size(); i++)
	{
		junctions_list_.push_back(Junction(elements.at(i)));
	}

	//Connect Roads
	ConnectRoads();


}

OpenDriveRoad* OpenDrive2AutoConv::GetRoadById(const int& _id)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).id_ == _id)
		{
			return &roads_list_.at(i);
		}
	}

	return nullptr;
}

Junction* OpenDrive2AutoConv::GetJunctionById(const int& _id)
{
	for(unsigned int i=0; i < junctions_list_.size(); i++)
	{
		if(junctions_list_.at(i).id_ == _id)
		{
			return &junctions_list_.at(i);
		}
	}

	return nullptr;
}

void OpenDrive2AutoConv::ConnectRoads()
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).predecessor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).predecessor_road_.at(0).link_type == ROAD_LINK)
			{
				OpenDriveRoad* p_pre_road = GetRoadById(roads_list_.at(i).predecessor_road_.at(0).from_road_id);
				if(p_pre_road != nullptr)
				{
					std::vector<Connection> conn_list = p_pre_road->GetFirstSectionConnections();
					for(unsigned k=0; k < conn_list.size(); k++)
					{
						conn_list.at(k).incoming_road_ = roads_list_.at(i).id_;
					}
					roads_list_.at(i).from_roads.insert(roads_list_.at(i).from_roads.begin(), conn_list.begin(), conn_list.end());
				}
			}
		}

		if(roads_list_.at(i).successor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).successor_road_.at(0).link_type == ROAD_LINK)
			{
				OpenDriveRoad* p_suc_road = GetRoadById(roads_list_.at(i).successor_road_.at(0).to_road_id);
				if(p_suc_road != nullptr)
				{
					std::vector<Connection> conn_list = p_suc_road->GetLastSectionConnections();
					for(unsigned k=0; k < conn_list.size(); k++)
					{
						conn_list.at(k).outgoing_road_ = roads_list_.at(i).id_;
					}
					roads_list_.at(i).to_roads.insert(roads_list_.at(i).to_roads.begin(), conn_list.begin(), conn_list.end());
				}
			}
		}
	}

	for(unsigned int i=0; i < junctions_list_.size(); i++)
	{
		for(unsigned int j=0; j < junctions_list_.at(i).connections_.size(); j++)
		{
			OpenDriveRoad* p_from_road = GetRoadById(junctions_list_.at(i).connections_.at(j).incoming_road_);
			OpenDriveRoad* p_to_road   = GetRoadById(junctions_list_.at(i).connections_.at(j).outgoing_road_);

			if(p_from_road != nullptr)
				p_from_road->to_roads.insert(p_from_road->to_roads.begin(), junctions_list_.at(i).connections_.begin(), junctions_list_.at(i).connections_.end());

			if(p_to_road != nullptr)
				p_to_road->from_roads.insert(p_to_road->from_roads.begin(), junctions_list_.at(i).connections_.begin(), junctions_list_.at(i).connections_.end());
		}

	}
}

void OpenDrive2AutoConv::GetMapLanes(std::vector<PlannerHNS::Lane>& all_lanes, const double& resolution)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).GetRoadLanes(all_lanes, resolution);
	}
}

void OpenDrive2AutoConv::GetTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).GetTrafficLights(all_lights);
	}
}

void OpenDrive2AutoConv::GetTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).GetTrafficSigns(all_signs);
	}
}

void OpenDrive2AutoConv::GetStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).GetStopLines(all_stop_lines);
	}
}

}
