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
	p_element = nullptr;
	PlannerHNS::MappingHelpers::FindFirstElement("header", doc.FirstChildElement(), p_element);
	if(p_element != nullptr)
	{
		OpenDriveHeader header(p_element);
		std::cout << "Final Results, Num:" << p_element << ", main element: " <<  p_element->Value() << std::endl;
	}

	std::cout << " >> Reading Data from OpenDRIVE map file ... " << std::endl;
	elements.clear();
	PlannerHNS::MappingHelpers::FindElements("road", doc.FirstChildElement(), elements);
	std::cout << "Final Results, Num:" << elements.size() << std::endl;
	for(unsigned int i=0; i < elements.size(); i++)
	{
		OpenDriveRoad road(elements.at(i));
	}
}


}
