/*
 * opendrive2autoware_converter_core.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: hatem
 */

#include "opendrive2autoware_converter/opendrive2autoware_converter_core.h"
#include <fstream>

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


	std::cout << " >> Reading Data from OpenDRIVE map file ... " << std::endl;
	std::vector<TiXmlElement*> elements;
	PlannerHNS::MappingHelpers::FindFirstElement("header", doc.FirstChildElement(), elements);
	if(elements.size() > 0)
	{
		p_element = elements.at(0);
		od_header header(p_element);

		std::cout << "Final Results, Num:" << elements.size() << ", main element: " <<  p_element->Value() << std::endl;
	}
}
