/*
 *  Copyright (c) 2019, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OPENDRIVE2AUTOWARE_CONVERTER
#define OPENDRIVE2AUTOWARE_CONVERTER

#include "op_planner/internal_road_network.h"
#include "op_planner/MappingHelpers.h"
#include "tinyxml.h"
#include "time.h"


namespace autoware_map
{

enum LINK_TYPE {PREDECESSOR_LINK, SUCCESSOR_LINK, EMPTY_LINK };
enum ELEMENT_TYPE{ROAD_ELEMENT, EMPTY_ELEMENT};
enum CONTACT_POINT{START_POINT, END_POINT, EMPTY_POINT };

class OpenDriveHeader
{

public:
	OpenDriveHeader()
	{
		rev_major_ = 0;
		rev_minor_ = 0;
		north_ = 0;
		south_ = 0;
		east_ = 0;
		west_ = 0;
		max_road_ = 0;
		max_junc_ = 0;
		max_prg_ = 0;
		date_ = time(0);
	}
	OpenDriveHeader(TiXmlElement* main_element)
	{
		if(main_element != nullptr)
		{
			if(main_element->Attribute("revMajor") != nullptr)
				rev_major_ =  strtol(main_element->Attribute("revMajor"), NULL, 10);
			else
				rev_major_ = 0;

			if(main_element->Attribute("revMinor") != nullptr)
				rev_minor_ =  strtol(main_element->Attribute("revMinor"), NULL, 10);
			else
				rev_minor_ = 0;
			if(main_element->Attribute("name") != nullptr)
				name_ = std::string(main_element->Attribute("name"));

			if(main_element->Attribute("version") != nullptr)
				version_ =  std::string(main_element->Attribute("version"));

			if(main_element->Attribute("date") != nullptr)
			{
				struct tm _tm;
				strptime(main_element->Attribute("date"), "%Day %Mon %dd %hh:%mm:%ss %yyyy", &_tm);
				date_ = mktime(&_tm);
			}

			if(main_element->Attribute("north") != nullptr)
				north_ =  strtod(main_element->Attribute("north"), NULL);
			else
				north_ =  0;

			if(main_element->Attribute("south") != nullptr)
				south_ =  strtod(main_element->Attribute("south"), NULL);
			else
				south_ =  0;

			if(main_element->Attribute("east") != nullptr)
				east_ =  strtod(main_element->Attribute("east"), NULL);
			else
				east_ =  0;

			if(main_element->Attribute("west") != nullptr)
				west_ =  strtod(main_element->Attribute("west"), NULL);
			else
				west_ =  0;

			if(main_element->Attribute("maxRoad") != nullptr)
				max_road_ =  strtol(main_element->Attribute("maxRoad"), NULL, 10);
			else
				max_road_ =  0;

			if(main_element->Attribute("maxJunc") != nullptr)
				max_junc_ =  strtol(main_element->Attribute("maxJunc"), NULL, 10);
			else
				max_junc_ =  0;

			if(main_element->Attribute("maxPrg") != nullptr)
				max_prg_ =  strtol(main_element->Attribute("maxPrg"), NULL, 10);
			else
				max_prg_ =  0;

			if(main_element->Attribute("vendor") != nullptr)
				vendor_ = std::string(main_element->Attribute("vendor"));

		}
	}

	int rev_major_;
    int rev_minor_;
    std::string name_ ;
    std::string version_;
    time_t date_ ;
    double north_;
    double south_;
    double east_;
    double west_;
    int max_road_;
    int max_junc_;
    int max_prg_;
    std::string vendor_;
};

class Link
{
public:
	LINK_TYPE link_type;
	ELEMENT_TYPE element_type;
	int element_id;
	CONTACT_POINT contact_point;

	Link(TiXmlElement* main_element)
	{
		if(PlannerHNS::MappingHelpers::GetStringValue(main_element, "").compare("predecessor") == 0)
			link_type = PREDECESSOR_LINK;
		else if(PlannerHNS::MappingHelpers::GetStringValue(main_element, "").compare("successor") == 0)
			link_type = SUCCESSOR_LINK;
		else
			link_type = EMPTY_LINK;

		if(PlannerHNS::MappingHelpers::GetStringAttribute(main_element, "elementType", "").compare("road") == 0)
			element_type = ROAD_ELEMENT;
		else
			element_type = EMPTY_ELEMENT;

		element_id = PlannerHNS::MappingHelpers::GetIntAttribute(main_element, "elementId", 0);

		if(PlannerHNS::MappingHelpers::GetStringAttribute(main_element, "contactPoint", "").compare("start") == 0)
			contact_point = START_POINT;
		else if(PlannerHNS::MappingHelpers::GetStringAttribute(main_element, "contactPoint", "").compare("end") == 0)
			contact_point = END_POINT;
		else
			contact_point = EMPTY_POINT;
	}
};

class OpenDriveRoad
{
public:
	OpenDriveRoad()
	{
		id_ = 0;
		junction_id_ = 0;
		length_ = 0;
	}

	OpenDriveRoad(TiXmlElement* main_element)
	{
		name_ = PlannerHNS::MappingHelpers::GetStringAttribute(main_element, "name", "");
		id_ = PlannerHNS::MappingHelpers::GetIntAttribute(main_element, "id", 0);
		junction_id_ = PlannerHNS::MappingHelpers::GetIntAttribute(main_element, "junction", -1);
		length_ = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "length", 0.0);

		std::vector<TiXmlElement*> link_elements;
		PlannerHNS::MappingHelpers::FindElements("link", main_element, link_elements);
		for(unsigned int i=0; i < link_elements.size(); i++)
		{
			std::vector<TiXmlElement*> pred_elements, succ_elements;

			PlannerHNS::MappingHelpers::FindElements("predecessor", link_elements.at(i), pred_elements);
			for(unsigned int j=0; j < pred_elements.size(); j++)
			{
				predecessor_links_.push_back(Link(pred_elements.at(j)));
			}

			PlannerHNS::MappingHelpers::FindElements("successor", link_elements.at(i), succ_elements);
			for(unsigned int j=0; j < succ_elements.size(); j++)
			{
				successor_links_.push_back(Link(succ_elements.at(j)));
			}
		}

		std::cout << "Road Loaded With ID: " << id_ << " , Length: " << length_ << std::endl;
	}

	std::string name_;
	int id_;
	int junction_id_;
	double length_;
	std::vector<Link> predecessor_links_;
	std::vector<Link> successor_links_;

};


class OpenDrive2AutoConv
{

public:
	OpenDrive2AutoConv();
    ~OpenDrive2AutoConv();
    void loadOpenDRIVE(const std::string& xodr_file, autoware_map::InternalRoadNet& map);

};

}

#endif // OPENDRIVE2AUTOWARE_CONVERTER
