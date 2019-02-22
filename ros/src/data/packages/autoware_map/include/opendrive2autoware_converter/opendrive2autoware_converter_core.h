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
enum GEOMETRY_TYPE {LINE_GEOMETRY, SPIRAL_GEOMETRY, ARC_GEOMETRY, POLY3_GEOMETRY,
	PARAM_POLY3_GEOMETRY, UNKNOWN_GEOMETRY };
enum ELEVATION_TYPE {ELEVATION_PROFILE, LATERAL_PROFILE};
enum LANE_DIRECTION {LEFT_LANE, RIGHT_LANE, CENTER_LANE };
enum MARKING_TYPE {BROKEN_MARK, SOLID_MARK, NONE_MARK, UNKNOWN_MARK};

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

//from <width> from <lane> from <left,right,center> from <laneSection> from <road>
class LaneWidth
{
public:
	double sOffset, a, b, c, d;
	LaneWidth(TiXmlElement* main_element)
	{
		sOffset = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "sOffset", 0);
		a = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "a", 0);
		b = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "b", 0);
		c = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "c", 0);
		d = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "d", 0);
	}
};

//from <width> from <lane> from <left,right,center> from <laneSection> from <road>
class RoadMark
{
public:
	double sOffset, width;
	MARKING_TYPE type;
	std::string weight;
	std::string color;

	RoadMark(TiXmlElement* main_element)
	{
		sOffset = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "sOffset", 0);
		width = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "width", 0);
		weight = PlannerHNS::MappingHelpers::GetStringAttribute(main_element, "weight", "");
		color = PlannerHNS::MappingHelpers::GetStringAttribute(main_element, "color", "");
		std::string str_type = PlannerHNS::MappingHelpers::GetStringAttribute(main_element, "type", "");

		if(str_type.compare("broken")==0)
		{
			type = BROKEN_MARK;
		}
		else if(str_type.compare("solid")==0)
		{
			type = SOLID_MARK;
		}
		else if(str_type.compare("none")==0)
		{
			type = NONE_MARK;
		}
		else
		{
			type = UNKNOWN_MARK;
		}
	}
};

//from <laneOffset> from <lanes> from <road>
class LaneOffset
{
public:
	double s, a, b, c, d;
	LaneOffset(TiXmlElement* main_element)
	{
		s = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "s", 0);
		a = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "a", 0);
		b = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "b", 0);
		c = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "c", 0);
		d = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "d", 0);
	}
};

//from <elevation> from <elevationProfile> from <road>
//from <superelevation> from <lateralProfile> from <road>
class Elevation
{
public:
	double s, a, b, c, d;
	ELEVATION_TYPE type;
	Elevation(TiXmlElement* main_element)
	{
		std::string val = PlannerHNS::MappingHelpers::GetStringValue(main_element, "");
		if(val.compare("elevation") == 0)
		{
			type = ELEVATION_PROFILE;
		}
		else if(val.compare("superelevation") == 0)
		{
			type = LATERAL_PROFILE;
		}

		s = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "s", 0);
		a = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "a", 0);
		b = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "b", 0);
		c = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "c", 0);
		d = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "d", 0);
	}
};

//from  <planView> from <road>
class Goemetry
{
public:
	double s,x, y, hdg, length; //genral use 'line'
	double curveStart, curveEnd; // for spiral
	double curvature; //for arc

	GEOMETRY_TYPE type;

	Goemetry(TiXmlElement* main_element)
	{
		type = UNKNOWN_GEOMETRY;
		curvature = 0;
		curveStart = 0;
		curveEnd = 0;
		s = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "s", 0);
		x = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "x", 0);
		y = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "y", 0);
		hdg = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "hdg", 0);
		length = PlannerHNS::MappingHelpers::GetDoubleAttribute(main_element, "length", 0);

		if(main_element != nullptr)
		{
			TiXmlElement* type_element = main_element->FirstChildElement();
			std::string val = PlannerHNS::MappingHelpers::GetStringValue(type_element, "");
			if(val.compare("line") == 0)
			{

			}
			else if(val.compare("arc") == 0)
			{
				curvature = PlannerHNS::MappingHelpers::GetDoubleAttribute(type_element, "curvature", 0);
			}
			else if(val.compare("spiral") == 0)
			{
				curveStart = PlannerHNS::MappingHelpers::GetDoubleAttribute(type_element, "curveStart", 0);
				curveEnd = PlannerHNS::MappingHelpers::GetDoubleAttribute(type_element, "curveEnd", 0);
			}
			else if(val.compare("poly3") == 0)
			{

			}
		}
	}
};

class RoadLink
{
public:
	LINK_TYPE link_type;
	ELEMENT_TYPE element_type;
	int element_id;
	CONTACT_POINT contact_point;

	RoadLink(TiXmlElement* main_element)
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

class LaneLink
{
public:
	LINK_TYPE link_type;
	int id;

	LaneLink(TiXmlElement* main_element)
	{
		if(PlannerHNS::MappingHelpers::GetStringValue(main_element, "").compare("predecessor") == 0)
			link_type = PREDECESSOR_LINK;
		else if(PlannerHNS::MappingHelpers::GetStringValue(main_element, "").compare("successor") == 0)
			link_type = SUCCESSOR_LINK;
		else
			link_type = EMPTY_LINK;

		id = PlannerHNS::MappingHelpers::GetIntAttribute(main_element, "id", 0);
	}
};

//from <lanes> from <road>
class OpenDriveLane
{
public:
	int id;
	int level;
	LANE_DIRECTION dir;
	LANE_TYPE type;
	std::vector<int> fromIds;
	std::vector<int> toIds;
	std::vector<LaneWidth> width_list;
	std::vector<RoadMark> mark_list;
	std::vector<LaneLink> predecessor_lane_;
	std::vector<LaneLink> successor_lane_;

	OpenDriveLane(TiXmlElement* main_element, LANE_DIRECTION _dir)
	{
		dir = _dir;
		id = PlannerHNS::MappingHelpers::GetIntAttribute(main_element, "id", 0);
		level = PlannerHNS::MappingHelpers::GetIntAttribute(main_element, "level", 0);
		std::string str_type = PlannerHNS::MappingHelpers::GetStringAttribute(main_element, "type", "");
		if(str_type.compare("driving")==0)
		{
			type = DRIVING_LANE;
		}
		else if(str_type.compare("border")==0)
		{
			type = BORDER_LANE;
		}
		else
		{
			type = DRIVING_LANE;
		}
//		else if(str_type.compare("driving")==0)
//		{
//			type = DRIVING_LANE;
//		}
//		else if(str_type.compare("driving")==0)
//		{
//			type = DRIVING_LANE;
//		}
//		else if(str_type.compare("driving")==0)
//		{
//			type = DRIVING_LANE;
//		}

		TiXmlElement* link_element = nullptr;
		PlannerHNS::MappingHelpers::FindFirstElement("link", main_element, link_element);
		if(link_element != nullptr)
		{
			std::vector<TiXmlElement*> pred_elements, succ_elements;

			PlannerHNS::MappingHelpers::FindElements("predecessor", link_element, pred_elements);
			for(unsigned int j=0; j < pred_elements.size(); j++)
			{
				predecessor_lane_.push_back(LaneLink(pred_elements.at(j)));
			}

			PlannerHNS::MappingHelpers::FindElements("successor", link_element, succ_elements);
			for(unsigned int j=0; j < succ_elements.size(); j++)
			{
				successor_lane_.push_back(LaneLink(succ_elements.at(j)));
			}
		}
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

		//Read Links
		TiXmlElement* link_element = nullptr;
		PlannerHNS::MappingHelpers::FindFirstElement("link", main_element, link_element);
		if(link_element != nullptr)
		{
			std::vector<TiXmlElement*> pred_elements, succ_elements;

			PlannerHNS::MappingHelpers::FindElements("predecessor", link_element, pred_elements);
			for(unsigned int j=0; j < pred_elements.size(); j++)
			{
				predecessor_road_.push_back(RoadLink(pred_elements.at(j)));
			}

			PlannerHNS::MappingHelpers::FindElements("successor", link_element, succ_elements);
			for(unsigned int j=0; j < succ_elements.size(); j++)
			{
				successor_road_.push_back(RoadLink(succ_elements.at(j)));
			}
		}

		//Read Links
		TiXmlElement* planView = nullptr;
		PlannerHNS::MappingHelpers::FindFirstElement("planView", main_element, planView);
		if(planView != nullptr)
		{
			//Get Geometries
			std::vector<TiXmlElement*> geom_elements;
			PlannerHNS::MappingHelpers::FindElements("geometry", link_element, geom_elements);
			for(unsigned int j=0; j < geom_elements.size(); j++)
			{
				geometries_.push_back(Goemetry(geom_elements.at(j)));
			}
		}

		TiXmlElement* lanes = nullptr;
		PlannerHNS::MappingHelpers::FindFirstElement("lanes", main_element, lanes);
		if(lanes != nullptr)
		{
			//laneOffsets
			std::vector<TiXmlElement*> offsets;
			PlannerHNS::MappingHelpers::FindElements("laneSecrion", lanes, offsets);
			for(unsigned int j=0; j < offsets.size(); j++)
			{
				laneOffsets_.push_back(LaneOffset(offsets.at(j)));
			}

			//laneSections and lanes
			std::vector<TiXmlElement*> sections;
			PlannerHNS::MappingHelpers::FindElements("laneSecrion", lanes, sections);
			for(unsigned int j=0; j < sections.size(); j++)
			{
				TiXmlElement* left_element = nullptr;
				PlannerHNS::MappingHelpers::FindFirstElement("left", sections.at(j), left_element);
				lanes_.push_back(OpenDriveLane(left_element, LEFT_LANE));

				TiXmlElement* center_element = nullptr;
				PlannerHNS::MappingHelpers::FindFirstElement("center", sections.at(j), center_element);
				lanes_.push_back(OpenDriveLane(center_element, CENTER_LANE));

				TiXmlElement* right_element = nullptr;
				PlannerHNS::MappingHelpers::FindFirstElement("right", sections.at(j), right_element);
				lanes_.push_back(OpenDriveLane(right_element, RIGHT_LANE));
			}
		}

		std::cout << "Road Loaded With ID: " << id_ << " , Length: " << length_ << std::endl;
	}

	std::string name_;
	int id_;
	int junction_id_;
	double length_;
	std::vector<RoadLink> predecessor_road_;
	std::vector<RoadLink> successor_road_;
	std::vector<Goemetry> geometries_;
	std::vector<OpenDriveLane> lanes_;
	std::vector<LaneOffset> laneOffsets_;

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
