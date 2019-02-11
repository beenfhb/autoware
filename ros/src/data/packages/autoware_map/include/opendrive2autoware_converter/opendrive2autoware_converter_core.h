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

class od_header
{

public:
	od_header()
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
	od_header(TiXmlElement* main_element)
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


class OpenDrive2AutoConv
{

public:
	OpenDrive2AutoConv();
    ~OpenDrive2AutoConv();
    void loadOpenDRIVE(const std::string& xodr_file, autoware_map::InternalRoadNet& map);

};

#endif // OPENDRIVE2AUTOWARE_CONVERTER
