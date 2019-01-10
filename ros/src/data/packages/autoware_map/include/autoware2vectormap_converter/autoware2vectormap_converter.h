/*
 *  Copyright (c) 2015, Nagoya University
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

#ifndef __AUTOWARE2VECTORMAP_CONVERTER_H__
#define __AUTOWARE2VECTORMAP_CONVERTER_H__
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_map/autoware_map.h>
#include <autoware_map/util.h>

#include <vector_map/vector_map.h>
#include <sys/stat.h>
#include <unordered_map>

//utils
int convertESPGToRef(int epsg);
double addAngles(double angle1, double angle2);
double convertDecimalToDDMMSS(const double decimal);
bool isWithinArea(double x, double y, const std::vector<autoware_map_msgs::Point> vertices);
bool getIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double &intersect_x, double &intersect_y );
void getMinMax(autoware_map_msgs::Point &min, autoware_map_msgs::Point &max, const std::vector<autoware_map_msgs::Point>points);

//visualization
void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2);
template <class T, class U>
U createObjectArray(const std::vector<T> data)
{
    U obj_array;
    // NOTE: Autoware want to use map messages with or without /use_sim_time.
    // Therefore we don't set obj_array.header.stamp.
    // obj_array.header.stamp = ros::Time::now();
    obj_array.header.frame_id = "map";
    obj_array.data = data;
    return obj_array;
}

visualization_msgs::Marker createLinkedLineMarker(const std::string& ns, int id, vector_map::Color color, const vector_map::VectorMap& vmap, const vector_map_msgs::Line& line);
visualization_msgs::MarkerArray createAreaMarkerArray(const vector_map::VectorMap& vmap, vector_map::Color color);
visualization_msgs::MarkerArray createLaneMarkerArray(const vector_map::VectorMap& vmap, vector_map::Color color);
visualization_msgs::MarkerArray createStopLineMarkerArray(const vector_map::VectorMap& vmap, vector_map::Color color);
visualization_msgs::MarkerArray createCrossWalkMarkerArray(const vector_map::VectorMap& vmap, vector_map::Color color);
visualization_msgs::MarkerArray createSignalMarkerArray(const vector_map::VectorMap& vmap, vector_map::Color red_color, vector_map::Color blue_color, vector_map::Color yellow_color, vector_map::Color other_color, vector_map::Color pole_color);
visualization_msgs::MarkerArray createCrossRoadMarkerArray(const vector_map::VectorMap& vmap, vector_map::Color color);

#endif // AUTOWARE_MAP_AUTOWARE_MAP_H
