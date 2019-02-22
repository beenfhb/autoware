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
#include <std_msgs/UInt64.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_map/autoware_map.h>
#include <autoware_map/util.h>

#include <vector_map/vector_map.h>
#include <sys/stat.h>
#include <unordered_map>


class Converter
{

public:
    Converter();
    ~Converter();
private:
    ros::NodeHandle nh;

    autoware_map::AutowareMap awmap_;

    std::vector<vector_map_msgs::Point> vmap_points_;
    std::vector<vector_map_msgs::Node> vmap_nodes_;
    std::vector<vector_map_msgs::Area> vmap_areas_;
    std::vector<vector_map_msgs::Line> vmap_lines_;
    std::vector<vector_map_msgs::DTLane> vmap_dtlanes_;
    std::vector<vector_map_msgs::Lane> vmap_lanes_;
    std::vector<vector_map_msgs::CrossRoad> vmap_cross_roads_;
    std::vector<vector_map_msgs::CrossWalk> vmap_cross_walks_;
    std::vector<vector_map_msgs::WayArea> vmap_way_areas_;
    std::vector<vector_map_msgs::Signal> vmap_signals_;
    std::vector<vector_map_msgs::Vector> vmap_vectors_;
    std::vector<vector_map_msgs::Pole> vmap_dummy_poles_;
    std::vector<vector_map_msgs::StopLine> vmap_stop_lines_;
    std::vector<vector_map_msgs::RoadSign> vmap_road_signs_;

    //Publishers & Subscribers
    std::map<std::string,ros::Publisher> vmap_pubs_;
    ros::Publisher vmap_stat_pub_;
    ros::Publisher marker_array_pub_;
    ros::Subscriber awmap_stat_sub_;

    void statusCallback(const std_msgs::UInt64::ConstPtr &available_category);
    void publishVectorMap();

};

struct WaypointWithYaw{
    autoware_map_msgs::Waypoint waypoint;
    autoware_map_msgs::Point point;
    autoware_map_msgs::Point left_point;
    autoware_map_msgs::Point right_point;
    std::vector<double> yaws;
    double yaw_avg;
};

//conversion functions these don't have to be a member function of Converter class since it does not touch member variables
void convertPoint(vector_map_msgs::Point &vmap_point, const autoware_map_msgs::Point awmap_point);
void createPoints(const autoware_map::AutowareMap &awmap, std::vector<vector_map_msgs::Point> &vmap_points);
void createNodes(const autoware_map::AutowareMap &awmap, std::vector<vector_map_msgs::Node> &vmap_nodes);
void createAreas(const autoware_map::AutowareMap &awmap, std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines);
void createCrossRoads(const autoware_map::AutowareMap &awmap, std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads);
int createSquareArea(double x, double y, double z, double length,
                     std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines,
                     std::vector<vector_map_msgs::Point> &vmap_points);
void createCrossWalks(const autoware_map::AutowareMap &awmap, std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks,
                      std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points);
int getJunctionType(const std::vector<autoware_map_msgs::WaypointRelation> awmap_waypoint_relations, std::vector<int> branching_idx, std::vector<int> merging_idx);
void createDTLanes(const autoware_map::AutowareMap &awmap, std::vector<vector_map_msgs::DTLane> &vmap_dtlanes, std::vector<vector_map_msgs::Lane> &vmap_lanes);
void createWayAreas(const autoware_map::AutowareMap &awmap, std::vector<vector_map_msgs::WayArea> &vmap_way_areas);
void createWayAreasFromLanes(const autoware_map::AutowareMap &awmap, std::vector<vector_map_msgs::WayArea> &vmap_way_areas,
                            std::vector<vector_map_msgs::Area> &vmap_areas,
                             std::vector<vector_map_msgs::Line> &vmap_lines,
                             std::vector<vector_map_msgs::Point> &vmap_points );
void createSignals( const autoware_map::AutowareMap &awmap,
                    std::vector<vector_map_msgs::Signal> &vmap_signals,
                    std::vector<vector_map_msgs::Vector> &vmap_vectors,
                    std::vector<vector_map_msgs::Pole> &vmap_dummy_poles);
void createStopLines( const autoware_map::AutowareMap &awmap,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                      std::vector<vector_map_msgs::RoadSign> &vmap_road_signs);
vector_map_msgs::RoadSign createDummyRoadSign(int id);

std::vector<int> findBranchingIdx(const std::vector<autoware_map_msgs::WaypointRelation> relation, int root_index);
std::vector<int> findMergingIdx(const std::vector<autoware_map_msgs::WaypointRelation> relation, int merged_index);

//utils
int convertESPGToRef(int epsg);
double addAngles(double angle1, double angle2);
double convertDecimalToDDMMSS(const double decimal);
bool isWithinArea(double x, double y, const std::vector<autoware_map_msgs::Point> vertices);
bool getIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double &intersect_x, double &intersect_y );
void getMinMax(autoware_map_msgs::Point &min, autoware_map_msgs::Point &max, const std::vector<autoware_map_msgs::Point>points);
int getMaxId(std::vector<vector_map_msgs::Point> points);
int getMaxId(std::vector<vector_map_msgs::Line> lines);
int getMaxId(std::vector<vector_map_msgs::Area> areas);
int getMaxId(std::vector<vector_map_msgs::StopLine> stop_lines);
int getMaxId(std::vector<vector_map_msgs::RoadSign> signs);
int getMaxId(std::vector<vector_map_msgs::WayArea> way_areas);
double getAngleAverage(std::vector<double> angles);

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
visualization_msgs::MarkerArray createWayAreaMarkerArray(const vector_map::VectorMap& vmap, vector_map::Color color);

#endif // AUTOWARE_MAP_AUTOWARE_MAP_H
