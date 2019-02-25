#include <autoware2vectormap_converter/autoware2vectormap_converter.h>
using vector_map::VectorMap;
using autoware_map::AutowareMap;


Converter::Converter()
{
    vmap_pubs_["point"]= nh.advertise<vector_map_msgs::PointArray>("vector_map_info/point", 1, true);
    vmap_pubs_["vector"] = nh.advertise<vector_map_msgs::VectorArray>("vector_map_info/vector", 1, true);
    vmap_pubs_["line"] = nh.advertise<vector_map_msgs::LineArray>("vector_map_info/line", 1, true);
    vmap_pubs_["area"] = nh.advertise<vector_map_msgs::AreaArray>("vector_map_info/area", 1, true);
    vmap_pubs_["dtlane"] = nh.advertise<vector_map_msgs::DTLaneArray>("vector_map_info/dtlane", 1, true);
    vmap_pubs_["node"] = nh.advertise<vector_map_msgs::NodeArray>("vector_map_info/node", 1, true);
    vmap_pubs_["lane"] = nh.advertise<vector_map_msgs::LaneArray>("vector_map_info/lane", 1, true);
    vmap_pubs_["way_area"] = nh.advertise<vector_map_msgs::WayAreaArray>("vector_map_info/way_area", 1, true);
    vmap_pubs_["stop_line"] = nh.advertise<vector_map_msgs::StopLineArray>("vector_map_info/stop_line", 1, true);
    vmap_pubs_["cross_walk"] = nh.advertise<vector_map_msgs::CrossWalkArray>("vector_map_info/cross_walk", 1, true);
    vmap_pubs_["signal"] = nh.advertise<vector_map_msgs::SignalArray>("vector_map_info/signal", 1, true);
    vmap_pubs_["pole"] = nh.advertise<vector_map_msgs::PoleArray>("vector_map_info/pole", 1, true);
    vmap_pubs_["cross_road"] = nh.advertise<vector_map_msgs::CrossRoadArray>("vector_map_info/cross_road", 1, true);
    vmap_pubs_["road_sign"] = nh.advertise<vector_map_msgs::RoadSignArray>("vector_map_info/road_sign", 1, true);

    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vector_map", 1, true);
    vmap_stat_pub_ = nh.advertise<std_msgs::Bool>("vmap_stat", 1, true);

    awmap_stat_sub_ = nh.subscribe("awmap_stat", 1, &Converter::statusCallback, this);

}

Converter::~Converter()
{
}

void Converter::statusCallback(const std_msgs::UInt64::ConstPtr &available_category)
{
    autoware_map::category_t awmap_required_category = autoware_map::Category::AREA |
                                                       autoware_map::Category::POINT |
                                                       autoware_map::Category::LANE |
                                                       autoware_map::Category::LANE_ATTR_RELATION |
                                                       autoware_map::Category::LANE_SIGNAL_LIGHT_RELATION |
                                                       autoware_map::Category::SIGNAL_LIGHT |
                                                       autoware_map::Category::WAYAREA |
                                                       autoware_map::Category::WAYPOINT |
                                                       autoware_map::Category::WAYPOINT_RELATION |
                                                       autoware_map::Category::WAYPOINT_LANE_RELATION |
                                                       autoware_map::Category::WAYPOINT_SIGNAL_RELATION;

    //get autoware_map
    awmap_.subscribe(nh, available_category->data, ros::Duration(5));
    if(awmap_.hasSubscribed(awmap_required_category) == false)
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(awmap_required_category - (awmap_required_category & awmap_.hasSubscribed()));
        ROS_WARN_STREAM("missing category : " << missing_category << std::endl <<
                        "Did not subscribe all required category! Converted vector map might lack some data!");
    }

    ROS_INFO_STREAM("start conversion!");
    std_msgs::Bool vmap_stat;
    vmap_stat.data = false;
    vmap_stat_pub_.publish(vmap_stat);
    //do conversions
    createPoints(awmap_, vmap_points_);
    createNodes(awmap_, vmap_nodes_);
    createAreas(awmap_, vmap_areas_, vmap_lines_);
    createCrossRoads(awmap_, vmap_cross_roads_);
    createDTLanes( awmap_, vmap_dtlanes_,vmap_lanes_);
    createCrossWalks(awmap_, vmap_cross_walks_, vmap_areas_, vmap_lines_, vmap_points_);
    createWayAreas(awmap_, vmap_way_areas_);
    createWayAreasFromLanes(awmap_, vmap_way_areas_, vmap_areas_, vmap_lines_, vmap_points_);
    createSignals(awmap_, vmap_signals_, vmap_vectors_,vmap_dummy_poles_);
    createStopLines(awmap_, vmap_lines_, vmap_points_, vmap_stop_lines_, vmap_road_signs_);

    //output converted map
    publishVectorMap();
    vmap_stat.data = true;
    vmap_stat_pub_.publish(vmap_stat);
    ROS_INFO_STREAM("Published converted vector map!");

    //release memory
    awmap_.clear();
    vmap_points_.clear();
    vmap_nodes_.clear();
    vmap_areas_.clear();
    vmap_lines_.clear();
    vmap_dtlanes_.clear();
    vmap_lanes_.clear();
    vmap_cross_roads_.clear();
    vmap_cross_walks_.clear();
    vmap_way_areas_.clear();
    vmap_signals_.clear();
    vmap_vectors_.clear();
    vmap_dummy_poles_.clear();
    vmap_stop_lines_.clear();
    vmap_road_signs_.clear();
}

void Converter::publishVectorMap()
{
    vector_map::category_t category = vector_map::Category::NONE;
    if(!vmap_points_.empty())
    {
        vmap_pubs_["point"].publish( createObjectArray<vector_map_msgs::Point, vector_map_msgs::PointArray>(vmap_points_));
        category |= vector_map::Category::POINT;
    }
    if(!vmap_vectors_.empty())
    {
        vmap_pubs_["vector"].publish( createObjectArray<vector_map_msgs::Vector, vector_map_msgs::VectorArray>(vmap_vectors_));
        category |= vector_map::Category::VECTOR;
    }
    if(!vmap_lines_.empty())
    {
        vmap_pubs_["line"].publish( createObjectArray<vector_map_msgs::Line, vector_map_msgs::LineArray>(vmap_lines_));
        category |= vector_map::Category::LINE;
    }
    if(!vmap_areas_.empty())
    {
        vmap_pubs_["area"].publish( createObjectArray<vector_map_msgs::Area, vector_map_msgs::AreaArray>(vmap_areas_));
        category |= vector_map::Category::AREA;
    }
    if(!vmap_dtlanes_.empty())
    {
        vmap_pubs_["dtlane"].publish( createObjectArray<vector_map_msgs::DTLane, vector_map_msgs::DTLaneArray>(vmap_dtlanes_));
        category |= vector_map::Category::DTLANE;
    }
    if(!vmap_nodes_.empty())
    {
        vmap_pubs_["node"].publish( createObjectArray<vector_map_msgs::Node, vector_map_msgs::NodeArray>(vmap_nodes_));
        category |= vector_map::Category::NODE;
    }
    if(!vmap_lanes_.empty())
    {
        vmap_pubs_["lane"].publish( createObjectArray<vector_map_msgs::Lane, vector_map_msgs::LaneArray>(vmap_lanes_));
        category |= vector_map::Category::LANE;
    }
    if(!vmap_way_areas_.empty())
    {
        vmap_pubs_["way_area"].publish( createObjectArray<vector_map_msgs::WayArea, vector_map_msgs::WayAreaArray>(vmap_way_areas_));
        category |= vector_map::Category::WAY_AREA;
    }
    if(!vmap_stop_lines_.empty())
    {
        vmap_pubs_["stop_line"].publish( createObjectArray<vector_map_msgs::StopLine, vector_map_msgs::StopLineArray>(vmap_stop_lines_));
        category |= vector_map::Category::STOP_LINE;
    }
    if(!vmap_cross_walks_.empty())
    {
        vmap_pubs_["cross_walk"].publish( createObjectArray<vector_map_msgs::CrossWalk, vector_map_msgs::CrossWalkArray>(vmap_cross_walks_));
        category |= vector_map::Category::CROSS_WALK;
    }
    if(!vmap_signals_.empty())
    {
        vmap_pubs_["signal"].publish( createObjectArray<vector_map_msgs::Signal, vector_map_msgs::SignalArray>(vmap_signals_));
        category |= vector_map::Category::SIGNAL;
    }
    if(!vmap_cross_roads_.empty())
    {
        vmap_pubs_["cross_road"].publish( createObjectArray<vector_map_msgs::CrossRoad, vector_map_msgs::CrossRoadArray>(vmap_cross_roads_));
        category |= vector_map::Category::CROSS_ROAD;
    }
    if(!vmap_dummy_poles_.empty())
    {
        vmap_pubs_["pole"].publish( createObjectArray<vector_map_msgs::Pole, vector_map_msgs::PoleArray>(vmap_dummy_poles_));
        category |= vector_map::Category::POLE;
    }
    if(!vmap_road_signs_.empty())
    {
        vmap_pubs_["road_sign"].publish( createObjectArray<vector_map_msgs::RoadSign, vector_map_msgs::RoadSignArray>(vmap_road_signs_));
        category |= vector_map::Category::ROAD_SIGN;
    }
    vector_map::VectorMap vmap;
    vmap.subscribe(nh, category);

    visualization_msgs::MarkerArray marker_array;
    insertMarkerArray(marker_array, createStopLineMarkerArray(vmap, vector_map::Color::WHITE));
    insertMarkerArray(marker_array, createCrossWalkMarkerArray(vmap, vector_map::Color::WHITE));
    insertMarkerArray(marker_array, createSignalMarkerArray(vmap, vector_map::Color::RED, vector_map::Color::BLUE, vector_map::Color::YELLOW, vector_map::Color::CYAN, vector_map::Color::GRAY));
    insertMarkerArray(marker_array, createCrossRoadMarkerArray(vmap, vector_map::Color::LIGHT_GREEN));
    insertMarkerArray(marker_array, createWayAreaMarkerArray(vmap, vector_map::Color::LIGHT_CYAN));
    // insertMarkerArray(marker_array, createAreaMarkerArray(vmap, vector_map::Color::WHITE));
    insertMarkerArray(marker_array, createLaneMarkerArray(vmap, vector_map::Color::YELLOW));

    marker_array_pub_.publish(marker_array);
}

void convertPoint(vector_map_msgs::Point &vmap_point,const autoware_map_msgs::Point awmap_point)
{
    if(isJapaneseCoordinate(awmap_point.epsg))
    {
        vmap_point.bx = awmap_point.x;
        vmap_point.ly = awmap_point.y;
    }
    else{
        vmap_point.bx = awmap_point.y;
        vmap_point.ly = awmap_point.x;
    }
    vmap_point.ref = convertESPGToRef(awmap_point.epsg);
    vmap_point.pid = awmap_point.point_id;
    vmap_point.b = convertDecimalToDDMMSS( awmap_point.lat );
    vmap_point.l = convertDecimalToDDMMSS( awmap_point.lng );
    vmap_point.h = awmap_point.z;

    //cannot convert mcodes from autoware_map_format
    vmap_point.mcode1 = 0;
    vmap_point.mcode2 = 0;
    vmap_point.mcode3 = 0;
}

void createAreas(const AutowareMap &awmap, std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines)
{
    //return if enough information is not subscribed
    autoware_map::category_t required_category = autoware_map::Category::POINT |
                                                 autoware_map::Category::AREA;
    if(!awmap.hasSubscribed(required_category) )
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & awmap.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "
                        << missing_category << std::endl
                        << "skipping conversion for area");
        return;
    }

    int line_id = getMaxId(vmap_lines) + 1;
    for ( auto awmap_area : awmap.findByFilter( [&](autoware_map_msgs::Area a){return true; }))
    {
        vector_map_msgs::Area vmap_area;
        vmap_area.aid = awmap_area.area_id;
        vmap_area.slid = line_id;

        //create lines that represent area
        auto end_itr = awmap_area.point_ids.end();
        auto begin_itr = awmap_area.point_ids.begin();
        //assumes that point id of corresponding points in AutowareMap and VectorMap are the same. (please check createPoints function)
        for (auto point_itr = begin_itr; point_itr != end_itr; point_itr++)
        {
            vector_map_msgs::Line line;
            line.lid =line_id;
            line.bpid = *point_itr;

            if(point_itr == begin_itr)
            {
                line.blid = 0;
            }
            else{
                line.blid = line_id - 1;
            }

            if(point_itr + 1 == end_itr)
            {
                line.flid = 0;
                line.fpid = *begin_itr; //close the loop
            }
            else{
                line.flid = line_id + 1;
                line.fpid = *(point_itr + 1);
            }

            vmap_lines.push_back(line);
            vmap_area.elid = line_id;
            line_id++;
        }
        vmap_areas.push_back(vmap_area);
    }
}

void createCrossRoads(const AutowareMap &awmap, std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads)
{
    //return if enough information is not subscribed
    autoware_map::category_t required_category = autoware_map::Category::POINT |
                                                 autoware_map::Category::AREA |
                                                 autoware_map::Category::LANE_ATTR_RELATION;
    if(!awmap.hasSubscribed(required_category) )
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & awmap.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "
                        << missing_category << std::endl
                        << "skipping conversion for cross_roads");
        return;
    }

    unsigned int id = 1;
    for ( auto awmap_relation : awmap.findByFilter( [&](autoware_map_msgs::LaneAttrRelation lar){return lar.attribute_type == autoware_map::LaneAttrRelation::INTERSECTION; }) )
    {
        //check whether same cross_road is already created
        if(std::find_if(vmap_cross_roads.begin(), vmap_cross_roads.end(), [&](vector_map_msgs::CrossRoad cr){return cr.aid == awmap_relation.area_id; }) != vmap_cross_roads.end())
        {
            continue;
        }
        vector_map_msgs::CrossRoad cross_road;
        cross_road.id = id++;
        cross_road.aid = awmap_relation.area_id;
        cross_road.linkid = 0;
        vmap_cross_roads.push_back(cross_road);
    }
}

int createSquareArea(double x, double y, double z, double length,
                     std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines,
                     std::vector<vector_map_msgs::Point> &vmap_points)
{
    vector_map_msgs::Point v1, v2, v3, v4;
    int point_id = getMaxId(vmap_points) + 1;
    v1.pid = point_id++;
    v1.bx = x - length / 2;
    v1.ly = y - length / 2;
    v1.h = z;

    v2.pid = point_id++;
    v2.bx = x - length / 2;
    v2.ly = y + length / 2;
    v2.h = z;

    v3.pid = point_id++;
    v3.bx = x + length / 2;
    v3.ly = y + length / 2;
    v3.h = z;

    v4.pid = point_id++;
    v4.bx = x + length / 2;
    v4.ly = y - length / 2;
    v4.h = z;

    vmap_points.push_back(v1);
    vmap_points.push_back(v2);
    vmap_points.push_back(v3);
    vmap_points.push_back(v4);

    vector_map_msgs::Line line;
    int lid, start_lid;
    lid =start_lid= getMaxId(vmap_lines) + 1;
    line.lid = lid;
    line.bpid = v1.pid;
    line.fpid = v2.pid;
    line.blid = 0;
    line.flid = lid + 1;
    vmap_lines.push_back(line);
    lid++;
    line.lid = lid;
    line.bpid = v2.pid;
    line.fpid = v3.pid;
    line.blid = lid - 1;
    line.flid = lid + 1;
    vmap_lines.push_back(line);
    lid++;
    line.lid = lid;
    line.bpid = v3.pid;
    line.fpid = v4.pid;
    line.blid = lid - 1;
    line.flid = lid + 1;
    vmap_lines.push_back(line);
    lid++;
    line.lid = lid;
    line.bpid = v4.pid;
    line.fpid = v1.pid;
    line.blid = lid - 1;
    line.flid = 0;
    vmap_lines.push_back(line);

    vector_map_msgs::Area area;
    area.aid = getMaxId(vmap_areas) + 1;
    area.slid = start_lid;
    area.elid = lid;
    vmap_areas.push_back(area);

    return area.aid;
}

void createCrossWalks(const AutowareMap &awmap, std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks,
                      std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points)
{
    unsigned int id = 1;
    for( auto awmap_relation : awmap.findByFilter([&](const autoware_map_msgs::LaneAttrRelation lar){ return lar.attribute_type == autoware_map::LaneAttrRelation::CROSS_WALK; }) )
    {
        //skip if area is already added
        if(std::find_if(vmap_cross_walks.begin(), vmap_cross_walks.end(), [&](vector_map_msgs::CrossWalk cw){return cw.aid == awmap_relation.area_id; }) != vmap_cross_walks.end())
        {
            continue;
        }

        int parent_id = id++;
        vector_map_msgs::CrossWalk cross_walk;
        cross_walk.id = parent_id;
        cross_walk.aid = awmap_relation.area_id;
        cross_walk.type = 0;
        cross_walk.linkid = 0;
        vmap_cross_walks.push_back(cross_walk);

        cross_walk.id = id++;
        cross_walk.aid = awmap_relation.area_id;
        cross_walk.type = 1;
        cross_walk.bdid = parent_id;
        cross_walk.linkid = 0;
        vmap_cross_walks.push_back(cross_walk);

        //create regions for detection area
        autoware_map_msgs::Area awmap_area = awmap.findById<autoware_map_msgs::Area>(awmap_relation.area_id);
        std::vector<autoware_map_msgs::Point> vertices;
        for ( int vertex : awmap_area.point_ids)
        {
            vertices.push_back(awmap.findById<autoware_map_msgs::Point>(vertex));
        }

        autoware_map_msgs::Point min,max;
        getMinMax(min,max,vertices);

        double resolution = 2.0;
        for (double x = min.x - resolution; x < max.x + resolution; x += resolution / 2)
        {
            for (double y = min.y - resolution; y < max.y + resolution; y += resolution / 2)
            {
                if( isWithinArea(x,y,vertices) )
                {
                    int area_id;
                    if(isJapaneseCoordinate(vertices.front().epsg))
                    {
                        area_id = createSquareArea(x,y,min.z,resolution, vmap_areas, vmap_lines, vmap_points);
                    }
                    else
                    {
                        area_id = createSquareArea(y,x,min.z,resolution, vmap_areas, vmap_lines, vmap_points);
                    }

                    cross_walk.id = id++;
                    cross_walk.aid = area_id;
                    cross_walk.type = 1;
                    cross_walk.bdid = parent_id;
                    cross_walk.linkid = 0;
                    vmap_cross_walks.push_back(cross_walk);
                }
            }
        }
    }
}

void createPoints(const AutowareMap &awmap, std::vector<vector_map_msgs::Point> &vmap_points)
{
    //return if enough information is not subscribed
    autoware_map::Category required_category = autoware_map::Category::POINT;
    if(!awmap.hasSubscribed(required_category) )
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & awmap.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "
                        << missing_category << std::endl
                        << "skipping conversion for points");
        return;
    }


    bool epsg_fail_flag = false;
    for ( auto awmap_pt : awmap.findByFilter( [&](autoware_map_msgs::Point pt){return true; }) )
    {
        if(isJapaneseCoordinate(awmap_pt.epsg) && !epsg_fail_flag) {
            epsg_fail_flag =true;
            ROS_WARN_STREAM("no corresponding Japanese Plane Rectangular CS Number for specified epsg value" );
        }
        vector_map_msgs::Point vmap_point;
        convertPoint(vmap_point, awmap_pt);
        vmap_points.push_back(vmap_point);
    }
}

void createNodes(const AutowareMap &awmap, std::vector<vector_map_msgs::Node> &vmap_nodes)
{
    //return if enough information is not subscribed
    autoware_map::category_t required_category = autoware_map::Category::POINT |
                                                 autoware_map::Category::WAYPOINT;
    if(!awmap.hasSubscribed(required_category) )
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & awmap.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "
                        << missing_category << std::endl
                        << "skipping conversion for nodes");
        return;
    }

    for ( auto awmap_wp : awmap.findByFilter([&](autoware_map_msgs::Waypoint){return true; } ))
    {
        vector_map_msgs::Node vmap_node;
        vmap_node.nid = awmap_wp.waypoint_id;
        vmap_node.pid = awmap_wp.waypoint_id;

        vmap_nodes.push_back(vmap_node);
    }
}

std::vector<int> findBranchingIdx(const std::vector<autoware_map_msgs::WaypointRelation> relation, int root_index)
{
    std::vector<int> branching_indices;
    for(auto itr = relation.begin(); itr != relation.end(); itr++)
    {
        if(itr->waypoint_id == root_index)
        {
            branching_indices.push_back(std::distance(relation.begin(), itr));
        }
    }
    return branching_indices;
}
std::vector<int> findMergingIdx(const std::vector<autoware_map_msgs::WaypointRelation> relation, int merged_index)
{
    std::vector<int> merging_indices;

    for(auto itr = relation.begin(); itr != relation.end(); itr++)
    {
        if(itr->next_waypoint_id == merged_index)
        {
            merging_indices.push_back(std::distance(relation.begin(), itr));
        }
    }
    return merging_indices;
}

int getJunctionType(const std::vector<autoware_map_msgs::WaypointRelation> awmap_waypoint_relations,
                    std::vector<int> branching_idx,
                    std::vector<int> merging_idx)
{

    // if(merging_idx.size() > 1) return vector_map_msgs::Lane::RIGHT_MERGING;
    // else         return vector_map_msgs::Lane::NORMAL;

    if(branching_idx.size() <= 1 && merging_idx.size() <= 1 )
    {
        return vector_map_msgs::Lane::NORMAL;
    }
    int left_branching_cnt = 0;
    int right_branching_cnt = 0;
    int straight_branching_cnt = 0;
    int left_merging_cnt = 0;
    int right_merging_cnt = 0;
    int straight_merging_cnt = 0;

    for(auto idx : branching_idx)
    {
        if( awmap_waypoint_relations.at(idx).blinker == 1 )
        {
            left_branching_cnt++;
        }
        if( awmap_waypoint_relations.at(idx).blinker == 2 )
        {
            right_branching_cnt++;
        }else{
            straight_branching_cnt++;
        }
    }
    for(auto idx : merging_idx)
    {
        if( awmap_waypoint_relations.at(idx).blinker == 1 )
        {
            left_merging_cnt++;
        }
        if( awmap_waypoint_relations.at(idx).blinker == 2 )
        {
            right_merging_cnt++;
        }else{
            straight_merging_cnt++;
        }
    }

    if(branching_idx.size() >= 2 && merging_idx.size() >= 2 )
    {
        return vector_map_msgs::Lane::COMPOSITION;
    }
    if ( right_branching_cnt >= 1 )
    {
        return vector_map_msgs::Lane::RIGHT_BRANCHING;
    }
    if ( left_branching_cnt >= 1 )
    {
        return vector_map_msgs::Lane::LEFT_BRANCHING;
    }
    if( straight_branching_cnt >= 2) {
        return vector_map_msgs::Lane::LEFT_BRANCHING;
    }
    if ( right_merging_cnt >= 1 )
    {
        return vector_map_msgs::Lane::RIGHT_MERGING;
    }
    if ( left_merging_cnt >= 1 )
    {
        return vector_map_msgs::Lane::LEFT_MERGING;
    }
    if( straight_merging_cnt >= 2) {
        return vector_map_msgs::Lane::LEFT_MERGING;
    }

    ROS_ERROR_STREAM("could not find appropriate junction type!!!!!!!");

    return vector_map_msgs::Lane::NORMAL;
}

void createDTLanes(const AutowareMap &awmap,
                   std::vector<vector_map_msgs::DTLane> &vmap_dtlanes,
                   std::vector<vector_map_msgs::Lane> &vmap_lanes)
{
    //return if enough information is not subscribed
    autoware_map::category_t required_category = autoware_map::Category::POINT |
                                                 autoware_map::Category::WAYPOINT_RELATION;
    if(!awmap.hasSubscribed(required_category) )
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & awmap.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "
                        << missing_category << std::endl
                        << "skipping conversion for dtlanes");
        return;
    }

    unsigned int id = 1;
    std::vector<autoware_map_msgs::WaypointRelation> awmap_waypoint_relations = awmap.findByFilter( [&](autoware_map_msgs::WaypointRelation wr){return true; });
    for ( auto awmap_waypoint_relation : awmap_waypoint_relations )
    {
        vector_map_msgs::DTLane vmap_dtlane;
        vmap_dtlane.did = id;
        vmap_dtlane.dist = awmap_waypoint_relation.distance;
        vmap_dtlane.pid = awmap_waypoint_relation.waypoint_id;

        autoware_map_msgs::Waypoint awmap_waypoint = awmap.findById<autoware_map_msgs::Waypoint>(awmap_waypoint_relation.waypoint_id);
        autoware_map_msgs::Waypoint awmap_next_waypoint = awmap.findById<autoware_map_msgs::Waypoint>(awmap_waypoint_relation.next_waypoint_id);

        vmap_dtlane.dir = convertDecimalToDDMMSS(awmap_waypoint_relation.yaw);
        vmap_dtlane.apara = 0;
        vmap_dtlane.r = 90000000000;

        autoware_map_msgs::Point pt1, pt2;
        pt1 = awmap.findById<autoware_map_msgs::Point>(awmap_waypoint.point_id);
        pt2 = awmap.findById<autoware_map_msgs::Point>(awmap_next_waypoint.point_id);
        double horizontal_dist = hypot(pt2.x - pt1.x, pt2.y - pt1.y);
        double vertical_dist = pt2.z - pt1.z;

        vmap_dtlane.slope = vertical_dist / horizontal_dist * 100; //decimal to percentage value
        vmap_dtlane.cant = 0;
        vmap_dtlane.lw = awmap_waypoint.left_width;
        vmap_dtlane.rw = awmap_waypoint.right_width;

        vmap_dtlanes.push_back(vmap_dtlane);

        // std::vector<autoware_map_msgs::WaypointRelation> related_waypoint_relations= awmap.findByFilter([&awmap_waypoint_relation](const autoware_map_msgs::WaypointRelation& wpr){return wpr.waypoint_id == awmap_waypoint_relation.waypoint_id;};

        vector_map_msgs::Lane vmap_lane;
        vmap_lane.lnid = id;
        vmap_lane.did = id;

        std::vector<int> merging_idx = findMergingIdx(awmap_waypoint_relations, awmap_waypoint_relation.waypoint_id);

        std::vector<int> branching_idx = findBranchingIdx(awmap_waypoint_relations, awmap_waypoint_relation.next_waypoint_id);

        //change order of branch/merge lanes according to blinkers. (staright < left turn < right turn)
        sort(merging_idx.begin(), merging_idx.end(), [&](const int x, const int y){ return awmap_waypoint_relations.at(x).blinker < awmap_waypoint_relations.at(y).blinker; });
        sort(branching_idx.begin(), branching_idx.end(), [&](const int x, const int y){ return awmap_waypoint_relations.at(x).blinker < awmap_waypoint_relations.at(y).blinker; });

        vmap_lane.jct = getJunctionType(awmap_waypoint_relations, branching_idx, merging_idx);
        vmap_lane.blid = 0;
        vmap_lane.flid = 0;
        vmap_lane.blid2 = 0;
        vmap_lane.blid3 = 0;
        vmap_lane.blid4 = 0;
        vmap_lane.flid2 = 0;
        vmap_lane.flid3 = 0;
        vmap_lane.flid4 = 0;

        if(merging_idx.size() >= 1)
            vmap_lane.blid = merging_idx.at(0) + 1;
        if(merging_idx.size() >= 2)
            vmap_lane.blid2 = merging_idx.at(1) + 1;
        if(merging_idx.size() >= 3)
            vmap_lane.blid3 = merging_idx.at(2) + 1;
        if(merging_idx.size() >= 4)
            vmap_lane.blid4 = merging_idx.at(3) + 1;
        if(branching_idx.size() >= 1)
            vmap_lane.flid = branching_idx.at(0) + 1;
        if(branching_idx.size() >= 2)
            vmap_lane.flid2 = branching_idx.at(1) + 1;
        if(branching_idx.size() >= 3)
            vmap_lane.flid3 = branching_idx.at(2) + 1;
        if(branching_idx.size() >= 4)
            vmap_lane.flid4 = branching_idx.at(3) + 1;

        vmap_lane.bnid = awmap_waypoint.point_id;
        vmap_lane.fnid = awmap_next_waypoint.point_id;
        vmap_lane.span =  awmap_waypoint_relation.distance;

        int awmap_lane_id = 0;
        auto waypoint_lane_relation = awmap.findByFilter([&](autoware_map_msgs::WaypointLaneRelation wlr){return wlr.waypoint_id == awmap_waypoint.waypoint_id; });
        auto awmap_lane = awmap.findById<autoware_map_msgs::Lane>(waypoint_lane_relation.front().lane_id );
        vmap_lane.lcnt = awmap_lane.num_of_lanes;
        vmap_lane.lanetype = awmap_waypoint_relation.blinker;
        vmap_lane.limitvel = awmap.findById<autoware_map_msgs::Lane>(awmap_lane_id).speed_limit;
        vmap_lane.refvel = awmap_waypoint.velocity;
        vmap_lane.lanecfgfg = (vmap_lane.lcnt > 1) ? 1 : 0;
        vmap_lane.lno = awmap_lane.lane_number;
        vmap_lane.roadsecid = 0;
        vmap_lane.linkwaid = 0;
        vmap_lanes.push_back(vmap_lane);
        id++;
    }
}

void createWayAreas(const AutowareMap &awmap, std::vector<vector_map_msgs::WayArea> &vmap_way_areas)
{
    //return if enough information is not subscribed
    autoware_map::category_t required_category = autoware_map::Category::POINT |
                                                 autoware_map::Category::AREA |
                                                 autoware_map::Category::WAYAREA;
    if(!awmap.hasSubscribed(required_category) )
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & awmap.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "
                        << missing_category << std::endl
                        << "skipping conversion for wayareas");
        return;
    }

    for ( auto awmap_area : awmap.findByFilter( [&](autoware_map_msgs::Wayarea wa){return true; }))
    {
        vector_map_msgs::WayArea way_area;
        way_area.waid = awmap_area.wayarea_id;
        way_area.aid = awmap_area.area_id;
        vmap_way_areas.push_back(way_area);
    }
}

void createWayAreasFromLanes(const AutowareMap &awmap, std::vector<vector_map_msgs::WayArea> &vmap_way_areas,
                             std::vector<vector_map_msgs::Area> &vmap_areas,
                             std::vector<vector_map_msgs::Line> &vmap_lines,
                             std::vector<vector_map_msgs::Point> &vmap_points )
{
    //return if enough information is not subscribed
    autoware_map::category_t required_category = autoware_map::Category::POINT |
                                                 autoware_map::Category::WAYPOINT |
                                                 autoware_map::Category::WAYPOINT_RELATION;
    if(!awmap.hasSubscribed(required_category) )
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & awmap.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "
                        << missing_category << std::endl
                        << "skipping conversion for wayareas");
        return;
    }

    const std::vector<autoware_map_msgs::WaypointRelation> awmap_waypoint_relations = awmap.findByFilter([&](autoware_map_msgs::WaypointRelation ){return true; });
    int line_id = getMaxId(vmap_lines) + 1;
    int point_id = getMaxId(vmap_points) + 1;
    int area_id = getMaxId(vmap_areas) + 1;
    int wayarea_id = getMaxId(vmap_way_areas) + 1;

    std::unordered_map<int, WaypointWithYaw> wp_yaw_map;
    for(auto relation : awmap_waypoint_relations)
    {
        autoware_map_msgs::Waypoint awmap_wp = awmap.findById<autoware_map_msgs::Waypoint>(relation.waypoint_id);
        autoware_map_msgs::Waypoint awmap_next_wp = awmap.findById<autoware_map_msgs::Waypoint>(relation.next_waypoint_id);
        autoware_map_msgs::Point awmap_pt = awmap.findById<autoware_map_msgs::Point>(awmap_wp.point_id);
        autoware_map_msgs::Point awmap_next_pt = awmap.findById<autoware_map_msgs::Point>(awmap_next_wp.point_id);

        double yaw = atan2(awmap_next_pt.y - awmap_pt.y, awmap_next_pt.x - awmap_pt.x);

        WaypointWithYaw wp_yaw;
        if(wp_yaw_map.find(relation.waypoint_id) != wp_yaw_map.end()) {
            wp_yaw = wp_yaw_map.at(relation.waypoint_id);
        }
        wp_yaw.waypoint = awmap_wp;
        wp_yaw.point = awmap_pt;
        wp_yaw.yaws.push_back(yaw);
        wp_yaw_map[relation.waypoint_id] = wp_yaw;

        WaypointWithYaw next_wp_yaw;
        if(wp_yaw_map.find(relation.next_waypoint_id) != wp_yaw_map.end()) {
            next_wp_yaw = wp_yaw_map.at(relation.next_waypoint_id);
        }
        next_wp_yaw.waypoint = awmap_next_wp;
        next_wp_yaw.point = awmap_next_pt;
        next_wp_yaw.yaws.push_back(yaw);
        wp_yaw_map[relation.next_waypoint_id] = next_wp_yaw;
    }

    for( auto &item : wp_yaw_map)
    {
        WaypointWithYaw wp_yaw = item.second;
        wp_yaw.yaw_avg = getAngleAverage(wp_yaw.yaws);
        double yaw = wp_yaw.yaw_avg;
        double left_width = wp_yaw.waypoint.left_width;
        double right_width = wp_yaw.waypoint.right_width;

        double angle_left, angle_right;
        if(isJapaneseCoordinate(wp_yaw.point.epsg)) {
            angle_left = addAngles(yaw, -M_PI / 2);
            angle_right = addAngles(yaw, M_PI / 2);
        }else
        {
            angle_left = addAngles(yaw, M_PI / 2);
            angle_right = addAngles(yaw, -M_PI / 2);
        }

        autoware_map_msgs::Point left_point,right_point;
        left_point.x = wp_yaw.point.x + left_width * cos(angle_left);
        left_point.y = wp_yaw.point.y + left_width * sin(angle_left);
        left_point.z = wp_yaw.point.z;
        left_point.point_id = point_id++;
        wp_yaw.left_point = left_point;

        right_point.x = wp_yaw.point.x + right_width * cos(angle_right);
        right_point.y = wp_yaw.point.y + right_width * sin(angle_right);
        right_point.z = wp_yaw.point.z;
        right_point.point_id = point_id++;
        wp_yaw.right_point = right_point;
        item.second = wp_yaw;
    }

    for(auto relation : awmap_waypoint_relations)
    {
        WaypointWithYaw wp_yaw = wp_yaw_map.at(relation.waypoint_id);
        WaypointWithYaw next_wp_yaw = wp_yaw_map.at(relation.next_waypoint_id);

        vector_map_msgs::Point pt1, pt2, pt3, pt4;

        convertPoint(pt1, wp_yaw.left_point);
        pt1.pid = wp_yaw.left_point.point_id;
        convertPoint(pt2, next_wp_yaw.left_point);
        pt2.pid = next_wp_yaw.left_point.point_id;
        if(pt2.pid == 0){
            std::cout << next_wp_yaw.waypoint.waypoint_id << std::endl;
        }
        convertPoint(pt3, next_wp_yaw.right_point);
        pt3.pid = next_wp_yaw.right_point.point_id;
        convertPoint(pt4, wp_yaw.right_point);
        pt4.pid = wp_yaw.right_point.point_id;

        vmap_points.push_back(pt1);
        vmap_points.push_back(pt2);
        vmap_points.push_back(pt3);
        vmap_points.push_back(pt4);

        vector_map_msgs::Line vmap_line;
        vmap_line.lid = line_id;
        vmap_line.bpid = pt1.pid;
        vmap_line.fpid = pt2.pid;
        vmap_line.blid = 0;
        vmap_line.flid = line_id + 1;
        vmap_lines.push_back(vmap_line);
        line_id++;

        vmap_line.lid = line_id;
        vmap_line.bpid = pt2.pid;
        vmap_line.fpid = pt3.pid;
        vmap_line.blid = line_id - 1;
        vmap_line.flid = line_id + 1;
        vmap_lines.push_back(vmap_line);
        line_id++;

        vmap_line.lid = line_id;
        vmap_line.bpid = pt3.pid;
        vmap_line.fpid = pt4.pid;
        vmap_line.blid = line_id - 1;
        vmap_line.flid = line_id + 1;
        vmap_lines.push_back(vmap_line);
        line_id++;

        vmap_line.lid = line_id;
        vmap_line.bpid = pt4.pid;
        vmap_line.fpid = pt1.pid;
        vmap_line.blid = line_id - 1;
        vmap_line.flid = 0;
        vmap_lines.push_back(vmap_line);
        line_id++;

        vector_map_msgs::Area vmap_area;
        vmap_area.aid = area_id++;
        vmap_area.slid = line_id - 4;
        vmap_area.elid = line_id - 1;
        vmap_areas.push_back(vmap_area);

        vector_map_msgs::WayArea vmap_way_area;
        vmap_way_area.aid = vmap_area.aid;
        vmap_way_area.waid = wayarea_id++;
        vmap_way_areas.push_back(vmap_way_area);
    }
}

void createSignals( const AutowareMap &awmap,
                    std::vector<vector_map_msgs::Signal> &vmap_signals,
                    std::vector<vector_map_msgs::Vector> &vmap_vectors,
                    std::vector<vector_map_msgs::Pole> &vmap_dummy_poles)
{
    //return if enough information is not subscribed
    autoware_map::category_t required_category = autoware_map::Category::POINT |
                                                 autoware_map::Category::SIGNAL_LIGHT |
                                                 autoware_map::Category::SIGNAL |
                                                 autoware_map::Category::WAYPOINT_SIGNAL_RELATION |
                                                 autoware_map::Category::LANE_SIGNAL_LIGHT_RELATION;
    if(!awmap.hasSubscribed(required_category) )
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & awmap.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "
                        << missing_category << std::endl
                        << "skipping conversion for signals");
        return;
    }

    std::vector<autoware_map_msgs::SignalLight> awmap_signal_lights = awmap.findByFilter( [&](autoware_map_msgs::SignalLight sl){return true; });
    std::vector<autoware_map_msgs::WaypointRelation> awmap_waypoint_relations = awmap.findByFilter( [&](autoware_map_msgs::WaypointRelation wr){return true; });
    std::vector<autoware_map_msgs::LaneSignalLightRelation> awmap_lane_signal_relations = awmap.findByFilter( [&](autoware_map_msgs::LaneSignalLightRelation lslr){return true; });

    unsigned int vector_id = 1;
    for ( auto awmap_signal_light : awmap_signal_lights)
    {
        vector_map_msgs::Signal vmap_signal;
        vmap_signal.id = awmap_signal_light.signal_light_id;
        vmap_signal.plid = awmap_signal_light.signal_id;

        //create dummy poles
        vector_map_msgs::Pole vmap_pole;
        vmap_pole.plid = vmap_signal.plid;
        vmap_dummy_poles.push_back(vmap_pole);

        //color:{1 = red, 2=green, 3=yellow}
        if(awmap_signal_light.color_type <=3 && awmap_signal_light.arrow_type == 0)
        {
            vmap_signal.type = awmap_signal_light.color_type;
        }
        else{
            vmap_signal.type = 9; //other
        }

        //create Vector to describe signal direction
        vector_map_msgs::Vector vmap_vector;
        vmap_vector.vid = vector_id;
        vmap_vector.pid = awmap_signal_light.point_id;
        vmap_vector.hang =  convertDecimalToDDMMSS( awmap_signal_light.horizontal_angle );
        vmap_vector.vang =  convertDecimalToDDMMSS( awmap_signal_light.vertical_angle );
        vmap_vectors.push_back(vmap_vector);
        vmap_signal.vid = vector_id;
        vector_id += 1;

        auto lane_signal_itr = std::find_if(   awmap_lane_signal_relations.begin(),
                                               awmap_lane_signal_relations.end(),
                                               [awmap_signal_light](autoware_map_msgs::LaneSignalLightRelation lslr){ return lslr.signal_light_id == awmap_signal_light.signal_light_id; });

        autoware_map_msgs::Lane awmap_lane;
        if (lane_signal_itr != awmap_lane_signal_relations.end())
        {
            awmap_lane = awmap.findById<autoware_map_msgs::Lane>(lane_signal_itr->lane_id);
        }

        int linkid = 0;
        for(auto itr = awmap_waypoint_relations.begin(); itr != awmap_waypoint_relations.end(); itr++)
        {
            if(itr->next_waypoint_id == awmap_lane.end_waypoint_id)
            {
                auto awmap_waypoint_lane_relations = awmap.findByFilter([&](autoware_map_msgs::WaypointLaneRelation wlr){return wlr.waypoint_id == itr->waypoint_id; });
                for(auto awmap_waypoint_lane_relation : awmap_waypoint_lane_relations)
                {
                    if( awmap_waypoint_lane_relation.lane_id == awmap_lane.lane_id)
                    {
                        linkid = std::distance(awmap_waypoint_relations.begin(), itr) + 1;
                    }
                }
            }
        }
        if(linkid == 0)
        {
            ROS_ERROR_STREAM("failed to find valid linkid for signal");
        }

        vmap_signal.linkid = linkid;
        vmap_signals.push_back(vmap_signal);
    }
}

vector_map_msgs::RoadSign createDummyRoadSign(int id)
{
    vector_map_msgs::RoadSign road_sign;
    road_sign.id = id;
    road_sign.vid = 0;
    road_sign.plid = 0;
    road_sign.type = 1;
    road_sign.linkid = 0;
    return road_sign;
}


void createStopLines( const AutowareMap &awmap,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                      std::vector<vector_map_msgs::RoadSign> &vmap_road_signs )
{
    autoware_map::category_t required_category = autoware_map::Category::POINT |
                                                 autoware_map::Category::WAYPOINT |
                                                 autoware_map::Category::WAYPOINT_RELATION;
    if(!awmap.hasSubscribed(required_category) )
    {
        autoware_map::Category missing_category = static_cast<autoware_map::Category>(required_category - (required_category & awmap.hasSubscribed()));
        ROS_WARN_STREAM("missing category : "
                        << missing_category << std::endl
                        << "skipping conversion for stoplines");
        return;
    }

    const std::vector<autoware_map_msgs::WaypointRelation> awmap_waypoint_relations = awmap.findByFilter([&](autoware_map_msgs::WaypointRelation ){return true; });

    int line_id = getMaxId(vmap_lines) + 1;
    int point_id = getMaxId(vmap_points) + 1;
    int stop_line_id = getMaxId(vmap_stop_lines) + 1;

    for(auto wp : awmap.findByFilter([] (const autoware_map_msgs::Waypoint wp){return wp.stop_line == 1; }))
    {
        autoware_map_msgs::Point awmap_pt = awmap.findById<autoware_map_msgs::Point>(wp.point_id);

        auto next_waypoint_relation = std::find_if(awmap_waypoint_relations.begin(),
                                                   awmap_waypoint_relations.end(),
                                                   [&](autoware_map_msgs::WaypointRelation wr){return wr.waypoint_id == wp.waypoint_id; });

        autoware_map_msgs::Waypoint next_wp = awmap.findById<autoware_map_msgs::Waypoint>(next_waypoint_relation->next_waypoint_id);
        autoware_map_msgs::Point awmap_next_pt = awmap.findById<autoware_map_msgs::Point>(next_wp.point_id);

        double yaw = atan2(awmap_next_pt.y - awmap_pt.y, awmap_next_pt.x - awmap_pt.x);
        double angle_left, angle_right;
        if(isJapaneseCoordinate(awmap_pt.epsg)) {
            angle_left = addAngles(yaw, -M_PI / 2);
            angle_right = addAngles(yaw, M_PI / 2);
        }else
        {
            angle_left = addAngles(yaw, M_PI / 2);
            angle_right = addAngles(yaw, -M_PI / 2);
        }

        vector_map_msgs::Point start_point, end_point;
        start_point.pid = point_id++;

        double right_width = wp.right_width;
        double left_width = wp.left_width;
        //stop line must intersect with waypoints left side of the line, shorten left side
        if(left_width < right_width * 0.9){
            left_width = right_width * 0.9;
        }
        //stop line cannot be right on waypoint with current rebuild_decision_maker
        double epsilon_x = cos(yaw) * 0.001;
        double epsilon_y = sin(yaw) * 0.001;

        start_point.bx = awmap_pt.x + ( left_width * 0.9 ) * cos(angle_left) + epsilon_x;
        start_point.ly = awmap_pt.y + ( left_width * 0.9) * sin(angle_left) + epsilon_y;
        start_point.h = awmap_pt.z;

        end_point.pid = point_id++;
        end_point.bx = awmap_pt.x + right_width * cos(angle_right) + epsilon_x;
        end_point.ly = awmap_pt.y + right_width * sin(angle_right) + epsilon_y;
        end_point.h = awmap_pt.z;

        // make sure that stop line does not intersect with other lanes.
        for(auto awmap_wp_relation : awmap_waypoint_relations )
        {
            if(awmap_wp_relation.waypoint_id == wp.waypoint_id || awmap_wp_relation.next_waypoint_id == wp.waypoint_id) {
                continue;
            }
            autoware_map_msgs::Waypoint wp1 = awmap.findById<autoware_map_msgs::Waypoint>(awmap_wp_relation.waypoint_id);
            autoware_map_msgs::Waypoint wp2 = awmap.findById<autoware_map_msgs::Waypoint>(awmap_wp_relation.next_waypoint_id);
            autoware_map_msgs::Point p1 = awmap.findById<autoware_map_msgs::Point>(wp1.point_id);
            autoware_map_msgs::Point p2 = awmap.findById<autoware_map_msgs::Point>(wp2.point_id);

            double intersect_x, intersect_y;
            if ( getIntersect(p1.x, p1.y, p2.x, p2.y,
                              start_point.bx, start_point.ly, end_point.bx, end_point.ly,
                              intersect_x, intersect_y))
            {
                double distance = std::hypot( awmap_pt.x - intersect_x, awmap_pt.y - intersect_y );
                //shorten length of stop line so that it does not cross any other lanes
                left_width = distance * 0.9;
                right_width = distance;
                start_point.bx = awmap_pt.x + left_width * cos(angle_left) + epsilon_x;
                start_point.ly = awmap_pt.y + left_width * sin(angle_left) + epsilon_y;
                end_point.bx = awmap_pt.x + right_width * cos(angle_right) + epsilon_x;
                end_point.ly = awmap_pt.y + right_width * sin(angle_right) + epsilon_y;
            }
        }

        //swap x and y if the coordinate is not in japanese rectangular coordinate system
        if(!isJapaneseCoordinate(awmap_pt.epsg))
        {
            double tmp = start_point.bx;
            start_point.bx = start_point.ly;
            start_point.ly = tmp;
            tmp = end_point.bx;
            end_point.bx = end_point.ly;
            end_point.ly = tmp;
        }

        vmap_points.push_back(start_point);
        vmap_points.push_back(end_point);

        vector_map_msgs::Line vmap_line;
        vmap_line.lid = line_id++;
        vmap_line.bpid = start_point.pid;
        vmap_line.fpid = end_point.pid;
        vmap_line.blid = vmap_line.flid = 0;
        vmap_lines.push_back(vmap_line);

        // only create road sign if stopline is not related to signal
        std::vector<autoware_map_msgs::WaypointSignalRelation> wsr_vector = awmap.findByFilter([&](const autoware_map_msgs::WaypointSignalRelation wsr){ return wsr.waypoint_id == wp.waypoint_id; });
        vector_map_msgs::StopLine vmap_stop_line;
        vmap_stop_line.id = stop_line_id++;
        vmap_stop_line.lid = vmap_line.lid;
        vmap_stop_line.tlid = 0;
        if( wsr_vector.empty()) {
            int road_sign_id = getMaxId(vmap_road_signs) + 1;
            vmap_road_signs.push_back(createDummyRoadSign(road_sign_id));
            vmap_stop_line.signid = road_sign_id;
        }

        vmap_stop_line.linkid = std::distance(awmap_waypoint_relations.begin(), next_waypoint_relation);
        vmap_stop_lines.push_back(vmap_stop_line);
    }
}
