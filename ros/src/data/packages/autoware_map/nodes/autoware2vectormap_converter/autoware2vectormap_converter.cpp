#include <autoware2vectormap_converter/autoware2vectormap_converter.h>

using vector_map::VectorMap;
using autoware_map::AutowareMap;

void convertPoint(vector_map_msgs::Point &vmap_point, autoware_map_msgs::Point awm_point)
{
    if(isJapaneseCoordinate(awm_point.epsg))
    {
        vmap_point.bx = awm_point.x;
        vmap_point.ly = awm_point.y;
    }
    else{
        vmap_point.bx = awm_point.y;
        vmap_point.ly = awm_point.x;
    }
    vmap_point.ref = convertESPGToRef(awm_point.epsg);
    vmap_point.pid = awm_point.point_id;
    vmap_point.b = convertDecimalToDDMMSS( awm_point.lat );
    vmap_point.l = convertDecimalToDDMMSS( awm_point.lng );
    vmap_point.h = awm_point.z;

    //cannot convert mcodes from autoware_map_format
    vmap_point.mcode1 = 0;
    vmap_point.mcode2 = 0;
    vmap_point.mcode3 = 0;
}

void createAreas(AutowareMap awm, std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines)
{
    int line_id = vmap_lines.size() + 1;
    for ( auto awm_area : awm.findByFilter( [&](autoware_map_msgs::Area a){return true; }))
    {
        vector_map_msgs::Area vmap_area;
        vmap_area.aid = awm_area.area_id;
        vmap_area.slid = line_id;

        //create lines that represent area
        auto end_itr = awm_area.point_ids.end();
        auto begin_itr = awm_area.point_ids.begin();
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

void createCrossRoads(AutowareMap awm, std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads)
{
    unsigned int id = 1;
    for ( auto awm_relation : awm.findByFilter( [&](autoware_map_msgs::LaneAttrRelation lar){return lar.attribute_type == autoware_map::LaneAttrRelation::INTERSECTION; }) )
    {
        //check whether same cross_road is already created
        if(std::find_if(vmap_cross_roads.begin(), vmap_cross_roads.end(), [&](vector_map_msgs::CrossRoad cr){return cr.aid == awm_relation.area_id; }) != vmap_cross_roads.end())
        {
            continue;
        }
        vector_map_msgs::CrossRoad cross_road;
        cross_road.id = id++;
        cross_road.aid = awm_relation.area_id;
        cross_road.linkid = 0;
        vmap_cross_roads.push_back(cross_road);
    }
}

int createSquareArea(double x, double y, double z, double length,
                     std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines,
                     std::vector<vector_map_msgs::Point> &vmap_points)
{
    vector_map_msgs::Point v1, v2, v3, v4;
    int point_id = vmap_points.size() + 1;
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
    lid =start_lid= vmap_lines.size() + 1;
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
    area.aid = vmap_areas.size() + 1;
    area.slid = start_lid;
    area.elid = lid;
    vmap_areas.push_back(area);

    return area.aid;
}

void createCrossWalks(AutowareMap awm, std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks,
                      std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points)
{
    unsigned int id = 1;
    for( auto awm_relation : awm.findByFilter([&](const autoware_map_msgs::LaneAttrRelation lar){ return lar.attribute_type == autoware_map::LaneAttrRelation::CROSS_WALK; }) )
    {
        //skip if area is already added
        if(std::find_if(vmap_cross_walks.begin(), vmap_cross_walks.end(), [&](vector_map_msgs::CrossWalk cw){return cw.aid == awm_relation.area_id; }) != vmap_cross_walks.end())
        {
            continue;
        }

        int parent_id = id++;
        vector_map_msgs::CrossWalk cross_walk;
        cross_walk.id = parent_id;
        cross_walk.aid = awm_relation.area_id;
        cross_walk.type = 0;
        cross_walk.linkid = 0;
        vmap_cross_walks.push_back(cross_walk);


        cross_walk.id = id++;
        cross_walk.aid = awm_relation.area_id;
        cross_walk.type = 1;
        cross_walk.bdid = parent_id;
        cross_walk.linkid = 0;
        vmap_cross_walks.push_back(cross_walk);

        //create regions for detection area
        autoware_map_msgs::Area awm_area = awm.findById<autoware_map_msgs::Area>(awm_relation.area_id);
        std::vector<autoware_map_msgs::Point> vertices;
        for ( int vertex : awm_area.point_ids)
        {
            vertices.push_back(awm.findById<autoware_map_msgs::Point>(vertex));
        }

        autoware_map_msgs::Point min,max;
        getMinMax(min,max,vertices);

        double resolution = 1.0;
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

void createPoints(AutowareMap awm, std::vector<vector_map_msgs::Point> &vmap_points)
{
    bool epsg_fail_flag = false;
    for ( auto awm_pt : awm.findByFilter( [&](autoware_map_msgs::Point pt){return true; }) )
    {
        if(isJapaneseCoordinate(awm_pt.epsg)) {
            epsg_fail_flag =true;
        }
        vector_map_msgs::Point vmap_point;
        convertPoint(vmap_point, awm_pt);
        vmap_points.push_back(vmap_point);
    }
    if(epsg_fail_flag)
    {
        ROS_WARN_STREAM("no corresponding Japanese Plane Rectangular CS Number for specified epsg value");
    }
}

void createNodes(AutowareMap awm, std::vector<vector_map_msgs::Node> &vmap_nodes)
{
    for ( auto awm_wp : awm.findByFilter([&](autoware_map_msgs::Waypoint){return true; } ))
    {
        vector_map_msgs::Node vmap_node;
        vmap_node.nid = awm_wp.waypoint_id;
        vmap_node.pid = awm_wp.waypoint_id;

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

int getJunctionType(const std::vector<autoware_map_msgs::WaypointRelation> awm_waypoint_relations,
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
        if( awm_waypoint_relations.at(idx).blinker == 1 )
        {
            left_branching_cnt++;
        }
        if( awm_waypoint_relations.at(idx).blinker == 2 )
        {
            right_branching_cnt++;
        }else{
            straight_branching_cnt++;
        }
    }
    for(auto idx : merging_idx)
    {
        if( awm_waypoint_relations.at(idx).blinker == 1 )
        {
            left_merging_cnt++;
        }
        if( awm_waypoint_relations.at(idx).blinker == 2 )
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

void createDTLanes(const AutowareMap awm,
                   std::vector<vector_map_msgs::DTLane> &vmap_dtlanes,
                   std::vector<vector_map_msgs::Lane> &vmap_lanes)
{
    unsigned int id = 1;
    std::vector<autoware_map_msgs::WaypointRelation> awm_waypoint_relations = awm.findByFilter( [&](autoware_map_msgs::WaypointRelation wr){return true; });
    for ( auto awm_waypoint_relation : awm_waypoint_relations )
    {
        vector_map_msgs::DTLane vmap_dtlane;
        vmap_dtlane.did = id;
        vmap_dtlane.dist = awm_waypoint_relation.distance;
        vmap_dtlane.pid = awm_waypoint_relation.waypoint_id;

        autoware_map_msgs::Waypoint awm_waypoint = awm.findById<autoware_map_msgs::Waypoint>(awm_waypoint_relation.waypoint_id);
        autoware_map_msgs::Waypoint awm_next_waypoint = awm.findById<autoware_map_msgs::Waypoint>(awm_waypoint_relation.next_waypoint_id);

        vmap_dtlane.dir = convertDecimalToDDMMSS(awm_waypoint_relation.yaw);
        vmap_dtlane.apara = 0;
        vmap_dtlane.r = 90000000000;

        autoware_map_msgs::Point pt1, pt2;
        pt1 = awm.findById<autoware_map_msgs::Point>(awm_waypoint.point_id);
        pt2 = awm.findById<autoware_map_msgs::Point>(awm_next_waypoint.point_id);
        double horizontal_dist = hypot(pt2.x - pt1.x, pt2.y - pt1.y);
        double vertical_dist = pt2.z - pt1.z;

        vmap_dtlane.slope = vertical_dist / horizontal_dist * 100; //decimal to percentage value
        vmap_dtlane.cant = 0;
        vmap_dtlane.lw = awm_waypoint.width / 2;
        vmap_dtlane.rw = awm_waypoint.width / 2;

        vmap_dtlanes.push_back(vmap_dtlane);

        // std::vector<autoware_map_msgs::WaypointRelation> related_waypoint_relations= awm.findByFilter([&awm_waypoint_relation](const autoware_map_msgs::WaypointRelation& wpr){return wpr.waypoint_id == awm_waypoint_relation.waypoint_id;};

        vector_map_msgs::Lane vmap_lane;
        vmap_lane.lnid = id;
        vmap_lane.did = id;

        std::vector<int> merging_idx = findMergingIdx(awm_waypoint_relations, awm_waypoint_relation.waypoint_id);

        std::vector<int> branching_idx = findBranchingIdx(awm_waypoint_relations, awm_waypoint_relation.next_waypoint_id);

        //change order of branch/merge lanes according to blinkers. (staright < left turn < right turn)
        sort(merging_idx.begin(), merging_idx.end(), [&](const int x, const int y){ return awm_waypoint_relations.at(x).blinker < awm_waypoint_relations.at(y).blinker; });
        sort(branching_idx.begin(), branching_idx.end(), [&](const int x, const int y){ return awm_waypoint_relations.at(x).blinker < awm_waypoint_relations.at(y).blinker; });

        vmap_lane.jct = getJunctionType(awm_waypoint_relations, branching_idx, merging_idx);
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

        vmap_lane.bnid = awm_waypoint.point_id;
        vmap_lane.fnid = awm_next_waypoint.point_id;
        vmap_lane.span =  awm_waypoint_relation.distance;

        int awm_lane_id = 0;
        auto waypoint_lane_relation = awm.findByFilter([&](autoware_map_msgs::WaypointLaneRelation wlr){return wlr.waypoint_id == awm_waypoint.waypoint_id; });
        auto awm_lane = awm.findById<autoware_map_msgs::Lane>(waypoint_lane_relation.front().lane_id );
        vmap_lane.lcnt = awm_lane.num_of_lanes;
        vmap_lane.lanetype = awm_waypoint_relation.blinker;
        vmap_lane.limitvel = awm.findById<autoware_map_msgs::Lane>(awm_lane_id).speed_limit;
        vmap_lane.refvel = awm_waypoint.velocity;
        vmap_lane.lanecfgfg = (vmap_lane.lcnt > 1) ? 1 : 0;
        vmap_lane.lno = awm_lane.lane_number;
        vmap_lane.roadsecid = 0;
        vmap_lane.linkwaid = 0;
        vmap_lanes.push_back(vmap_lane);
        id++;
    }
}

void createWayAreas(const AutowareMap awm, std::vector<vector_map_msgs::WayArea> &vmap_way_areas)
{
    for ( auto awm_area : awm.findByFilter( [&](autoware_map_msgs::Wayarea wa){return true; }))
    {
        vector_map_msgs::WayArea way_area;
        way_area.waid = awm_area.wayarea_id;
        way_area.aid = awm_area.area_id;
        vmap_way_areas.push_back(way_area);
    }
}

void createSignals( const AutowareMap awm,
                    std::vector<vector_map_msgs::Signal> &vmap_signals,
                    std::vector<vector_map_msgs::Vector> &vmap_vectors,
                    std::vector<vector_map_msgs::Pole> &vmap_dummy_poles
                    )
{

    std::vector<autoware_map_msgs::SignalLight> awm_signal_lights = awm.findByFilter( [&](autoware_map_msgs::SignalLight sl){return true; });
    std::vector<autoware_map_msgs::WaypointRelation> awm_waypoint_relations = awm.findByFilter( [&](autoware_map_msgs::WaypointRelation wr){return true; });
    std::vector<autoware_map_msgs::LaneSignalLightRelation> awm_lane_signal_relations = awm.findByFilter( [&](autoware_map_msgs::LaneSignalLightRelation lslr){return true; });

    unsigned int vector_id = 1;
    for ( auto awm_signal_light : awm_signal_lights)
    {
        vector_map_msgs::Signal vmap_signal;
        vmap_signal.id = awm_signal_light.signal_light_id;
        vmap_signal.plid = awm_signal_light.signal_id;

        //create dummy poles
        vector_map_msgs::Pole vmap_pole;
        vmap_pole.plid = vmap_signal.plid;
        vmap_dummy_poles.push_back(vmap_pole);

        //color:{1 = red, 2=green, 3=yellow}
        if(awm_signal_light.color_type <=3 && awm_signal_light.arrow_type == 0)
        {
            vmap_signal.type = awm_signal_light.color_type;
        }
        else{
            vmap_signal.type = 9; //other
        }

        //create Vector to describe signal direction
        vector_map_msgs::Vector vmap_vector;
        vmap_vector.vid = vector_id;
        vmap_vector.pid = awm_signal_light.point_id;
        vmap_vector.hang =  convertDecimalToDDMMSS( awm_signal_light.horizontal_angle );
        vmap_vector.vang =  convertDecimalToDDMMSS( awm_signal_light.vertical_angle );
        vmap_vectors.push_back(vmap_vector);
        vmap_signal.vid = vector_id;
        vector_id += 1;

        auto lane_signal_itr = std::find_if(   awm_lane_signal_relations.begin(),
                                               awm_lane_signal_relations.end(),
                                               [awm_signal_light](autoware_map_msgs::LaneSignalLightRelation lslr){ return lslr.signal_light_id == awm_signal_light.signal_light_id; });

        autoware_map_msgs::Lane awm_lane;
        if (lane_signal_itr != awm_lane_signal_relations.end())
        {
            awm_lane = awm.findById<autoware_map_msgs::Lane>(lane_signal_itr->lane_id);
        }

        int linkid = 0;
        for(auto itr = awm_waypoint_relations.begin(); itr != awm_waypoint_relations.end(); itr++)
        {
            if(itr->next_waypoint_id == awm_lane.end_waypoint_id)
            {
                auto awm_waypoint_lane_relations = awm.findByFilter([&](autoware_map_msgs::WaypointLaneRelation wlr){return wlr.waypoint_id == itr->waypoint_id; });
                for(auto awm_waypoint_lane_relation : awm_waypoint_lane_relations)
                {
                    if( awm_waypoint_lane_relation.lane_id == awm_lane.lane_id)
                    {
                        linkid = std::distance(awm_waypoint_relations.begin(), itr) + 1;
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


void createStopLines( const AutowareMap awm,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::StopLine> &vmap_stop_lines,
                      std::vector<vector_map_msgs::RoadSign> &vmap_road_signs
                      )
{
    const std::vector<autoware_map_msgs::WaypointRelation> awm_waypoint_relations = awm.findByFilter([&](autoware_map_msgs::WaypointRelation ){return true; });

    int line_id = vmap_lines.size() + 1;
    int point_id = vmap_points.size() + 1;
    int stop_line_id = vmap_stop_lines.size() + 1;

    for(auto wp : awm.findByFilter([] (const autoware_map_msgs::Waypoint wp){return wp.stop_line == 1; }))
    {
        autoware_map_msgs::Point awm_pt = awm.findById<autoware_map_msgs::Point>(wp.point_id);

        auto next_waypoint_relation = std::find_if(awm_waypoint_relations.begin(),
                                                   awm_waypoint_relations.end(),
                                                   [&](autoware_map_msgs::WaypointRelation wr){return wr.waypoint_id == wp.waypoint_id; });

        autoware_map_msgs::Waypoint next_wp = awm.findById<autoware_map_msgs::Waypoint>(next_waypoint_relation->next_waypoint_id);
        autoware_map_msgs::Point awm_next_pt = awm.findById<autoware_map_msgs::Point>(next_wp.point_id);

        double yaw = atan2(awm_next_pt.y - awm_pt.y, awm_next_pt.x - awm_pt.x);
        double angle_left, angle_right;
        if(isJapaneseCoordinate(awm_pt.epsg)) {
            angle_left = addAngles(yaw, -M_PI / 2);
            angle_right = addAngles(yaw, M_PI / 2);
        }else
        {
            angle_left = addAngles(yaw, M_PI / 2);
            angle_right = addAngles(yaw, -M_PI / 2);
        }

        vector_map_msgs::Point start_point, end_point;
        start_point.pid = point_id++;

        double r = wp.width / 2;
        //stop line cannot be right on waypoint with current rebuild_decision_maker
        double epsilon_x = cos(yaw) * 0.001;
        double epsilon_y = sin(yaw) * 0.001;

        //stop line must intersect with waypoints left side of the line, lengthen left side
        start_point.bx = awm_pt.x + ( r * 0.9 ) * cos(angle_left) + epsilon_x;
        start_point.ly = awm_pt.y + (r * 0.9) * sin(angle_left) + epsilon_y;
        start_point.h = awm_pt.z;

        end_point.pid = point_id++;
        end_point.bx = awm_pt.x + r * cos(angle_right) + epsilon_x;
        end_point.ly = awm_pt.y + r * sin(angle_right) + epsilon_y;
        end_point.h = awm_pt.z;

        // make sure that stop line does not intersect with other lanes.
        for(auto awm_wp_relation : awm_waypoint_relations )
        {
            if(awm_wp_relation.waypoint_id == wp.waypoint_id || awm_wp_relation.next_waypoint_id == wp.waypoint_id) {
                continue;
            }
            autoware_map_msgs::Waypoint wp1 = awm.findById<autoware_map_msgs::Waypoint>(awm_wp_relation.waypoint_id);
            autoware_map_msgs::Waypoint wp2 = awm.findById<autoware_map_msgs::Waypoint>(awm_wp_relation.next_waypoint_id);
            autoware_map_msgs::Point p1 = awm.findById<autoware_map_msgs::Point>(wp1.point_id);
            autoware_map_msgs::Point p2 = awm.findById<autoware_map_msgs::Point>(wp2.point_id);

            double intersect_x, intersect_y;
            if ( getIntersect(p1.x, p1.y, p2.x, p2.y,
                              start_point.bx, start_point.ly, end_point.bx, end_point.ly,
                              intersect_x, intersect_y))
            {
                double distance = std::hypot( awm_pt.x - intersect_x, awm_pt.y - intersect_y );
                r = distance * 0.9;   //shorten length of stop line so that it does not cross any other lanes

                start_point.bx = awm_pt.x + (r * 0.9) * cos(angle_left) + epsilon_x;
                start_point.ly = awm_pt.y + (r * 0.9) * sin(angle_left) + epsilon_y;
                end_point.bx = awm_pt.x + r * cos(angle_right) + epsilon_x;
                end_point.ly = awm_pt.y + r * sin(angle_right) + epsilon_y;
            }
        }

        //swap x and y if the coordinate is not in japanese rectangular coordinate system
        if(!isJapaneseCoordinate(awm_pt.epsg))
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
        std::vector<autoware_map_msgs::WaypointSignalRelation> wsr_vector = awm.findByFilter([&](const autoware_map_msgs::WaypointSignalRelation wsr){ return wsr.waypoint_id == wp.waypoint_id; });
        vector_map_msgs::StopLine vmap_stop_line;
        vmap_stop_line.id = stop_line_id++;
        vmap_stop_line.lid = vmap_line.lid;
        vmap_stop_line.tlid = 0;
        if( wsr_vector.empty()) {
            int road_sign_id = vmap_road_signs.size() + 1;
            vmap_road_signs.push_back(createDummyRoadSign(road_sign_id));
            vmap_stop_line.signid = road_sign_id;
        }

        vmap_stop_line.linkid = std::distance(awm_waypoint_relations.begin(), next_waypoint_relation);
        vmap_stop_lines.push_back(vmap_stop_line);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "awm_vmap_converter");
    ros::NodeHandle nh;

    AutowareMap awm;

    autoware_map::category_t awm_required_category = autoware_map::Category::AREA |
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


    ROS_INFO_STREAM("Waiting for the map to be ready");

    awm.subscribe(nh, awm_required_category, ros::Duration(5));
    if(awm.hasSubscribed(awm_required_category) == false)
    {
        ROS_WARN("Did not subscribe all required category! The converted vector map might lack some data!");
    }

    std::vector<vector_map_msgs::Point> vmap_points;
    std::vector<vector_map_msgs::Node> vmap_nodes;
    std::vector<vector_map_msgs::Area> vmap_areas;
    std::vector<vector_map_msgs::Line> vmap_lines;
    std::vector<vector_map_msgs::DTLane> vmap_dtlanes;
    std::vector<vector_map_msgs::Lane> vmap_lanes;
    std::vector<vector_map_msgs::CrossRoad> vmap_cross_roads;
    std::vector<vector_map_msgs::CrossWalk> vmap_cross_walks;
    std::vector<vector_map_msgs::WayArea> vmap_way_areas;
    std::vector<vector_map_msgs::Signal> vmap_signals;
    std::vector<vector_map_msgs::Vector> vmap_vectors;
    std::vector<vector_map_msgs::Pole> vmap_dummy_poles;
    std::vector<vector_map_msgs::StopLine> vmap_stop_lines;
    std::vector<vector_map_msgs::RoadSign> vmap_road_signs;

    createPoints(awm, vmap_points);
    createNodes(awm, vmap_nodes);
    createAreas(awm, vmap_areas, vmap_lines);
    createCrossRoads(awm, vmap_cross_roads);
    createDTLanes( awm, vmap_dtlanes,vmap_lanes);
    createCrossWalks(awm, vmap_cross_walks, vmap_areas, vmap_lines, vmap_points);
    createWayAreas(awm, vmap_way_areas);
    createSignals(  awm, vmap_signals, vmap_vectors,vmap_dummy_poles);
    createStopLines( awm, vmap_lines, vmap_points, vmap_stop_lines, vmap_road_signs);
    // createDummyRoadSign(vmap_road_signs);

    ros::Publisher point_pub = nh.advertise<vector_map_msgs::PointArray>("vector_map_info/point", 1, true);
    ros::Publisher vector_pub = nh.advertise<vector_map_msgs::VectorArray>("vector_map_info/vector", 1, true);
    ros::Publisher line_pub = nh.advertise<vector_map_msgs::LineArray>("vector_map_info/line", 1, true);
    ros::Publisher area_pub = nh.advertise<vector_map_msgs::AreaArray>("vector_map_info/area", 1, true);
    ros::Publisher dtlane_pub = nh.advertise<vector_map_msgs::DTLaneArray>("vector_map_info/dtlane", 1, true);
    ros::Publisher node_pub = nh.advertise<vector_map_msgs::NodeArray>("vector_map_info/node", 1, true);
    ros::Publisher lane_pub = nh.advertise<vector_map_msgs::LaneArray>("vector_map_info/lane", 1, true);
    ros::Publisher way_area_pub = nh.advertise<vector_map_msgs::WayAreaArray>("vector_map_info/way_area", 1, true);
    ros::Publisher stop_line_pub = nh.advertise<vector_map_msgs::StopLineArray>("vector_map_info/stop_line", 1, true);
    ros::Publisher cross_walk_pub = nh.advertise<vector_map_msgs::CrossWalkArray>("vector_map_info/cross_walk", 1, true);
    ros::Publisher signal_pub = nh.advertise<vector_map_msgs::SignalArray>("vector_map_info/signal", 1, true);
    ros::Publisher pole_pub = nh.advertise<vector_map_msgs::PoleArray>("vector_map_info/pole", 1, true);
    ros::Publisher cross_road_pub = nh.advertise<vector_map_msgs::CrossRoadArray>("vector_map_info/cross_road", 1, true);
    ros::Publisher road_sign_pub = nh.advertise<vector_map_msgs::RoadSignArray>("vector_map_info/road_sign", 1, true);

    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("vector_map", 1, true);
    ros::Publisher stat_pub = nh.advertise<std_msgs::Bool>("vmap_stat", 1, true);
    // ros::Publisher side_walk_pub = nh.advertise<vector_map_msgs::SideWalkArray>("vector_map_info/side_walk", 1, true);
    // ros::Publisher rail_crossing_pub = nh.advertise<vector_map_msgs::RailCrossingArray>("vector_map_info/rail_crossing", 1, true);


    std_msgs::Bool stat;
    stat.data = false;
    stat_pub.publish(stat);

    vector_map::category_t category = vector_map::Category::NONE;
    if(!vmap_points.empty())
    {
        point_pub.publish( createObjectArray<vector_map_msgs::Point, vector_map_msgs::PointArray>(vmap_points));
        category |= vector_map::Category::POINT;
    }
    if(!vmap_vectors.empty())
    {
        vector_pub.publish( createObjectArray<vector_map_msgs::Vector, vector_map_msgs::VectorArray>(vmap_vectors));
        category |= vector_map::Category::VECTOR;
    }
    if(!vmap_lines.empty())
    {
        line_pub.publish( createObjectArray<vector_map_msgs::Line, vector_map_msgs::LineArray>(vmap_lines));
        category |= vector_map::Category::LINE;
    }
    if(!vmap_areas.empty())
    {
        area_pub.publish( createObjectArray<vector_map_msgs::Area, vector_map_msgs::AreaArray>(vmap_areas));
        category |= vector_map::Category::AREA;
    }
    if(!vmap_dtlanes.empty())
    {
        dtlane_pub.publish( createObjectArray<vector_map_msgs::DTLane, vector_map_msgs::DTLaneArray>(vmap_dtlanes));
        category |= vector_map::Category::DTLANE;
    }
    if(!vmap_nodes.empty())
    {
        node_pub.publish( createObjectArray<vector_map_msgs::Node, vector_map_msgs::NodeArray>(vmap_nodes));
        category |= vector_map::Category::NODE;
    }
    if(!vmap_lanes.empty())
    {
        lane_pub.publish( createObjectArray<vector_map_msgs::Lane, vector_map_msgs::LaneArray>(vmap_lanes));
        category |= vector_map::Category::LANE;
    }
    if(!vmap_way_areas.empty())
    {
        way_area_pub.publish( createObjectArray<vector_map_msgs::WayArea, vector_map_msgs::WayAreaArray>(vmap_way_areas));
        category |= vector_map::Category::WAY_AREA;
    }
    if(!vmap_stop_lines.empty())
    {
        stop_line_pub.publish( createObjectArray<vector_map_msgs::StopLine, vector_map_msgs::StopLineArray>(vmap_stop_lines));
        category |= vector_map::Category::STOP_LINE;
    }
    if(!vmap_cross_walks.empty())
    {
        cross_walk_pub.publish( createObjectArray<vector_map_msgs::CrossWalk, vector_map_msgs::CrossWalkArray>(vmap_cross_walks));
        category |= vector_map::Category::CROSS_WALK;
    }
    if(!vmap_signals.empty())
    {
        signal_pub.publish( createObjectArray<vector_map_msgs::Signal, vector_map_msgs::SignalArray>(vmap_signals));
        category |= vector_map::Category::SIGNAL;
    }
    if(!vmap_cross_roads.empty())
    {
        cross_road_pub.publish( createObjectArray<vector_map_msgs::CrossRoad, vector_map_msgs::CrossRoadArray>(vmap_cross_roads));
        category |= vector_map::Category::CROSS_ROAD;
    }
    if(!vmap_dummy_poles.empty())
    {
        pole_pub.publish( createObjectArray<vector_map_msgs::Pole, vector_map_msgs::PoleArray>(vmap_dummy_poles));
        category |= vector_map::Category::POLE;
    }
    if(!vmap_road_signs.empty())
    {
        road_sign_pub.publish( createObjectArray<vector_map_msgs::RoadSign, vector_map_msgs::RoadSignArray>(vmap_road_signs));
        category |= vector_map::Category::ROAD_SIGN;
    }
    vector_map::VectorMap vmap;
    vmap.subscribe(nh, category);

    visualization_msgs::MarkerArray marker_array;
    insertMarkerArray(marker_array, createStopLineMarkerArray(vmap, vector_map::Color::WHITE));
    insertMarkerArray(marker_array, createCrossWalkMarkerArray(vmap, vector_map::Color::WHITE));
    insertMarkerArray(marker_array, createSignalMarkerArray(vmap, vector_map::Color::RED, vector_map::Color::BLUE, vector_map::Color::YELLOW, vector_map::Color::CYAN, vector_map::Color::GRAY));
    insertMarkerArray(marker_array, createCrossRoadMarkerArray(vmap, vector_map::Color::LIGHT_GREEN));
    // insertMarkerArray(marker_array, createAreaMarkerArray(vmap, vector_map::Color::WHITE));
    insertMarkerArray(marker_array, createLaneMarkerArray(vmap, vector_map::Color::YELLOW));

    marker_array_pub.publish(marker_array);

    stat.data = true;
    stat_pub.publish(stat);
    ros::spin();
}
