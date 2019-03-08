/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lane_bypass_planner/lane_bypass_planner.h"

LaneBypassPlanner::LaneBypassPlanner() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_), INVALID_COST_(-1.0), COSTMAP_OBSTACLE_COST_(100.0)
{

    pnh_.param<bool>("enable_smooth_transition", enable_smooth_transition_, bool(false));
    pnh_.param<bool>("enable_force_lane_select", enable_force_lane_select_, bool(false));
    pnh_.param<bool>("enable_replan_when_moving", enable_replan_when_moving_, bool(true));
    pnh_.param<bool>("enable_smooth_transition_only_for_cost_calculation_and_vizualization",
                     enable_smooth_transition_only_for_cost_calculation_and_vizualization_, bool(true));
    pnh_.param<int>("sub_lane_num", sub_lane_num_odd_, int(9));
    pnh_.param<double>("sub_lane_width", sub_lane_width_, double(0.8));
    pnh_.param<int>("cost_check_num_max", cost_check_num_max_, int(30));
    pnh_.param<double>("smooth_transit_dist", smooth_transit_dist_, double(10.0));
    pnh_.param<double>("cost_weight_be_center", cost_weight_.be_center, double(0.1));
    pnh_.param<double>("cost_weight_stay_there", cost_weight_.stay_there, double(0.1));
    pnh_.param<double>("cost_weight_stay_there_while", cost_weight_.stay_there_while, double(0.5));

    std::string input_path_name, output_path_name, input_selfpose_name, input_selftwist_name, input_costmap_name;
    pnh_.param<std::string>("input_path_name", input_path_name, std::string("/lane_bypass_planner/in_path"));
    pnh_.param<std::string>("output_path_name", output_path_name, std::string("/lane_bypass_planner/out_path"));
    pnh_.param<std::string>("input_selfpose_name", input_selfpose_name, std::string("/current_pose"));
    pnh_.param<std::string>("input_selftwist_name", input_selftwist_name, std::string("/current_velocity"));
    pnh_.param<std::string>("input_costmap_name", input_costmap_name, std::string("/semantics/costmap_generator/occupancy_grid"));

    cost_map_sub_ = nh_.subscribe(input_costmap_name, 1, &LaneBypassPlanner::costmapCallback, this);
    lane_sub_ = nh_.subscribe(input_path_name, 1, &LaneBypassPlanner::laneCallback, this);
    selfpose_sub_ = nh_.subscribe(input_selfpose_name, 1, &LaneBypassPlanner::selfposeCallback, this);
    selftwist_sub_ = nh_.subscribe(input_selftwist_name, 1, &LaneBypassPlanner::selftwistCallback, this);
    lane_num_sub_ = pnh_.subscribe("/force_lane_change_number", 1, &LaneBypassPlanner::lanenumCallback, this);
    lane_pub_ = nh_.advertise<autoware_msgs::Lane>(output_path_name, 1, true);

    double exec_rate;
    pnh_.param<double>("exec_rate", exec_rate, double(10));
    timer_ = nh_.createTimer(ros::Duration(1.0 / exec_rate), &LaneBypassPlanner::timerCallback, this);

    sub_lane_num_odd_ = (sub_lane_num_odd_ % 2 == 0) ? sub_lane_num_odd_ + 1 : sub_lane_num_odd_; // should be odd number
    center_lane_num_ = (int)((sub_lane_num_odd_ - 1) / 2);                                        // initialize as center
    best_lane_num_ = center_lane_num_;
    force_lane_change_num_ = center_lane_num_;

    /* debug */
    debug_sublane_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/sublane", 1, true);
}

void LaneBypassPlanner::timerCallback(const ros::TimerEvent &e)
{
    /* Guard */
    if (current_costmap_ptr_ == nullptr || current_lane_ptr_ == nullptr ||
        current_selfpose_ptr_ == nullptr || current_selftwist_ptr_ == nullptr)
    {
        ROS_WARN_DELAYED_THROTTLE(3.0, "waiting topic... costmap:%d, lane:%d, selfpose:%d, selftwist:%d",
                                  current_costmap_ptr_ != nullptr, current_lane_ptr_ != nullptr,
                                  current_selfpose_ptr_ != nullptr, current_selftwist_ptr_ != nullptr);
        return;
    }

    /* transform lane to target frame coordinate */
    autoware_msgs::Lane transformed_lane;
    if (!coordinateTransformLane(*current_lane_ptr_, current_costmap_ptr_->header.frame_id, transformed_lane))
        return;

    /* if flag is false, don't plan sublane when following the original lane & velocity > 0 */
    bool force_center = false;
    if (!enable_replan_when_moving_ && std::fabs(current_selftwist_ptr_->twist.linear.x) > 1.0 &&
        best_lane_num_ == center_lane_num_)
    {
        force_center = true;
    }

    /* generate sublanes with parallel shift */
    std::vector<autoware_msgs::Lane> v_sub_lane, v_sub_lane_orig;
    generateSubLane(transformed_lane, sub_lane_num_odd_, sub_lane_width_, v_sub_lane);

    /* change the beginning of the sublane smoothly */ 
    if (enable_smooth_transition_ || enable_smooth_transition_only_for_cost_calculation_and_vizualization_)
    {
        v_sub_lane_orig = v_sub_lane;
        smoothingTransition(*current_selfpose_ptr_, smooth_transit_dist_, v_sub_lane);
    }

    int min_cost_index;
    std::vector<double> v_costs(sub_lane_num_odd_, INVALID_COST_);

    /* calculate the best sublane */
    autoware_msgs::Lane bypass_lane;
    if (enable_force_lane_select_)
    {      
        min_cost_index = force_lane_change_num_;
        bypass_lane = v_sub_lane.at(force_lane_change_num_);
        v_costs.at(force_lane_change_num_) = 0.0;
    }
    else if (force_center)
    {   
        min_cost_index = center_lane_num_;
        bypass_lane = v_sub_lane.at(center_lane_num_);
        v_costs.at(center_lane_num_) = 0.0;
    }
    else
    {
        if (!calculateBypassLane(v_sub_lane, current_costmap_ptr_, bypass_lane, v_costs, min_cost_index)) {
            min_cost_index = center_lane_num_;
            bypass_lane = v_sub_lane.at(center_lane_num_);
        }
            
    }
    if (enable_smooth_transition_only_for_cost_calculation_and_vizualization_) {
        bypass_lane = v_sub_lane_orig.at(min_cost_index);
    }

    /* coordinate transform to original waypoints frame_id */
    autoware_msgs::Lane output_msg; 
    coordinateTransformLane(bypass_lane, current_lane_ptr_->header.frame_id, output_msg);

    /* publish */
    lane_pub_.publish(output_msg);

    /* debug */
    debugPublishSubLane(v_sub_lane, v_costs);
}

void LaneBypassPlanner::generateSubLane(const autoware_msgs::Lane &base_lane, const int sub_lane_num,
                                        const double sub_lane_width, std::vector<autoware_msgs::Lane> &v_sub_lane)
{
    const int num = (sub_lane_num % 2 == 0) ? sub_lane_num + 1 : sub_lane_num; // should be odd number
    double length = (-1.0) * (num - 1.0) / 2.0 * sub_lane_width;

    /* generate sublane */
    for (int i = 0; i < num; ++i)
    {
        autoware_msgs::Lane lane = base_lane;
        for (auto &waypoint : lane.waypoints)
        {
            const double yaw = tf2::getYaw(waypoint.pose.pose.orientation);
            waypoint.pose.pose.position.x += length * sin(yaw);
            waypoint.pose.pose.position.y -= length * cos(yaw);
        }
        v_sub_lane.push_back(lane);

        length += sub_lane_width;
    }
}

bool LaneBypassPlanner::transformRosPose(const geometry_msgs::PoseStamped &in_pose, const std::string &target_frame,
                                         geometry_msgs::PoseStamped &out_pose)
{
    geometry_msgs::Pose my_pose;
    tf2::Transform tf_in2out;
    try
    {
        std::string frame_from = in_pose.header.frame_id;
        std::string frame_to = target_frame;
        if (frame_from.front() == '/')
            frame_from.erase(0, 1);
        if (frame_to.front() == '/')
            frame_to.erase(0, 1);
        geometry_msgs::TransformStamped ros_in2out;
        ros_in2out = tf_buffer_.lookupTransform(frame_to, frame_from, ros::Time(0));
        tf2::fromMsg(ros_in2out.transform, tf_in2out);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }
    tf2::Transform tf_in2pose, tf_out2pose;
    tf2::fromMsg(in_pose.pose, tf_in2pose);
    tf_out2pose = tf_in2out * tf_in2pose;
    tf2::toMsg(tf_out2pose, my_pose);
    out_pose.pose = my_pose;
    out_pose.header.frame_id = target_frame;
    out_pose.header.stamp = in_pose.header.stamp;

    return true;
}

void LaneBypassPlanner::smoothingTransition(const geometry_msgs::PoseStamped &selfpose, const double &smooth_transit_dist,
                                            std::vector<autoware_msgs::Lane> &v_sub_lane)
{

    /* coordinate transform from map to velodyne */
    geometry_msgs::PoseStamped my_posestamped;
    transformRosPose(selfpose, v_sub_lane[0].header.frame_id, my_posestamped);
    const geometry_msgs::Point my_p = my_posestamped.pose.position;

    /* calculate smoothing parameter */
    std::vector<double> v_tanh;
    {
        const double tanh_smoothness_factor = 2.1;
        double smooth_dist_dx = (2.0 * tanh_smoothness_factor) / smooth_transit_dist;
        double x = -tanh_smoothness_factor;
        for (int i = 0; i < (int)smooth_transit_dist; ++i)
        {
            const double y = (std::tanh(x) + 1.0) / 2.0;
            v_tanh.push_back(y);
            x += smooth_dist_dx;
        }
    }

    /* calculate nearest lane from self pose */
    int nearest_i = 0;
    double min_dist_squared = 1.0E9;
    for (int i = 0; i < v_sub_lane.size(); ++i)
    {
        const double dx = my_p.x - v_sub_lane.at(i).waypoints.at(0).pose.pose.position.x;
        const double dy = my_p.y - v_sub_lane.at(i).waypoints.at(0).pose.pose.position.y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_squared)
        {
            min_dist_squared = dist_sq;
            nearest_i = i;
        }
    }

    /* smoothing lane from nearest lane */
    for (auto &lane : v_sub_lane)
    {
        for (int i = 0; i < lane.waypoints.size() && i < (int)smooth_transit_dist; ++i)
        {
            geometry_msgs::Point *p = &lane.waypoints.at(i).pose.pose.position;
            geometry_msgs::Point *ref_p = &v_sub_lane.at(nearest_i).waypoints.at(i).pose.pose.position;
            p->x = p->x * v_tanh.at(i) + ref_p->x * (1.0 - v_tanh.at(i));
            p->y = p->y * v_tanh.at(i) + ref_p->y * (1.0 - v_tanh.at(i));
        }
    }
}

bool LaneBypassPlanner::calculateBypassLane(const std::vector<autoware_msgs::Lane> &v_sub_lane,
                                            const std::shared_ptr<nav_msgs::OccupancyGrid> current_costmap_ptr_,
                                            autoware_msgs::Lane &final_lane, std::vector<double> &costs, int &min_cost_index)
{
    /* calculate path cost */
    calculateLaneCost(v_sub_lane, current_costmap_ptr_, cost_check_num_max_, costs);

    /* add extra costs */
    // printf("cost : \n");
    for (int i = 0; i < costs.size(); ++i)
    {
        if (costs.at(i) == INVALID_COST_)
            continue;

        // printf("num = %d, obstacle_cost = %f, ", i, costs.at(i));
        // add distance cost from center line
        const double length_from_center = std::fabs(((-1.0) * (sub_lane_num_odd_ - 1.0) / 2.0 + i) * sub_lane_width_);
        const double be_center_cost = cost_weight_.be_center * length_from_center;
        costs.at(i) += be_center_cost;

        // add cost to choose the same one as before
        const double length_from_last = std::fabs(best_lane_num_ - i) * sub_lane_width_;
        const double stay_there_cost = cost_weight_.stay_there * length_from_last * length_from_last;
        costs.at(i) += stay_there_cost;

        // add cost to choose the same one as before
        const double stay_time_max = 5.0;
        const double staying_time = (ros::Time::now() - stay_start_time_).toSec();
        const double time_effect = std::min(1.0, std::max(stay_time_max - staying_time, 0.0) / stay_time_max);
        const double stay_there_while_cost = cost_weight_.stay_there_while * length_from_last * time_effect;
        costs.at(i) += stay_there_while_cost;

        // printf("be_center_cost = %f, stay_there_cost = %f, stay_there_while_cost = %f, total = %f\n", be_center_cost, stay_there_cost, stay_there_while_cost, costs.at(i));
    }
    // printf("\n");

    /* get minimum cost lane */
    double min_cost = std::numeric_limits<double>::max();
    min_cost_index = -1;
    for (int i = 0; i < (int)costs.size(); ++i) {
        if (costs.at(i) == INVALID_COST_)
            continue;
        
        if (costs.at(i) < min_cost) {
            min_cost_index = i;
            min_cost = costs.at(i);
        }
    }
    if (min_cost_index == -1)
    {
        ROS_WARN("Every lane has obstacles. Fail to get bypass lane.");
        return false;
    }
    final_lane = v_sub_lane.at(min_cost_index);
    if (best_lane_num_ != min_cost_index)
    {
        stay_start_time_ = ros::Time::now();
    }
    best_lane_num_ = min_cost_index;

    return true;
}

void LaneBypassPlanner::calculateLaneCost(const std::vector<autoware_msgs::Lane> &v_sub_lane,
                                          const std::shared_ptr<nav_msgs::OccupancyGrid> costmap_ptr,
                                          const int check_num_max, std::vector<double> &v_cost)
{
    v_cost.clear();
    tf2::Transform tf_target2costorigin;
    tf2::fromMsg(costmap_ptr->info.origin, tf_target2costorigin);
    int lane_num = 0;

    for (const auto &lane : v_sub_lane)
    {
        double cost = 0.0;
        int evaluate_points_num = 0;

        for (const auto &waypoint : lane.waypoints)
        {
            /* convert waypoint positino to costmap origin coordinate */
            geometry_msgs::Pose pose;
            tf2::Transform tf_target2waypoint, tf_costorigin2waypoint;
            tf2::fromMsg(waypoint.pose.pose, tf_target2waypoint);
            tf_costorigin2waypoint = tf_target2costorigin.inverse() * tf_target2waypoint;
            tf2::toMsg(tf_costorigin2waypoint, pose);

            const double x_max = costmap_ptr->info.width * costmap_ptr->info.resolution;
            const double y_max = costmap_ptr->info.height * costmap_ptr->info.resolution;

            /* out of range */
            if (pose.position.x < 0.0 || x_max < pose.position.x || pose.position.y < 0.0 || y_max < pose.position.y)
                continue;

            const int i = std::floor(pose.position.y / costmap_ptr->info.resolution) * costmap_ptr->info.width +
                          std::floor(pose.position.x / costmap_ptr->info.resolution);
            if (i < 0 || costmap_ptr->data.size() <= i)
            {
                ROS_ERROR("something wrong in access to cosmap data. i = %d, size = %d", i, (int)costmap_ptr->data.size());
                continue;
            }

            /* if there is an obstacle in the lane, set the lane as invalid */
            if (costmap_ptr->data.at(i) >= COSTMAP_OBSTACLE_COST_)
            {
                cost = INVALID_COST_;
                break;
            }

            /* accumurate costs */
            cost += costmap_ptr->data.at(i);
            ++evaluate_points_num;
            if (check_num_max < evaluate_points_num)
                break;
        }
        ++lane_num;

        /* take average cost fot lane */
        if (evaluate_points_num = 0) 
            cost = INVALID_COST_;
        if (cost != INVALID_COST_)
            cost /= evaluate_points_num;

        v_cost.push_back(cost);
    }
};

void LaneBypassPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    current_costmap_ptr_ = std::make_shared<nav_msgs::OccupancyGrid>(*msg);
}

void LaneBypassPlanner::laneCallback(const autoware_msgs::Lane::ConstPtr &msg)
{
    if (msg->waypoints.size() == 0) {
        ROS_WARN("waypoints size in lane message is zero. ignore planning.");
        return;
    }
    current_lane_ptr_ = std::make_shared<autoware_msgs::Lane>(*msg);
}

void LaneBypassPlanner::selfposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_selfpose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(*msg);
}

void LaneBypassPlanner::selftwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_selftwist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
}

void LaneBypassPlanner::lanenumCallback(const std_msgs::Int32 &msg)
{
    if (msg.data > sub_lane_num_odd_ - 1) {
        ROS_INFO("received no.%d, but lane num is %d. set %d", msg.data, sub_lane_num_odd_, sub_lane_num_odd_ - 1);
        force_lane_change_num_ = sub_lane_num_odd_ - 1;
    } else if (msg.data < 0) {
        ROS_INFO("received no.%d. set %d", msg.data, 0);
        force_lane_change_num_ = 0;
    } else {
        force_lane_change_num_ = msg.data;
    }
}

bool LaneBypassPlanner::coordinateTransformLane(const autoware_msgs::Lane &in_lane,
                                                const std::string &target_frame,
                                                autoware_msgs::Lane &out_lane)
{

    tf2::Transform tf_target2lane;
    std::string check_target_frame = target_frame;
    try
    {
        if (check_target_frame.front() == '/')
        {
            check_target_frame.erase(0, 1);
        }
        geometry_msgs::TransformStamped ros_target2lane;
        ros_target2lane = tf_buffer_.lookupTransform(check_target_frame, in_lane.header.frame_id, ros::Time(0)); // lane header stamp is inappropiate. just use ros::Time(0) here.
        tf2::fromMsg(ros_target2lane.transform, tf_target2lane);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }

    out_lane = in_lane;
    for (auto &waypoint : out_lane.waypoints)
    {
        tf2::Transform tf_target2waypoint, tf_lane2waypoint;
        tf2::fromMsg(waypoint.pose.pose, tf_lane2waypoint);
        tf_target2waypoint = tf_target2lane * tf_lane2waypoint;
        tf2::toMsg(tf_target2waypoint, waypoint.pose.pose);
    }
    out_lane.header.frame_id = check_target_frame;
    return true;
}


void LaneBypassPlanner::debugPublishSubLane(const std::vector<autoware_msgs::Lane> &v_sub_lane, const std::vector<double> &v_cost)
{
    if (debug_sublane_pub_.getNumSubscribers() == 0)
        return;

    if (v_sub_lane.size() == 0)
        return;

    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    std::string header_string = current_lane_ptr_->header.frame_id;
    const double max_cost = std::max(*std::max_element(v_cost.begin(), v_cost.end()), 0.001);

    const int display_point = std::min(10, (int)v_sub_lane[0].waypoints.size()-1);

    if (header_string.front() == '/')
    {
        header_string.erase(0, 1);
    }
    int lane_num = 0;
    for (const auto &lane : v_sub_lane)
    {
        /* convert to lane original coordinate */
        autoware_msgs::Lane lane_map;
        coordinateTransformLane(lane, current_lane_ptr_->header.frame_id, lane_map);
        visualization_msgs::Marker marker;

        /* for lane */
        marker.header.frame_id = header_string;
        marker.header.stamp = ros::Time(0);
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();
        marker.scale.x = 0.1;
        marker.scale.y = 0.5;
        marker.scale.z = 0.2;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        float tmp = (float)(1.0 - 0.8 * v_cost.at(lane_num) / max_cost);
        marker.color.a = tmp;
        if (lane_num == best_lane_num_) {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        if (v_cost.at(lane_num) == INVALID_COST_) {
            marker.color.r = 0.6f;
            marker.color.g = 0.6f;
            marker.color.b = 0.6f;
            marker.color.a = 0.5f;
        }

        for (int i = 0; i < lane_map.waypoints.size() && i < cost_check_num_max_; ++i) {
            marker.points.push_back(lane_map.waypoints.at(i).pose.pose.position);
        }
        marker_array.markers.push_back(marker);

        /* to display cost */
        marker.header.frame_id = header_string;
        marker.header.stamp = ros::Time(0);
        marker.id = id++;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.0;
        marker.scale.y = 0.0;
        marker.scale.z = 0.4;
        marker.color.r = 0.8f;
        marker.color.g = 0.8f;
        marker.color.b = 0.8f;
        marker.color.a = 1.0f;
        char str[10];
        std::sprintf(str, "%.2f", v_cost.at(lane_num));
        marker.text = str;
        marker.pose.position = lane_map.waypoints[display_point].pose.pose.position;
        marker_array.markers.push_back(marker);

        ++lane_num;
    }
    debug_sublane_pub_.publish(marker_array);
}
