#include <iostream>
#include <memory>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <autoware_msgs/Lane.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>

#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class LaneBypassPlanner
{
  public:
    LaneBypassPlanner();
    ~LaneBypassPlanner(){};

  private: // ros
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher lane_pub_;
    ros::Subscriber cost_map_sub_, lane_sub_, selfpose_sub_, lane_num_sub_;
    ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void laneCallback(const autoware_msgs::Lane::ConstPtr &msg);
    void lanemunCallback(const std_msgs::Int32 &msg);
    void selfposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent &e);

    void coordinateTransformLane(const autoware_msgs::Lane &in_lane_ptr,
                                 const std::string &target_frame, autoware_msgs::Lane &out_lane_ptr);
    void generateSubLane(const autoware_msgs::Lane &base_lane, const int sub_lane_num,
                         const double sub_lane_width, std::vector<autoware_msgs::Lane> &v_sub_lane);
    void smoothingTransition(const geometry_msgs::PoseStamped &selfpose, const double &smooth_transit_dist,
                             std::vector<autoware_msgs::Lane> &v_sub_lane);
    bool calculateBypassLane(const std::vector<autoware_msgs::Lane> &v_sub_lane,
                             const std::shared_ptr<nav_msgs::OccupancyGrid> current_costmap_ptr_,
                             autoware_msgs::Lane &final_lane);
    void calculateLaneCost(const std::vector<autoware_msgs::Lane> &v_sub_lane,
                           const std::shared_ptr<nav_msgs::OccupancyGrid> current_costmap_ptr_,
                           const int check_num_max, std::vector<double> &costs);

    void debugPublishSubLane(const std::vector<autoware_msgs::Lane> &v_sub_lane);
    void debugPublishFinalLane(const autoware_msgs::Lane &lane);

    std::shared_ptr<nav_msgs::OccupancyGrid> current_costmap_ptr_;
    std::shared_ptr<autoware_msgs::Lane> current_lane_ptr_;
    std::shared_ptr<geometry_msgs::PoseStamped> current_selfpose_ptr_;

    int force_lane_change_num_;

    /* params */
    bool enable_smooth_transition_;
    bool enable_force_lane_select_;
    int sub_lane_num_odd_;
    int cost_check_num_max_;
    double sub_lane_width_;
    double smooth_transit_dist_;

    /* cost function parameters*/
    struct CostWeight
    {
        double be_center;
        double stay_there;
        double stay_there_while;
    };
    CostWeight cost_weight_;
    int best_lane_num_;
    ros::Time stay_start_time_;

    /* debug */
    ros::Publisher debug_sublane_pub_;
};

LaneBypassPlanner::LaneBypassPlanner() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{

    pnh_.param<bool>("enable_smooth_transition", enable_smooth_transition_, bool(false));
    pnh_.param<bool>("enable_force_lane_select", enable_force_lane_select_, bool(false));
    pnh_.param<int>("sub_lane_num", sub_lane_num_odd_, int(9));
    pnh_.param<double>("sub_lane_width", sub_lane_width_, double(0.8));
    sub_lane_num_odd_ = (sub_lane_num_odd_ % 2 == 0) ? sub_lane_num_odd_ + 1 : sub_lane_num_odd_; // should be odd number
    best_lane_num_ = (int)((sub_lane_num_odd_ - 1) / 2);                                          // initialize as center
    force_lane_change_num_ = best_lane_num_;
    pnh_.param<int>("cost_check_num_max", cost_check_num_max_, int(30));
    pnh_.param<double>("smooth_transit_dist", smooth_transit_dist_, double(10.0));

    pnh_.param<double>("cost_weight_be_center", cost_weight_.be_center, double(0.1));
    pnh_.param<double>("cost_weight_stay_there", cost_weight_.stay_there, double(0.1));
    pnh_.param<double>("cost_weight_stay_there_while", cost_weight_.stay_there_while, double(0.5));

    cost_map_sub_ = nh_.subscribe("/semantics/costmap_generator/occupancy_grid", 1, &LaneBypassPlanner::costmapCallback, this);
    lane_sub_ = nh_.subscribe("/in_bypass_waypoints", 1, &LaneBypassPlanner::laneCallback, this);
    selfpose_sub_ = nh_.subscribe("/current_pose", 1, &LaneBypassPlanner::selfposeCallback, this);
    lane_num_sub_ = pnh_.subscribe("/force_lane_change_number", 1, &LaneBypassPlanner::lanemunCallback, this);
    lane_pub_ = nh_.advertise<autoware_msgs::Lane>("/safety_waypoints", 1, true);

    double exec_rate;
    pnh_.param<double>("exec_rate", exec_rate, double(10));
    timer_ = nh_.createTimer(ros::Duration(1.0 / exec_rate), &LaneBypassPlanner::timerCallback, this);

    /* debug */
    debug_sublane_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/sublane", 1, true);
}

void LaneBypassPlanner::timerCallback(const ros::TimerEvent &e)
{
    /* Guard */
    if (current_costmap_ptr_ == nullptr || current_lane_ptr_ == nullptr || current_selfpose_ptr_ == nullptr)
    {
        ROS_WARN_DELAYED_THROTTLE(1.0, "waiting topic... costmap:%d, lane:%d, selfpose:%d",
                                  current_costmap_ptr_ != nullptr, current_lane_ptr_ != nullptr, current_selfpose_ptr_ != nullptr);
        return;
    }

    /* transform lane to target frame coordinate */
    autoware_msgs::Lane transformed_lane;
    coordinateTransformLane(*current_lane_ptr_, current_costmap_ptr_->header.frame_id, transformed_lane);

    std::vector<autoware_msgs::Lane> v_sub_lane;
    generateSubLane(transformed_lane, sub_lane_num_odd_, sub_lane_width_, v_sub_lane);
    if (enable_smooth_transition_)
    {
        smoothingTransition(*current_selfpose_ptr_, smooth_transit_dist_, v_sub_lane);
    }

    autoware_msgs::Lane bypass_lane;
    if (enable_force_lane_select_)
    {      
        bypass_lane = v_sub_lane.at(force_lane_change_num_);
    }
    else
    {
        if (!calculateBypassLane(v_sub_lane, current_costmap_ptr_, bypass_lane))
            return;
    }

    autoware_msgs::Lane output_msg; // in map coordinate
    coordinateTransformLane(bypass_lane, current_lane_ptr_->header.frame_id, output_msg);
    lane_pub_.publish(output_msg);

    /* debug */
    debugPublishFinalLane(output_msg);
    debugPublishSubLane(v_sub_lane);
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

void LaneBypassPlanner::smoothingTransition(const geometry_msgs::PoseStamped &selfpose, const double &smooth_transit_dist,
                                            std::vector<autoware_msgs::Lane> &v_sub_lane)
{

    /* coordinate transform from map to velodyne */
    /* ------------------------------------------------------------- */
    /* TODO: THIS SHOULD BE SUMARIZED WITH OTHER TRANSFORMATION CODE */
    /* ------------------------------------------------------------- */

    geometry_msgs::Pose my_pose;
    tf2::Transform tf_velo2world;
    try
    {
        std::string frame_from = v_sub_lane[0].header.frame_id;
        std::string frame_to = selfpose.header.frame_id;
        if (frame_from.front() == '/')
            frame_from.erase(0, 1);
        if (frame_to.front() == '/')
            frame_to.erase(0, 1);
        geometry_msgs::TransformStamped ros_velo2world;
        ros_velo2world = tf_buffer_.lookupTransform(frame_from, frame_to, selfpose.header.stamp);
        tf2::fromMsg(ros_velo2world.transform, tf_velo2world);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    tf2::Transform tf_world2selfpose, tf_velo2selfpose;
    tf2::fromMsg(selfpose.pose, tf_world2selfpose);
    tf_velo2selfpose = tf_velo2world * tf_world2selfpose;
    tf2::toMsg(tf_velo2selfpose, my_pose);
    const geometry_msgs::Point my_p = my_pose.position;

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
                                            autoware_msgs::Lane &final_lane)
{
    /* calculate path cost */
    std::vector<double> costs;
    calculateLaneCost(v_sub_lane, current_costmap_ptr_, cost_check_num_max_, costs);

    /* add extra costs */
    // printf("cost : \n");
    for (int i = 0; i < costs.size(); ++i)
    {
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
    std::vector<double>::iterator min_itr = std::min_element(costs.begin(), costs.end());
    size_t min_idx = std::distance(costs.begin(), min_itr);
    if (costs.at(min_idx) > 99.9)
    {
        ROS_WARN("Every lane has obstacles. Fail to get bypass lane.");
        return false;
    }
    final_lane = v_sub_lane.at(min_idx);
    if (best_lane_num_ != min_idx)
    {
        stay_start_time_ = ros::Time::now();
    }
    best_lane_num_ = min_idx;

    return true;
}

void LaneBypassPlanner::calculateLaneCost(const std::vector<autoware_msgs::Lane> &v_sub_lane,
                                          const std::shared_ptr<nav_msgs::OccupancyGrid> costmap_ptr,
                                          const int check_num_max, std::vector<double> &v_cost)
{
    tf2::Transform tf_target2costorigin;
    tf2::fromMsg(costmap_ptr->info.origin, tf_target2costorigin);
    int lane_num = 0;

    for (const auto &lane : v_sub_lane)
    {
        double cost = 0.0;
        int evaluate_points_num = 0;

        for (const auto &waypoint : lane.waypoints)
        {
            geometry_msgs::Pose pose;
            tf2::Transform tf_target2waypoint, tf_costorigin2waypoint;
            tf2::fromMsg(waypoint.pose.pose, tf_target2waypoint);
            tf_costorigin2waypoint = tf_target2costorigin.inverse() * tf_target2waypoint;
            tf2::toMsg(tf_costorigin2waypoint, pose);

            const double x_max = costmap_ptr->info.width * costmap_ptr->info.resolution;
            const double y_max = costmap_ptr->info.height * costmap_ptr->info.resolution;

            if (pose.position.x < 0.0 || x_max < pose.position.x || pose.position.y < 0.0 || y_max < pose.position.y)
                continue;

            const int i = std::floor(pose.position.y / costmap_ptr->info.resolution) * costmap_ptr->info.width +
                          std::floor(pose.position.x / costmap_ptr->info.resolution);
            if (i < 0 || costmap_ptr->data.size() <= i)
            {
                ROS_ERROR("something wrong in access to cosmap data. i = %d, size = %d", i, costmap_ptr->data.size());
                continue;
            }
            if (costmap_ptr->data.at(i) == 100)
            {
                cost = -1.0;
                break;
            }
            cost += costmap_ptr->data.at(i);
            ++evaluate_points_num;
            if (check_num_max < evaluate_points_num)
                break;
        }
        ++lane_num;
        if (evaluate_points_num != 0 && cost > 0.0)
            cost /= evaluate_points_num;
        else if (cost < -0.5)
            cost = 100.0;

        v_cost.push_back(cost);
    }
};

void LaneBypassPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    current_costmap_ptr_ = std::make_shared<nav_msgs::OccupancyGrid>(*msg);
}

void LaneBypassPlanner::laneCallback(const autoware_msgs::Lane::ConstPtr &msg)
{
    current_lane_ptr_ = std::make_shared<autoware_msgs::Lane>(*msg);
}

void LaneBypassPlanner::selfposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_selfpose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(*msg);
}

void LaneBypassPlanner::lanemunCallback(const std_msgs::Int32 &msg)
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

void LaneBypassPlanner::coordinateTransformLane(const autoware_msgs::Lane &in_lane,
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
        ros_target2lane = tf_buffer_.lookupTransform(check_target_frame, in_lane.header.frame_id, in_lane.header.stamp);
        tf2::fromMsg(ros_target2lane.transform, tf_target2lane);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
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
}

void LaneBypassPlanner::debugPublishSubLane(const std::vector<autoware_msgs::Lane> &v_sub_lane)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    for (const auto &lane : v_sub_lane)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = lane.header.frame_id;

        if (marker.header.frame_id.front() == '/')
        {
            marker.header.frame_id.erase(0, 1);
        }
        marker.header.stamp = lane.header.stamp;
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
        marker.color.a = 0.5f;
        for (const auto &waypoint : lane.waypoints)
        {
            marker.points.push_back(waypoint.pose.pose.position);
        }
        marker_array.markers.push_back(marker);
    }
    debug_sublane_pub_.publish(marker_array);
}

void LaneBypassPlanner::debugPublishFinalLane(const autoware_msgs::Lane &lane)
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.frame_id = lane.header.frame_id;
    if (marker.header.frame_id.front() == '/')
    {
        marker.header.frame_id.erase(0, 1);
    }
    marker.header.stamp = lane.header.stamp;
    marker.id = sub_lane_num_odd_;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.2;
    marker.scale.y = 0.5;
    marker.scale.z = 0.2;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    for (const auto &waypoint : lane.waypoints)
    {
        marker.points.push_back(waypoint.pose.pose.position);
    }
    marker_array.markers.push_back(marker);

    debug_sublane_pub_.publish(marker_array);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lane_bypass_planner");

    LaneBypassPlanner node;
    ros::spin();

    return 0;
}