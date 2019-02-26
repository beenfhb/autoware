/*
 *  Copyright (c) 2018, TierIV, Inc
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

#include "stop_area_core.h"

#include <string>
#include <utility>
#include <algorithm>


StopArea::StopArea(ros::NodeHandle nh, ros::NodeHandle private_nh)
	: nh_(nh)
	, private_nh_(private_nh)
	//	,tf_listener_(tf_buffer_)
	, baselink_frame_("/base_link")
	, sim_time_(1.0)
	, sim_time_delta_(0.1)
	, filtering_radius_(0.5)
	, filtering_points_size_(5)
{
		private_nh_.param<std::string>("baselink_frame",  baselink_frame_, baselink_frame_);

		private_nh_.param("sim_time", sim_time_, sim_time_);
		private_nh_.param("sim_time_delta", sim_time_delta_, sim_time_delta_);
		private_nh_.param("filtering_radius", filtering_radius_, filtering_radius_);
		private_nh_.param("filtering_points_size", filtering_points_size_, filtering_points_size_);

		Rectangular offset_rect;
		private_nh_.param("offset_front",  offset_rect.front,   0.1);
		private_nh_.param("offset_rear",   offset_rect.rear,    0.1);
		private_nh_.param("offset_left",   offset_rect.left,    0.1);
		private_nh_.param("offset_right",  offset_rect.right,   0.1);
		private_nh_.param("offset_top",    offset_rect.top,     0.1);
		private_nh_.param("offset_bottom", offset_rect.bottom,  -0.3);

		bool use_vehicle_info_param = true;
		private_nh_.param("use_vehicle_info_param",  use_vehicle_info_param, use_vehicle_info_param);

		vehicle_info_.length = 5.0;
		vehicle_info_.width = 2.0;
		vehicle_info_.height = 2.0;
		vehicle_info_.wheel_base = 2.95;
		vehicle_info_.tread_front = 1.5;
		vehicle_info_.tread_rear = 1.5;
		vehicle_info_.center_to_base = -1.5;

		if(use_vehicle_info_param) {
				nh_.param("/actuation/vehicle_info/length",          vehicle_info_.length,        vehicle_info_.length);
				nh_.param("/actuation/vehicle_info/width",           vehicle_info_.width,          vehicle_info_.width);
				nh_.param("/actuation/vehicle_info/height",          vehicle_info_.height,         vehicle_info_.height);
				nh_.param("/actuation/vehicle_info/wheel_base",      vehicle_info_.wheel_base,     vehicle_info_.wheel_base);
				nh_.param("/actuation/vehicle_info/tread_front",     vehicle_info_.tread_front,    vehicle_info_.tread_front);
				nh_.param("/actuation/vehicle_info/tread_rear",      vehicle_info_.tread_rear,     vehicle_info_.tread_rear);
				nh_.param("/actuation/vehicle_info/center_to_base",  vehicle_info_.center_to_base, vehicle_info_.center_to_base);
		}
		else {
			private_nh_.param("vehicle_info_length",          vehicle_info_.length,        vehicle_info_.length);
			private_nh_.param("vehicle_info_width",           vehicle_info_.width,          vehicle_info_.width);
			private_nh_.param("vehicle_info_height",          vehicle_info_.height,         vehicle_info_.height);
			private_nh_.param("vehicle_info_wheel_base",      vehicle_info_.wheel_base,     vehicle_info_.wheel_base);
			private_nh_.param("vehicle_info_tread_front",     vehicle_info_.tread_front,    vehicle_info_.tread_front);
			private_nh_.param("vehicle_info_tread_rear",      vehicle_info_.tread_rear,     vehicle_info_.tread_rear);
			private_nh_.param("vehicle_info_center_to_base",  vehicle_info_.center_to_base, vehicle_info_.center_to_base);
		}

		vehicle_rect_.front  =  vehicle_info_.length / 2.0 - vehicle_info_.center_to_base;
		vehicle_rect_.rear   = -vehicle_info_.length / 2.0 - vehicle_info_.center_to_base;
		vehicle_rect_.left   =  vehicle_info_.width  / 2.0;
		vehicle_rect_.right  = -vehicle_info_.width  / 2.0;
		vehicle_rect_.top    =  vehicle_info_.height;
		vehicle_rect_.bottom = 0;

		safety_rect_.front  = vehicle_rect_.front  + offset_rect.front;
		safety_rect_.rear   = vehicle_rect_.rear   - offset_rect.rear;
		safety_rect_.left   = vehicle_rect_.left   + offset_rect.left;
		safety_rect_.right  = vehicle_rect_.right  - offset_rect.right;
		safety_rect_.top    = vehicle_rect_.top    + offset_rect.top;
		safety_rect_.bottom = vehicle_rect_.bottom - offset_rect.bottom;

		config_sub_ = nh_.subscribe("/config/stop_area", 1, &StopArea::configCallback, this);
		points_sub_ = nh_.subscribe("/points_raw", 1, &StopArea::pointCloudCallback, this);
		twist_sub_ = nh_.subscribe("/current_velocity", 1, &StopArea::twistCallback, this);
		vehicle_cmd_sub_ = nh_.subscribe("/vehicle_cmd", 1, &StopArea::vehicleCmdCallback, this);
		vehicle_status_sub_ = nh_.subscribe("/vehicle_status", 1, &StopArea::vehicleStatusCallback, this);

		emergency_flag_pub_ = private_nh_.advertise<std_msgs::Int32>("emergency_flag", 10);
		diag_pub_ = private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("diag", 10);
		points_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("points_obstacle", 10);

		marker_array_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 10);
}

void StopArea::configCallback(const autoware_config_msgs::ConfigStopArea::ConstPtr& config_msg_ptr)
{
		sim_time_ = config_msg_ptr->sim_time;
		sim_time_delta_ = config_msg_ptr->sim_time_delta;

		filtering_radius_ = config_msg_ptr->filtering_radius;
		filtering_points_size_ = config_msg_ptr->filtering_points_size;

		Rectangular offset_rect;
		offset_rect.front   = config_msg_ptr->offset_front;
		offset_rect.rear    = config_msg_ptr->offset_rear;
		offset_rect.left    = config_msg_ptr->offset_left;
		offset_rect.right   = config_msg_ptr->offset_right;
		offset_rect.top     = config_msg_ptr->offset_top;
		offset_rect.bottom  = config_msg_ptr->offset_bottom;

		bool use_vehicle_info_param = config_msg_ptr->use_vehicle_info_param;
		if(use_vehicle_info_param) {
      nh_.param("/actuation/vehicle_info/length",          vehicle_info_.length,        vehicle_info_.length);
      nh_.param("/actuation/vehicle_info/width",           vehicle_info_.width,          vehicle_info_.width);
      nh_.param("/actuation/vehicle_info/height",          vehicle_info_.height,         vehicle_info_.height);
      nh_.param("/actuation/vehicle_info/wheel_base",      vehicle_info_.wheel_base,     vehicle_info_.wheel_base);
      nh_.param("/actuation/vehicle_info/tread_front",     vehicle_info_.tread_front,    vehicle_info_.tread_front);
      nh_.param("/actuation/vehicle_info/tread_rear",      vehicle_info_.tread_rear,     vehicle_info_.tread_rear);
      nh_.param("/actuation/vehicle_info/center_to_base",  vehicle_info_.center_to_base, vehicle_info_.center_to_base);
    }
    else {
      vehicle_info_.length         = config_msg_ptr->vehicle_info_length;
      vehicle_info_.width          = config_msg_ptr->vehicle_info_width;
      vehicle_info_.height         = config_msg_ptr->vehicle_info_height;
      vehicle_info_.wheel_base     = config_msg_ptr->vehicle_info_wheel_base;
      vehicle_info_.tread_front    = config_msg_ptr->vehicle_info_tread_front;
      vehicle_info_.tread_rear     = config_msg_ptr->vehicle_info_tread_rear;
      vehicle_info_.center_to_base = config_msg_ptr->vehicle_info_center_to_base;
    }

    vehicle_rect_.front  =  vehicle_info_.length / 2.0 - vehicle_info_.center_to_base;
    vehicle_rect_.rear   = -vehicle_info_.length / 2.0 - vehicle_info_.center_to_base;
    vehicle_rect_.left   =  vehicle_info_.width  / 2.0;
    vehicle_rect_.right  = -vehicle_info_.width  / 2.0;
    vehicle_rect_.top    =  vehicle_info_.height;
    vehicle_rect_.bottom = 0;

    safety_rect_.front  = vehicle_rect_.front  + offset_rect.front;
    safety_rect_.rear   = vehicle_rect_.rear   - offset_rect.rear;
    safety_rect_.left   = vehicle_rect_.left   + offset_rect.left;
    safety_rect_.right  = vehicle_rect_.right  - offset_rect.right;
    safety_rect_.top    = vehicle_rect_.top    + offset_rect.top;
    safety_rect_.bottom = vehicle_rect_.bottom - offset_rect.bottom;
}

void StopArea::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg_ptr)
{
    current_twist_msg_ = *twist_msg_ptr;
}

void StopArea::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& vehicle_cmd_msg_ptr)
{
    vehicle_cmd_msg_ = *vehicle_cmd_msg_ptr;
}

void StopArea::vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status_msg_ptr)
{
    vehicle_status_msg_ = *vehicle_status_msg_ptr;
}

void StopArea::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_in_cloud_msg_ptr)
{
		std::string sensor_frame = sensorTF_in_cloud_msg_ptr->header.frame_id;
		ros::Time sensor_time = sensorTF_in_cloud_msg_ptr->header.stamp;
		sensor_msgs::PointCloud2::Ptr baselinkTF_in_cloud_msg_ptr(new sensor_msgs::PointCloud2);
    try {
        tf_listener_.waitForTransform(baselink_frame_, sensor_frame, sensor_time, ros::Duration(1.0));
        pcl_ros::transformPointCloud(baselink_frame_, *sensorTF_in_cloud_msg_ptr, *baselinkTF_in_cloud_msg_ptr, tf_listener_);
    }
    catch (tf::TransformException& ex) {
        ROS_ERROR("Transform error: %s", ex.what());
        return;
    }
	// geometry_msgs::TransformStamped transform_stamped;
	// try {
	// 	transform_stamped = tf_buffer_.lookupTransform(baselink_frame_, sensor_frame, ros::Time(0), ros::Duration(1.0));
	// }
	// catch (tf2::TransformException &ex) {
	// 	ROS_WARN("%s", ex.what());
	// 	return;
	// }
	// tf2::doTransform(*sensorTF_in_cloud_msg_ptr, *baselinkTF_in_cloud_msg_ptr, transform_stamped);

		pcl::PointCloud<pcl::PointXYZ>::Ptr baselinkTF_in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*baselinkTF_in_cloud_msg_ptr, *baselinkTF_in_cloud_ptr);

    const auto polygon = createVehicleTrajectoryPolygon();

    const auto min_max_x_iter = std::minmax_element(std::begin(polygon), std::end(polygon),
        [](const Point2d& lhs, const Point2d& rhs) {
            return lhs.x < rhs.x;
        }
    );
    const auto min_max_y_iter = std::minmax_element(std::begin(polygon), std::end(polygon),
        [](const Point2d& lhs, const Point2d& rhs) {
            return lhs.y < rhs.y;
        }
    );

    Rectangular min_max_rect;
    min_max_rect.front  = min_max_x_iter.second->x;
    min_max_rect.rear   = min_max_x_iter.first->x;
    min_max_rect.left   = min_max_y_iter.second->y;
    min_max_rect.right  = min_max_y_iter.first->y;
    min_max_rect.top    = safety_rect_.top;
    min_max_rect.bottom = safety_rect_.bottom;

    auto baselinkTF_filtered_cloud_ptr = extractPointsInRect(baselinkTF_in_cloud_ptr, min_max_rect);

    pcl::PointCloud<pcl::PointXYZ>::Ptr baselinkTF_stop_area_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    auto baselinkTF_arc_area_cloud_ptr = insidePointCloud(baselinkTF_filtered_cloud_ptr, polygon);
    auto baselinkTF_stop_area_cloud_removed_footprint_ptr = removePointsInRect(baselinkTF_arc_area_cloud_ptr, vehicle_rect_);
    auto baselinkTF_stop_area_cloud_removed_footprint_outlier_ptr = removeOutlierPoints(baselinkTF_stop_area_cloud_removed_footprint_ptr);


//----
    diagnostic_msgs::DiagnosticArray diag_array_msg;
    diag_array_msg.header.frame_id = "";
    diag_array_msg.header.stamp = sensor_time;
    diagnostic_msgs::DiagnosticStatus stat_msg;
    stat_msg.hardware_id = "emergency_stop";
    stat_msg.name = "stop_area";

    std_msgs::Int32 emergency_flag_msg;

    if(vehicle_cmd_msg_.twist_cmd.twist.linear.x < 0.01 && current_twist_msg_.twist.linear.x < 0.01) {
        stat_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
        stat_msg.message = "vehicle has stopped";
        emergency_flag_msg.data = static_cast<int>(EmergencyFlag::OK);
    }
    else if(baselinkTF_stop_area_cloud_removed_footprint_outlier_ptr->points.empty()) {
        stat_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
        stat_msg.message = "OK";
        emergency_flag_msg.data = static_cast<int>(EmergencyFlag::OK);
    }
    else {
        stat_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat_msg.message = "found obstacle";
        emergency_flag_msg.data = static_cast<int>(EmergencyFlag::Stop);
    }

    diag_array_msg.status.push_back(stat_msg);
    diag_pub_.publish(diag_array_msg);
    emergency_flag_pub_.publish(emergency_flag_msg);

//----

    sensor_msgs::PointCloud2 baselinkTF_stop_area_cloud_removed_footprint_outlier_ptr_msg;
    pcl::toROSMsg(*baselinkTF_stop_area_cloud_removed_footprint_outlier_ptr, baselinkTF_stop_area_cloud_removed_footprint_outlier_ptr_msg);
    baselinkTF_stop_area_cloud_removed_footprint_outlier_ptr_msg.header.frame_id = "/base_link";
    baselinkTF_stop_area_cloud_removed_footprint_outlier_ptr_msg.header.stamp = sensor_time;
    points_pub_.publish(baselinkTF_stop_area_cloud_removed_footprint_outlier_ptr_msg);

}

template <class PointType>
boost::shared_ptr< pcl::PointCloud<PointType> > StopArea::extractPointsInRect(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr, const Rectangular& rect)
{
    boost::shared_ptr< pcl::PointCloud<PointType> > out_cloud_ptr(new pcl::PointCloud<PointType>);
    out_cloud_ptr->points.reserve( in_cloud_ptr->points.size() );
    for (const auto& point : in_cloud_ptr->points) {
			if (point.x < rect.front && point.x > rect.rear  &&
			    point.y < rect.left  && point.y > rect.right &&
					point.z < rect.top   && point.z > rect.bottom  )
			{
						out_cloud_ptr->points.push_back(point);
			}
	}
    return out_cloud_ptr;
}

template <class PointType>
boost::shared_ptr< pcl::PointCloud<PointType> > StopArea::removePointsInRect(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr, const Rectangular& rect)
{
    boost::shared_ptr< pcl::PointCloud<PointType> > out_cloud_ptr(new pcl::PointCloud<PointType>);
    out_cloud_ptr->points.reserve( in_cloud_ptr->points.size() );
    for (const auto& point : in_cloud_ptr->points) {
		if (point.x > rect.front || point.x < rect.rear  ||
		    point.y > rect.left  || point.y < rect.right ||
				point.z > rect.top   || point.z < rect.bottom  )
		{
			out_cloud_ptr->points.push_back(point);
		}
	}
    return out_cloud_ptr;
}

template <class PointType>
boost::shared_ptr< pcl::PointCloud<PointType> > StopArea::removeOutlierPoints(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr)
{
    boost::shared_ptr< pcl::PointCloud<PointType> > out_cloud_ptr(new pcl::PointCloud<PointType>);
    if(in_cloud_ptr->points.empty()) {
        return out_cloud_ptr;
    }
    out_cloud_ptr->points.reserve( in_cloud_ptr->points.size() );
    //TODO
    pcl::RadiusOutlierRemoval<PointType> outrem;
    outrem.setInputCloud(in_cloud_ptr);
    outrem.setRadiusSearch(filtering_radius_);
    outrem.setMinNeighborsInRadius (filtering_points_size_);
    outrem.filter (*out_cloud_ptr);
    return out_cloud_ptr;
}


template <class PointType>
boost::shared_ptr< pcl::PointCloud<PointType> > StopArea::insidePointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> > & pointcloud, const std::vector<Point2d>& polygon)
{
    boost::shared_ptr< pcl::PointCloud<PointType> > output_pointcloud(new pcl::PointCloud<PointType>);

    for(const auto& point : pointcloud->points){
        bool is_inside = isInPolygon(Point2d(point.x, point.y), polygon);
        if(is_inside){
            output_pointcloud->points.push_back(point);
        }
    }
    return output_pointcloud;
}


std::vector<Point2d> StopArea::createVehicleTrajectoryPolygon()
{
    //calc turning radius
    const double v = current_twist_msg_.twist.linear.x;
    const double w = current_twist_msg_.twist.angular.z;
    const double r = current_twist_msg_.twist.angular.z != 0 ? current_twist_msg_.twist.linear.x/current_twist_msg_.twist.angular.z : 10000;

    //calc turning center point
    Point2d tier_front_left(vehicle_info_.wheel_base,   vehicle_info_.tread_front / 2.0);
    Point2d tier_front_right(vehicle_info_.wheel_base, -vehicle_info_.tread_front / 2.0);
    Point2d tier_rear_left(0,   vehicle_info_.tread_rear / 2.0);
    Point2d tier_rear_right(0, -vehicle_info_.tread_rear / 2.0);

    Point2d circle_center;
    if(w >= 0) {
        lncl(Point2d(tier_rear_left.x, tier_rear_left.y), Point2d(tier_rear_right.x, tier_rear_right.y), Point2d(vehicle_info_.wheel_base, 0), r, Point2d(tier_rear_left.x, tier_rear_left.y), &circle_center);
    }
    else {
        lncl(Point2d(tier_rear_left.x, tier_rear_left.y), Point2d(tier_rear_right.x, tier_rear_right.y), Point2d(vehicle_info_.wheel_base, 0), r, Point2d(tier_rear_right.x, tier_rear_right.y), &circle_center);
    }

    //create vehicle vertex list
    auto predictPoint2d = [](const Point2d point, const Point2d circle_center, const double delta_theta)
	{
        const double r = std::sqrt(std::pow(circle_center.x-point.x, 2.0) + std::pow(circle_center.y-point.y, 2.0));
        double theta0 = std::atan2(point.y-circle_center.y, point.x-circle_center.x);
        double theta = theta0 + delta_theta;
        const double x = r*std::cos(theta) - r*std::cos(theta0) + point.x;
        const double y = r*std::sin(theta) - r*std::sin(theta0) + point.y;
        return Point2d(x, y);
	};

    Point2d vehicle_front_left( safety_rect_.front, safety_rect_.left);
    Point2d vehicle_rear_left(  safety_rect_.rear,  safety_rect_.left);
    Point2d vehicle_rear_right( safety_rect_.rear,  safety_rect_.right);
    Point2d vehicle_front_right(safety_rect_.front, safety_rect_.right);

    std::vector< std::array<Point2d, 4> > vehicle_vertices_array;
    for(double t = 0; t < sim_time_; t+=sim_time_delta_) {
        std::array<Point2d, 4> vehicle_vertices;
        double delta_theta = v*t/r;
        vehicle_vertices.at(0) = predictPoint2d(vehicle_front_left, circle_center, delta_theta);
        vehicle_vertices.at(1) = predictPoint2d(vehicle_rear_left, circle_center, delta_theta);
        vehicle_vertices.at(2) = predictPoint2d(vehicle_rear_right, circle_center, delta_theta);
        vehicle_vertices.at(3) = predictPoint2d(vehicle_front_right, circle_center, delta_theta);
        vehicle_vertices_array.push_back(vehicle_vertices);
    }

    //create intersection list
    std::vector< std::array<Point2d, 2> > intersection_points_array;
    for(size_t i = 0; i < vehicle_vertices_array.size()-1; ++i) {
        for(size_t j = i+1; j < vehicle_vertices_array.size(); ++j) {
            std::array<Point2d, 2> intersection_points;
            intersection_points.at(0) = computeIntersectionPoint(vehicle_vertices_array.at(i).at(0), vehicle_vertices_array.at(i).at(1),
                                                                 vehicle_vertices_array.at(j).at(0), vehicle_vertices_array.at(j).at(1));
            intersection_points.at(1) = computeIntersectionPoint(vehicle_vertices_array.at(i).at(2), vehicle_vertices_array.at(i).at(3),
                                                                 vehicle_vertices_array.at(j).at(2), vehicle_vertices_array.at(j).at(3));
            intersection_points_array.push_back(intersection_points);
        }
    }

    //Judg inside vertex
    std::vector< std::array<Point2d, 4> > valid_vehicle_vertices_array;
    for(size_t i = 0; i < vehicle_vertices_array.size(); ++i) {
        std::array<Point2d, 4> valid_vehicle_vertices;

        for(size_t j = 0; j < vehicle_vertices_array.at(i).size(); ++j) {
            bool is_in = false;

            for(size_t k = 0; k < vehicle_vertices_array.size(); ++k) {
                if(i == k) {
                    continue;
                }
                is_in = isInPolygon(vehicle_vertices_array.at(i).at(j), vehicle_vertices_array.at(k));
                if(is_in) {
                    break;
                }
            }

            valid_vehicle_vertices.at(j) = is_in ? Point2d() : vehicle_vertices_array.at(i).at(j);
        }
        valid_vehicle_vertices_array.push_back(valid_vehicle_vertices);
    }

    //Judg inside intersection
    auto searchPair = [](const size_t index, const size_t n)
	{
        size_t a = index;
        for(size_t i = 0; i < n; ++i) {
            if(a < n-i) {
                return std::make_pair(i, a+i+1);
            }
            a -= n-i;
        }
	};

    std::vector< std::array<Point2d, 2> > valid_intersections_array;
    for(size_t i = 0; i < intersection_points_array.size(); ++i) {
        std::array<Point2d, 2> valid_intersections;
        auto pair = searchPair(i, vehicle_vertices_array.size()-1);

        for(size_t j = 0; j < intersection_points_array.at(i).size(); ++j) {
            bool is_in = false;

            for(size_t k = 0; k < vehicle_vertices_array.size(); ++k) {
                if(pair.first == k || pair.second == k) {
                    continue;
                }

                is_in = isInPolygon(intersection_points_array.at(i).at(j), vehicle_vertices_array.at(k));
                if(is_in) {
                    break;
                }
            }

            valid_intersections.at(j) = is_in ? Point2d() : intersection_points_array.at(i).at(j);
        }
        valid_intersections_array.push_back(valid_intersections);
    }


    //concat vertices list and intersection list
    std::array< std::vector< Point2d > ,2> concat_array;
    for(size_t i = 0; i < valid_vehicle_vertices_array.size(); ++i) {
        concat_array.at(0).push_back(valid_vehicle_vertices_array.at(i).at(0));
        concat_array.at(0).push_back(valid_vehicle_vertices_array.at(i).at(1));
        concat_array.at(1).push_back(valid_vehicle_vertices_array.at(i).at(2));
        concat_array.at(1).push_back(valid_vehicle_vertices_array.at(i).at(3));
    }
    for(size_t i = 0; i < valid_intersections_array.size(); ++i) {
        concat_array.at(0).push_back(valid_intersections_array.at(i).at(0));
        concat_array.at(1).push_back(valid_intersections_array.at(i).at(1));
    }

    //sort list
    const Point2d point01 = valid_vehicle_vertices_array.at(0).at(1);
    std::sort(std::begin(concat_array.at(0)), std::end(concat_array.at(0)),
        [&point01](const Point2d& lhs_point, const Point2d& rhs_point)
        {
            const double lhs_square_dis = std::pow(lhs_point.x-point01.x, 2.0) + std::pow(lhs_point.y-point01.y, 2.0);
            const double rhs_square_dis = std::pow(rhs_point.x-point01.x, 2.0) + std::pow(rhs_point.y-point01.y, 2.0);
            return lhs_square_dis < rhs_square_dis;
        }
    );

    const Point2d point02 = valid_vehicle_vertices_array.at(0).at(2);
    std::sort(std::begin(concat_array.at(1)), std::end(concat_array.at(1)),
        [&point02](const Point2d& lhs_point, const Point2d& rhs_point)
        {
            const double lhs_square_dis = std::pow(lhs_point.x-point02.x, 2.0) + std::pow(lhs_point.y-point02.y, 2.0);
            const double rhs_square_dis = std::pow(rhs_point.x-point02.x, 2.0) + std::pow(rhs_point.y-point02.y, 2.0);
            return lhs_square_dis > rhs_square_dis;
        }
    );

    //merge
    std::vector<Point2d> polygon;
    for(const auto& point : concat_array.at(0)) {
        if(point.x != 0 && point.y != 0) {
            polygon.push_back(point);
        }
    }
    for(const auto& point : concat_array.at(1)) {
        if(point.x != 0 && point.y != 0) {
            polygon.push_back(point);
        }
    }

    //visualization
    visualization_msgs::MarkerArray marker_array_msg;

    auto vehicle_trajectory_marker = createVehicleTrajectoryMarker("vehicle_trajectory", vehicle_vertices_array);
    marker_array_msg.markers.push_back(vehicle_trajectory_marker);

    auto vehicle_vertices_marker = createPointsMarker("vehicle_vertices", valid_vehicle_vertices_array);
    marker_array_msg.markers.push_back(vehicle_vertices_marker);

    auto intersections_marker = createPointsMarker("intersections", valid_intersections_array);
    marker_array_msg.markers.push_back(intersections_marker);

    auto vehicle_trajectory_polygon_marker = createVehicleTrajectoryPolygonMarker("vehicle_trajectory_polygon", polygon, safety_rect_.bottom, safety_rect_.top);
    marker_array_msg.markers.push_back(vehicle_trajectory_polygon_marker);

    auto rotate_center_marker = createSphereMarker("rotate_center", circle_center);
    marker_array_msg.markers.push_back(rotate_center_marker);

    auto vehicle_polygon_msg = createRectPolygon("vehicle_polygon", vehicle_rect_);
    marker_array_msg.markers.push_back(vehicle_polygon_msg);

    auto stop_area_polygon_msg = createRectPolygon("safety_polygon", safety_rect_);
    marker_array_msg.markers.push_back(stop_area_polygon_msg);

    std::vector<Point2d> tiers;
    tiers.push_back(tier_rear_left);
    tiers.push_back(tier_rear_right);
    tiers.push_back(tier_front_left);
    tiers.push_back(tier_front_right);
    auto tier_marker_array_msg = createTireMakerArrayMsg(tiers, 0.4318*2.0, 0.215, circle_center, ros::Time::now());
    for(const auto& tier_marker : tier_marker_array_msg.markers) {
        marker_array_msg.markers.push_back(tier_marker);
    }

    marker_array_pub_.publish(marker_array_msg);


    return polygon;
}

visualization_msgs::Marker StopArea::createVehicleTrajectoryMarker(const std::string& marker_ns, const std::vector< std::array<Point2d, 4> >& vehicle_vertices_array)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = baselink_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = marker_ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    geometry_msgs::Point point;
    for(size_t i = 0; i < vehicle_vertices_array.size(); ++i) {
        for(size_t j = 0; j < vehicle_vertices_array.at(i).size(); ++j) {
            size_t k = j==vehicle_vertices_array.at(i).size()-1 ? 0 : j+1;
            point.x = vehicle_vertices_array.at(i).at(j).x;
            point.y = vehicle_vertices_array.at(i).at(j).y;
            point.z = 0;
            marker.points.push_back(point);
            point.x = vehicle_vertices_array.at(i).at(k).x;
            point.y = vehicle_vertices_array.at(i).at(k).y;
            point.z = 0;
            marker.points.push_back(point);
        }
    }

    return marker;
}

visualization_msgs::Marker StopArea::createVehicleTrajectoryPolygonMarker(const std::string& marker_ns, const std::vector<Point2d>& point_array, const double bottom, const double top)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = baselink_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = marker_ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    geometry_msgs::Point point;
    for(size_t i = 0; i < point_array.size(); ++i) {
        size_t j = i==point_array.size()-1 ? 0 : i+1;

        point.x = point_array.at(i).x;
        point.y = point_array.at(i).y;
        point.z = bottom;
        marker.points.push_back(point);
        point.x = point_array.at(j).x;
        point.y = point_array.at(j).y;
        point.z = bottom;
        marker.points.push_back(point);

        point.x = point_array.at(i).x;
        point.y = point_array.at(i).y;
        point.z = top;
        marker.points.push_back(point);
        point.x = point_array.at(j).x;
        point.y = point_array.at(j).y;
        point.z = top;
        marker.points.push_back(point);

        point.x = point_array.at(i).x;
        point.y = point_array.at(i).y;
        point.z = bottom;
        marker.points.push_back(point);
        point.x = point_array.at(i).x;
        point.y = point_array.at(i).y;
        point.z = top;
        marker.points.push_back(point);
    }

    return marker;
}

template< class PointArrayT >
visualization_msgs::Marker StopArea::createPointsMarker(const std::string& marker_ns, const std::vector< PointArrayT >& points_array)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = baselink_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = marker_ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    geometry_msgs::Point point;
    for(size_t i = 0; i < points_array.size(); ++i) {
        for(size_t j = 0; j < points_array.at(i).size(); ++j) {
            point.x = points_array.at(i).at(j).x;
            point.y = points_array.at(i).at(j).y;
            point.z = 0;
            if(point.x != 0 && point.y != 0) {
                marker.points.push_back(point);
            }
        }
    }
    return marker;
}

visualization_msgs::Marker StopArea::createRectPolygon(const std::string& marker_ns, const Rectangular& rect)
{
    auto createPoint = [](const double x, const double y, const double z)
    {
        geometry_msgs::Point tmp_p;
        tmp_p.x = x;
        tmp_p.y = y;
        tmp_p.z = z;
        return tmp_p;
    };

    visualization_msgs::Marker marker;
    marker.header.frame_id = baselink_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = marker_ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    std::array<geometry_msgs::Point, 4> rect_bot =
    {
         createPoint(rect.front, rect.left,   rect.bottom)
        ,createPoint(rect.rear,  rect.left,   rect.bottom)
        ,createPoint(rect.rear,  rect.right,  rect.bottom)
        ,createPoint(rect.front, rect.right,  rect.bottom)
    };

    std::array<geometry_msgs::Point, 4> rect_top =
    {
         createPoint(rect.front, rect.left,   rect.top)
        ,createPoint(rect.rear,  rect.left,   rect.top)
        ,createPoint(rect.rear,  rect.right,  rect.top)
        ,createPoint(rect.front, rect.right,  rect.top)
    };

    for(size_t i = 0; i < rect_bot.size(); ++i) {
        size_t j = i==rect_bot.size()-1 ? 0 : i+1;
        marker.points.push_back(rect_bot.at(i));
        marker.points.push_back(rect_bot.at(j));
    }

    for(size_t i = 0; i < rect_top.size(); ++i) {
        size_t j = i==rect_top.size()-1 ? 0 : i+1;
        marker.points.push_back(rect_top.at(i));
        marker.points.push_back(rect_top.at(j));
    }

    for(size_t i = 0; i < rect_bot.size(); ++i) {
        marker.points.push_back(rect_bot.at(i));
        marker.points.push_back(rect_top.at(i));
    }

    return marker;
}

visualization_msgs::Marker StopArea::createSphereMarker(const std::string& marker_ns, const Point2d& circle_center)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = baselink_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = marker_ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = circle_center.x;
    marker.pose.position.y = circle_center.y;
    marker.pose.position.z = 0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.8;
    marker.color.r = 1.0;
    marker.color.g = 0.3;
    marker.color.b = 1.0;

    return marker;
}

visualization_msgs::MarkerArray StopArea::createTireMakerArrayMsg(const std::vector<Point2d> tiers, const double radius, const double width, const Point2d circle_center, const ros::Time& ros_time)
{
    int id = 0;
    visualization_msgs::MarkerArray tier_marker_array;

    static double rad = 0.0;

    double rad_delta = current_twist_msg_.twist.linear.x * 0.1 / radius;

    rad = circle_center.y>0 ? rad-rad_delta : rad+rad_delta;

    for(const auto& tier : tiers) {
        geometry_msgs::Quaternion orientation_msg;
        quaternionTFToMsg(tf::createQuaternionFromRPY(M_PI_2, rad, std::atan2(circle_center.y-tier.y, circle_center.x-tier.x)+M_PI_2), orientation_msg);

        visualization_msgs::Marker tier_marker;
        tier_marker.header.frame_id = baselink_frame_;
        tier_marker.header.stamp = ros_time;
        tier_marker.ns = "tier";
        tier_marker.id = id++;
        tier_marker.type = visualization_msgs::Marker::CYLINDER;
        tier_marker.action = visualization_msgs::Marker::ADD;
        tier_marker.pose.position.x = tier.x;
        tier_marker.pose.position.y = tier.y;
        tier_marker.pose.position.z = radius/2.0;
        tier_marker.pose.orientation = orientation_msg;
        tier_marker.scale.x = radius;
        tier_marker.scale.y = radius;
        tier_marker.scale.z = width;
        tier_marker.color.a = 0.5;
        tier_marker.color.r = 0.8;
        tier_marker.color.g = 0.8;
        tier_marker.color.b = 0.8;

        tier_marker_array.markers.push_back(tier_marker);
    }

    return tier_marker_array;
}
