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

 #ifndef STOP_AREA_CORE_H
 #define STOP_AREA_CORE_H

#include <ros/ros.h>

#include <tf/tf.h>

// #include <tf2/transform_datatypes.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>

//#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_config_msgs/ConfigStopArea.h>

#include "stop_area_util.h"

class StopArea
{
    enum class GearShift{
        Drive = 16,
        Neutral = 32,
        Reverse = 64,
        Parking = 128,
   };

   enum class EmergencyFlag{
       OK   = 0,
       Stop = 1,
   };

   struct VehicleInfo{
       VehicleInfo():length(0),width(0),height(0),wheel_base(0),tread_front(0),tread_rear(0),center_to_base(0){};
       double length;
       double width;
       double height;
       double wheel_base;
       double tread_front;
       double tread_rear;
       double center_to_base;
   };

   struct Rectangular{
       Rectangular():front(0),rear(0),left(0),right(0),top(0),bottom(0){};
       double front;
       double rear;
       double left;
       double right;
       double top;
       double bottom;
   };

public:
	StopArea(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
    void configCallback(const autoware_config_msgs::ConfigStopArea::ConstPtr& config_msg_ptr);
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg_ptr);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_in_cloud_msg_ptr);
    void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& vehicle_cmd_msg_ptr);
    void vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status_msg_ptr);

    template <class PointType>
    boost::shared_ptr< pcl::PointCloud<PointType> > extractPointsInRect(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr, const Rectangular& rect);
    template <class PointType>
    boost::shared_ptr< pcl::PointCloud<PointType> > removePointsInRect(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr, const Rectangular& rect);
    template <class PointType>
    boost::shared_ptr< pcl::PointCloud<PointType> > removeOutlierPoints(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr);
    template <class PointType>
    boost::shared_ptr< pcl::PointCloud<PointType> > insidePointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> > & pointcloud, const std::vector<Point2d>& polygon);

    std::vector<Point2d> createVehicleTrajectoryPolygon();

    visualization_msgs::Marker createVehicleTrajectoryPolygonMarker(const std::string& marker_ns, const std::vector<Point2d>& point_array, const double bottom, const double top);
    visualization_msgs::Marker createVehicleTrajectoryMarker(const std::string& marker_ns, const std::vector< std::array<Point2d, 4> >& vehicle_corners_array);
    visualization_msgs::Marker createRectPolygon(const std::string& marker_ns, const Rectangular& rect);
    template< class PointArrayT >
    visualization_msgs::Marker createPointsMarker(const std::string& marker_ns, const std::vector< PointArrayT >& points_array);
    visualization_msgs::Marker createSphereMarker(const std::string& marker_ns, const Point2d& circle_center);
    visualization_msgs::MarkerArray createTireMakerArrayMsg(const std::vector<Point2d> tiers, const double radius, const double width, const Point2d circle_center, const ros::Time& ros_time);


    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;

	   ros::Subscriber config_sub_;
    ros::Subscriber points_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber vehicle_cmd_sub_;
    ros::Subscriber vehicle_status_sub_;

    ros::Publisher  emergency_flag_pub_;
    ros::Publisher  diag_pub_;
    ros::Publisher 	points_pub_;
    ros::Publisher  marker_array_pub_;
    //diagnostic_updater::Updater diag_updater_;

    std::string baselink_frame_;
    VehicleInfo vehicle_info_;
    Rectangular vehicle_rect_;
    Rectangular safety_rect_;

    geometry_msgs::TwistStamped current_twist_msg_;
    autoware_msgs::VehicleCmd vehicle_cmd_msg_;
    autoware_msgs::VehicleStatus vehicle_status_msg_;

    double sim_time_;
    double sim_time_delta_;
    double filtering_radius_;
    int filtering_points_size_;
};

#endif
