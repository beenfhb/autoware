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

#include <string>

#include <ros/ros.h>

#include <tf/tf.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/BoundingBox.h>

//#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_config_msgs/ConfigStopArea.h>


struct Point2d{
    Point2d() : x(0), y(0){};
    Point2d(double x, double y) : x(x), y(y){};
    double x;
    double y;
};

// http://www.shibata.nu/kenji/author/cgdam.txt
int lnprm( Point2d pt1, Point2d pt2, double *a, double *b, double *c )
{
    double  xlk, ylk, rsq, rinv;
    double  accy = 1.0E-15;

    xlk = pt2.x - pt1.x;
    ylk = pt2.y - pt1.y;
    rsq = pow(xlk, 2.0) + pow(ylk, 2.0);

    if(rsq < accy){
        return (-1);
    }else{
        rinv = 1.0 / sqrt(rsq);
        *a = -ylk * rinv;
        *b = xlk * rinv;
        *c = (pt1.x * pt2.y - pt2.x * pt1.y) * rinv;
    }

    return (0);
}

// http://www.shibata.nu/kenji/author/cgdam.txt
int lncl(Point2d pt1, Point2d pt2, Point2d xyr, double r, Point2d pnear, Point2d* xy)
{
    double  root, factor, xo, yo, f, g, fsq, gsq, fgsq, xjo, yjo, a, b, c;
    double  fygx, fxgy, t, fginv, t1, t2, x1, y1, x2, y2, sqdst1, sqdst2;
    double  accy = 1.0E-15;

    if(lnprm(pt1, pt2, &a, &b, &c)) return (-1);

    root = 1.0 / (a*a + b*b);
    factor = -c * root;
    xo = a * factor;
    yo = b * factor;
    root = sqrt(root);
    f = b * root;
    g = -a * root;

    fsq = f*f;
    gsq = g*g;
    fgsq = fsq+gsq;

    if(fgsq < accy){
        return (3);
    }else{
        xjo = xyr.x - xo;
        yjo = xyr.y - yo;
        fygx = f*yjo - g*xjo;
        root = r*r*fgsq - fygx*fygx;

        if (root < -accy){
            return (-1);
        }else{
            fxgy = f*xjo + g*yjo;

            if (root < accy){
                t = fxgy / fgsq;
                xy->x = xo + f*t;
                xy->y = yo + g*t;
                return (1);
            }else{
                root = sqrt(root);
                fginv = 1.0 / fgsq;
                t1 = (fxgy - root)*fginv;
                t2 = (fxgy + root)*fginv;
                x1 = xo + f*t1;
                y1 = yo + g*t1;
                x2 = xo + f*t2;
                y2 = yo + g*t2;
            }
        }
    }

    sqdst1 = pow((pnear.x - x1), 2.0) + pow((pnear.y - y1), 2.0);
    sqdst2 = pow((pnear.x - x2), 2.0) + pow((pnear.y - y2), 2.0);

    if (sqdst1 < sqdst2){
        xy->x = x1;
        xy->y = y1;
    }else{
        xy->x = x2;
        xy->y = y2;
    }

    return (2);
}

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

    struct Area{
        Area():front(0),rear(0),left(0),right(0),top(0),bottom(0){};
        double front;
        double rear;
        double left;
        double right;
        double top;
        double bottom;
    };
public:
	StopArea();

private:
	void configCallback(const autoware_config_msgs::ConfigStopArea::ConstPtr& config_msg_ptr);
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg_ptr);
	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_in_cloud_msg_ptr);
    void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& vehicle_cmd_msg_ptr);
    void vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status_msg_ptr);

    Area createStopArea();

    template <class PointType>
    boost::shared_ptr< pcl::PointCloud<PointType> > extractPointsInArea(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr, const Area& area);
    template <class PointType>
    boost::shared_ptr< pcl::PointCloud<PointType> > removePointsInArea(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr, const Area& area);
    template <class PointType>
    boost::shared_ptr< pcl::PointCloud<PointType> > removeOutlierPoints(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr);
    template <class PointType>
    boost::shared_ptr< pcl::PointCloud<PointType> > insidePointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> >& pointcloud, const Point2d& sphere, const double sphere1_r, const double sphere2_r, const Point2d &left_rear, const Point2d &right_rear, const Point2d &left_front, const Point2d &right_front, const double min_z, const double max_z);

    jsk_recognition_msgs::BoundingBox createBoundingBox(const Area& area, const ros::Time ros_time);
    void createArcArea(Area stop_area, Area vehicle_area, ros::Time ros_time, Point2d *circle_center_ptr, double *left_r, double *right_r, Point2d *left_rear_ptr, Point2d *right_rear_ptr, Point2d *left_front_ptr, Point2d *right_front_ptr);
    geometry_msgs::PolygonStamped createStopAreaPolygon(Area stop_area, Point2d circle_center, double left_r, double right_r, Point2d left_rear, Point2d right_rear, Point2d left_front, Point2d right_front, ros::Time ros_time);
    geometry_msgs::PolygonStamped createAreaPolygon(const Area& area, const ros::Time& ros_time);

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	tf::TransformListener tf_listener_;
	// tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;

	ros::Subscriber config_sub_;
	ros::Subscriber points_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber vehicle_cmd_sub_;
    ros::Subscriber vehicle_status_sub_;
    ros::Publisher 	points_pub_;
    ros::Publisher 	vehicle_area_polygon_pub_;
    ros::Publisher 	stop_area_polygon_pub_;
    ros::Publisher 	rotate_center_point_pub_;
    ros::Publisher  emergency_flag_pub_;
    ros::Publisher  diag_pub_;
    //diagnostic_updater::Updater diag_updater_;

	std::string baselink_frame_;
    Area vehicle_area_;
    geometry_msgs::TwistStamped current_twist_msg_;
    autoware_msgs::VehicleCmd vehicle_cmd_msg_;
    autoware_msgs::VehicleStatus vehicle_status_msg_;

    double stop_area_front_min_;
    double stop_area_front_max_;
    double stop_area_front_ratio_;
    double stop_area_rear_;
    double stop_area_side_;
    double stop_area_height_min_;
    double stop_area_height_max_;

    double filtering_radius_;
    int filtering_points_size_;
};

StopArea::StopArea()
	:nh_()
	,nh_private_("~")
//	,tf_listener_(tf_buffer_)
	,baselink_frame_("/base_link")
    ,stop_area_front_min_(1.0)
    ,stop_area_front_max_(5.0)
    ,stop_area_front_ratio_(0)
    ,stop_area_rear_(0.3)
    ,stop_area_side_(0.1)
    ,stop_area_height_min_(0.5)
    ,stop_area_height_max_(2.0)
    ,filtering_radius_(0.5)
    ,filtering_points_size_(5)
{
	nh_private_.param<std::string>("baselink_frame",  baselink_frame_, baselink_frame_);

    bool use_vehicle_info_param_ = true;
	nh_private_.param("use_vehicle_info_param",  use_vehicle_info_param_, use_vehicle_info_param_);

    double vehicle_center_to_base_ = -1.0;
	double vehicle_length_ = 5.0;
	double vehicle_width_ = 2.0;
	double vehicle_height_ = 2.5;
	if(use_vehicle_info_param_) {
		nh_.param("/actuation/vehicle_info/center_to_base",  vehicle_center_to_base_, vehicle_center_to_base_);
		nh_.param("/actuation/vehicle_info/length", vehicle_length_, vehicle_length_);
		nh_.param("/actuation/vehicle_info/width", vehicle_width_, vehicle_width_);
		nh_.param("/actuation/vehicle_info/height", vehicle_height_, vehicle_height_);
	}
	else {
		nh_private_.param("vehicle_center_to_base", vehicle_center_to_base_, vehicle_center_to_base_);
		nh_private_.param("vehicle_length", vehicle_length_, vehicle_length_);
		nh_private_.param("vehicle_width", vehicle_width_, vehicle_width_);
		nh_private_.param("vehicle_height", vehicle_height_, vehicle_height_);
	}

    vehicle_area_.front  =  vehicle_length_ / 2.0 - vehicle_center_to_base_;
	vehicle_area_.rear   = -vehicle_length_ / 2.0 - vehicle_center_to_base_;
	vehicle_area_.left   =  vehicle_width_  / 2.0;
	vehicle_area_.right  = -vehicle_width_  / 2.0;
	vehicle_area_.top    =  vehicle_height_;
	vehicle_area_.bottom = 0;

    nh_private_.param("stop_area_front_min", stop_area_front_min_, stop_area_front_min_);
    nh_private_.param("stop_area_front_max", stop_area_front_max_, stop_area_front_max_);
    nh_private_.param("stop_area_front_ratio", stop_area_front_ratio_, stop_area_front_ratio_);
    nh_private_.param("stop_area_rear", stop_area_rear_, stop_area_rear_);
    nh_private_.param("stop_area_side", stop_area_side_, stop_area_side_);
    nh_private_.param("stop_area_height_min", stop_area_height_min_, stop_area_height_min_);
    nh_private_.param("stop_area_height_max", stop_area_height_max_, stop_area_height_max_);

    nh_private_.param("filtering_radius", filtering_radius_, filtering_radius_);
    nh_private_.param("filtering_points_size", filtering_points_size_, filtering_points_size_);

	config_sub_ = nh_.subscribe("/config/stop_area", 1, &StopArea::configCallback, this);
	points_sub_ = nh_.subscribe("/points_raw", 1, &StopArea::pointCloudCallback, this);
    twist_sub_ = nh_.subscribe("/current_velocity", 1, &StopArea::twistCallback, this);
    vehicle_cmd_sub_ = nh_.subscribe("/vehicle_cmd", 1, &StopArea::vehicleCmdCallback, this);
    vehicle_status_sub_ = nh_.subscribe("/vehicle_status", 1, &StopArea::vehicleStatusCallback, this);
    points_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("points_obstacle", 10);
    vehicle_area_polygon_pub_ = nh_private_.advertise<geometry_msgs::PolygonStamped>("vehicle_area_polygon", 10);
    stop_area_polygon_pub_ = nh_private_.advertise<geometry_msgs::PolygonStamped>("stop_area_polygon", 10);
    rotate_center_point_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>("rotate_center_point", 10);
    emergency_flag_pub_ = nh_private_.advertise<std_msgs::Int32>("emergency_flag", 10);
    diag_pub_ = nh_private_.advertise<diagnostic_msgs::DiagnosticArray>("diag", 10);
//    diag_updater_.setHardwareID("EmergencyStop");
//    diag_updater_.add("StopArea", &StopArea::diag, this);
}

void StopArea::configCallback(const autoware_config_msgs::ConfigStopArea::ConstPtr& config_msg_ptr)
{
  bool use_vehicle_info_param_ = config_msg_ptr->use_vehicle_info_param;
  double vehicle_center_to_base_ = -1.0;
  double vehicle_length_ = 5.0;
  double vehicle_width_ = 2.0;
  double vehicle_height_ = 2.5;
  if(use_vehicle_info_param_) {
	  nh_.param("/actuation/vehicle_info/center_to_base",  vehicle_center_to_base_, vehicle_center_to_base_);
	  nh_.param("/actuation/vehicle_info/length", vehicle_length_, vehicle_length_);
	  nh_.param("/actuation/vehicle_info/width", vehicle_width_, vehicle_width_);
	  nh_.param("/actuation/vehicle_info/height", vehicle_height_, vehicle_height_);
  }
  else {
	  vehicle_center_to_base_ = config_msg_ptr->vehicle_center_to_base;
	  vehicle_length_ = config_msg_ptr->vehicle_length;
	  vehicle_width_ = config_msg_ptr->vehicle_width;
	  vehicle_height_ = config_msg_ptr->vehicle_height;
  }
  vehicle_area_.front  =  vehicle_length_ / 2.0 - vehicle_center_to_base_;
  vehicle_area_.rear   = -vehicle_length_ / 2.0 - vehicle_center_to_base_;
  vehicle_area_.left   =  vehicle_width_  / 2.0;
  vehicle_area_.right  = -vehicle_width_  / 2.0;
  vehicle_area_.top    =  vehicle_height_;
  vehicle_area_.bottom = 0;

  stop_area_front_min_ = config_msg_ptr->stop_area_front_min;
  stop_area_front_max_ = config_msg_ptr->stop_area_front_max;
  stop_area_front_ratio_ = config_msg_ptr->stop_area_front_ratio;
  stop_area_rear_ = config_msg_ptr->stop_area_rear;
  stop_area_side_ = config_msg_ptr->stop_area_side;
  stop_area_height_min_ = config_msg_ptr->stop_area_height_min;
  stop_area_height_max_ = config_msg_ptr->stop_area_height_max;

  filtering_radius_ = config_msg_ptr->filtering_radius;
  filtering_points_size_ = config_msg_ptr->filtering_points_size;

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
// template <class PointType>
// pcl::PointCloud<PointType>::Ptr StopArea::transformPoints(const pcl::PointCloud<PointType>::Ptr& in_cloud_ptr, const std::string& target_frame)
// {
//     sensor_msgs::PointCloud2::Ptr baselinkTF_in_cloud_msg_ptr(new sensor_msgs::PointCloud2);
//     pcl_ros::transformPointCloud(target_frame_, *sensorTF_in_cloud_msg_ptr, *baselinkTF_in_cloud_msg_ptr, tf_listener_);
// }

StopArea::Area StopArea::createStopArea()
{
    double velocity_x = std::fabs(current_twist_msg_.twist.linear.x) * std::cos(current_twist_msg_.twist.angular.z);
    double velocity_y = std::fabs(current_twist_msg_.twist.linear.x) * std::sin(current_twist_msg_.twist.angular.z);
    double stop_area_front = stop_area_front_ratio_ * velocity_x;
    double stop_area_rear = stop_area_rear_;
    if(stop_area_front > stop_area_front_max_) {
        stop_area_front = stop_area_front_max_;
    }
    else if(stop_area_front < stop_area_front_min_){
        stop_area_front = stop_area_front_min_;
    }
    if(vehicle_status_msg_.gearshift == static_cast<int>(GearShift::Reverse)) {
        std::swap(stop_area_front, stop_area_rear);
    }

    double stop_area_left  = stop_area_side_;
    double stop_area_right = stop_area_side_;

    Area area;
    area.front  =  stop_area_front  + vehicle_area_.front;
	area.rear   = -stop_area_rear   + vehicle_area_.rear;
	area.left   =  stop_area_left  + vehicle_area_.left;
	area.right  = -stop_area_right  + vehicle_area_.right;
	area.top    = stop_area_height_max_;
	area.bottom = stop_area_height_min_;

    return area;
}

template <class PointType>
boost::shared_ptr< pcl::PointCloud<PointType> > StopArea::extractPointsInArea(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr, const Area& area)
{
    boost::shared_ptr< pcl::PointCloud<PointType> > out_cloud_ptr(new pcl::PointCloud<PointType>);
    out_cloud_ptr->points.reserve( in_cloud_ptr->points.size() );
    for (const auto& point : in_cloud_ptr->points) {
		if (point.x < area.front && point.x > area.rear  &&
		    point.y < area.left  && point.y > area.right &&
			point.z < area.top   && point.z > area.bottom  )
		{
			out_cloud_ptr->points.push_back(point);
		}
	}
    return out_cloud_ptr;
}

template <class PointType>
boost::shared_ptr< pcl::PointCloud<PointType> > StopArea::removePointsInArea(const boost::shared_ptr< pcl::PointCloud<PointType> >& in_cloud_ptr, const Area& area)
{
    boost::shared_ptr< pcl::PointCloud<PointType> > out_cloud_ptr(new pcl::PointCloud<PointType>);
    out_cloud_ptr->points.reserve( in_cloud_ptr->points.size() );
    for (const auto& point : in_cloud_ptr->points) {
		if (point.x > area.front || point.x < area.rear  ||
		    point.y > area.left  || point.y < area.right ||
			point.z > area.top   || point.z < area.bottom  )
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
    if(in_cloud_ptr->points.empty()){
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

    pcl::PointCloud<pcl::PointXYZI>::Ptr baselinkTF_in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    auto stop_area = createStopArea();
	pcl::fromROSMsg(*baselinkTF_in_cloud_msg_ptr, *baselinkTF_in_cloud_ptr);

    Point2d circle_center, left_rear, right_rear, left_front, right_front;
    double left_r=0, right_r=0;
    createArcArea(stop_area, vehicle_area_, sensor_time, &circle_center, &left_r, &right_r, &left_rear, &right_rear, &left_front, &right_front);
    double circle1_r = left_r > right_r ? left_r : right_r;
    double circle2_r = left_r > right_r ? right_r : left_r;

    pcl::PointCloud<pcl::PointXYZI>::Ptr baselinkTF_stop_area_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    auto baselinkTF_arc_area_cloud_ptr = insidePointCloud(baselinkTF_in_cloud_ptr, circle_center, circle1_r, circle2_r, left_rear, right_rear, left_front, right_front, stop_area.bottom, stop_area.top);
    Area around_area = stop_area;
    around_area.front = vehicle_area_.front;
    auto baselinkTF_rect_area_cloud_ptr = extractPointsInArea(baselinkTF_in_cloud_ptr, around_area);
    *baselinkTF_stop_area_cloud_ptr = *baselinkTF_arc_area_cloud_ptr;
    *baselinkTF_stop_area_cloud_ptr += *baselinkTF_rect_area_cloud_ptr;

    auto baselinkTF_stop_area_cloud_removed_footprint_ptr = removePointsInArea(baselinkTF_stop_area_cloud_ptr, vehicle_area_);
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
        stat_msg.message = "";
        emergency_flag_msg.data = static_cast<int>(EmergencyFlag::OK);
    }
    else {
        stat_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat_msg.message = "found obstacle";
        //std::cout << baselinkTF_stop_area_cloud_removed_footprint_outlier_ptr->points.size() << std::endl;
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


    auto vehicle_polygon_msg = createAreaPolygon(vehicle_area_, sensor_time);
    vehicle_area_polygon_pub_.publish(vehicle_polygon_msg);

    auto stop_area_polygon_msg = createStopAreaPolygon(stop_area, circle_center, left_r, right_r, left_rear, right_rear, left_front, right_front, sensor_time);
    stop_area_polygon_pub_.publish(stop_area_polygon_msg);


    geometry_msgs::PointStamped rotate_center_point_msg;
    rotate_center_point_msg.header.frame_id = baselink_frame_;
    rotate_center_point_msg.header.stamp = sensor_time;
    rotate_center_point_msg.point.x = circle_center.x;
    rotate_center_point_msg.point.y = circle_center.y;
    rotate_center_point_msg.point.z = 0;
    rotate_center_point_pub_.publish(rotate_center_point_msg);

}

// void StopArea::diag(diagnostic_updater::DianosticStatusWrapper &stat)
// {
//
// }

jsk_recognition_msgs::BoundingBox StopArea::createBoundingBox(const Area& area, const ros::Time ros_time)
{
    jsk_recognition_msgs::BoundingBox bounding_box_msg;
    bounding_box_msg.header.frame_id = baselink_frame_;
    bounding_box_msg.header.stamp = ros_time;
    bounding_box_msg.pose.position.x = (area.front+area.rear)/2.0;
    bounding_box_msg.pose.position.y = (area.left+area.right)/2.0;
    bounding_box_msg.pose.position.z = (area.top+area.bottom)/2.0;
    bounding_box_msg.pose.orientation.x = 0;
    bounding_box_msg.pose.orientation.y = 0;
    bounding_box_msg.pose.orientation.z = 0;
    bounding_box_msg.pose.orientation.w = 1.0;
    bounding_box_msg.dimensions.x = area.front-area.rear;
    bounding_box_msg.dimensions.y = area.left-area.right;
    bounding_box_msg.dimensions.z = area.top-area.bottom;
    // bounding_box_msg.value = 0.0;
    // bounding_box_msg.label = 0;
	return bounding_box_msg;
}

bool collidePointSphere(const double px, const double py, const double sx, const double sy, const double sr)
{
    return (px-sx)*(px-sx)+(py-sy)*(py-sy) < sr*sr;
}

bool isPointLeftFromLine(const double px, const double py, const double l1x, const double l1y, const double l2x, const double l2y)
{
    return (l2x-l1x)*(py-l1y)-(l2y-l1y)*(px-l1x) > 0;
}

template <class PointType>
boost::shared_ptr< pcl::PointCloud<PointType> > StopArea::insidePointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> > & pointcloud, const Point2d& sphere, const double sphere1_r, const double sphere2_r, const Point2d &left_rear, const Point2d &right_rear, const Point2d &left_front, const Point2d &right_front, const double min_z, const double max_z)
{
    boost::shared_ptr< pcl::PointCloud<PointType> > input_pointcloud(new pcl::PointCloud<PointType>);
    boost::shared_ptr< pcl::PointCloud<PointType> > output_pointcloud(new pcl::PointCloud<PointType>);

    *input_pointcloud = *pointcloud;
    for(const auto& point : input_pointcloud->points){
        bool is_inside = collidePointSphere(point.x, point.y, sphere.x, sphere.y, sphere1_r);
        if(is_inside){
            output_pointcloud->points.push_back(point);
        }
    }

    *input_pointcloud = *output_pointcloud;
    output_pointcloud->points.clear();
    for(const auto& point : input_pointcloud->points){
        bool is_inside = collidePointSphere(point.x, point.y, sphere.x, sphere.y, sphere2_r);
        if(!is_inside){
            output_pointcloud->points.push_back(point);
        }
    }

    *input_pointcloud = *output_pointcloud;
    output_pointcloud->points.clear();
    for(const auto& point : input_pointcloud->points){
        bool is_leftside = isPointLeftFromLine(point.x, point.y, left_rear.x, left_rear.y, right_rear.x, right_rear.y);
        if(is_leftside){
            output_pointcloud->points.push_back(point);
        }
    }

    *input_pointcloud = *output_pointcloud;
    output_pointcloud->points.clear();
    for(const auto& point : input_pointcloud->points){
        bool is_leftside = isPointLeftFromLine(point.x, point.y, left_front.x, left_front.y, right_front.x, right_front.y);
        if(!is_leftside){
            output_pointcloud->points.push_back(point);
        }
    }

    *input_pointcloud = *output_pointcloud;
    output_pointcloud->points.clear();
    for(const auto& point : input_pointcloud->points){
        if(point.z > min_z && point.z < max_z && point.x > 0) {
            output_pointcloud->points.push_back(point);
        }
    }
    return output_pointcloud;
}

void StopArea::createArcArea(Area stop_area, Area vehicle_area, ros::Time ros_time, Point2d *circle_center_ptr, double *left_r, double *right_r, Point2d *left_rear_ptr, Point2d *right_rear_ptr, Point2d *left_front_ptr, Point2d *right_front_ptr)
{
    double d = stop_area.front-vehicle_area.front;
    double r = current_twist_msg_.twist.angular.z != 0 ? std::fabs(current_twist_msg_.twist.linear.x/current_twist_msg_.twist.angular.z) : 10000;
    if(current_twist_msg_.twist.angular.z >= 0) {
        lncl(Point2d(0, stop_area.left), Point2d(0, stop_area.right), Point2d(0, stop_area.right), r, Point2d(0, stop_area.left), circle_center_ptr);
    }
    else {
        lncl(Point2d(0, stop_area.left), Point2d(0, stop_area.right), Point2d(0, stop_area.left), r, Point2d(0, stop_area.right), circle_center_ptr);
    }

    double left_vehcicle_x  = circle_center_ptr->y > 0 ? stop_area.rear  : vehicle_area.front;
    double right_vehcicle_x = circle_center_ptr->y > 0 ? vehicle_area.front : stop_area.rear;
    *left_r  = std::sqrt(std::pow(left_vehcicle_x-circle_center_ptr->x, 2.0)+std::pow(stop_area.left-circle_center_ptr->y, 2.0));
    *right_r = std::sqrt(std::pow(right_vehcicle_x-circle_center_ptr->x, 2.0)+std::pow(stop_area.right-circle_center_ptr->y, 2.0));;

    if(circle_center_ptr->y > 0) {
        Point2d tmp_p;
        lncl(Point2d(stop_area.rear, stop_area.left), Point2d(stop_area.front, stop_area.left), Point2d(circle_center_ptr->x, circle_center_ptr->y), *left_r, Point2d(stop_area.front, stop_area.left), &tmp_p);
        left_rear_ptr->x = tmp_p.x;
        left_rear_ptr->y = tmp_p.y;
        right_rear_ptr->x = right_vehcicle_x;
        right_rear_ptr->y = stop_area.right;
    }
    else {
        Point2d tmp_p;
        lncl(Point2d(stop_area.rear, stop_area.right), Point2d(stop_area.front, stop_area.right), Point2d(circle_center_ptr->x, circle_center_ptr->y), *right_r, Point2d(stop_area.front, stop_area.right), &tmp_p);
        left_rear_ptr->x = left_vehcicle_x;
        left_rear_ptr->y = stop_area.left;
        right_rear_ptr->x = tmp_p.x;
        right_rear_ptr->y = tmp_p.y;
    }

    double left_theta = std::atan2(left_rear_ptr->y-circle_center_ptr->y, left_rear_ptr->x-circle_center_ptr->x);
    double right_theta = std::atan2(right_rear_ptr->y-circle_center_ptr->y, right_rear_ptr->x-circle_center_ptr->x);
    double center_r  = std::sqrt(std::pow(vehicle_area.front-circle_center_ptr->x, 2.0)+std::pow(0-circle_center_ptr->y, 2.0));
    double theta = circle_center_ptr->y > 0 ?  d/center_r : -d/center_r;
    double left_front_theta  = circle_center_ptr->y > 0 ? theta-left_theta+right_theta : theta;
    double right_front_theta = circle_center_ptr->y > 0 ? theta : theta+left_theta-right_theta;

    double left_x0 = *left_r * std::cos(left_theta);
    double left_y0 = *left_r * std::sin(left_theta);
    left_front_ptr->x = *left_r * std::cos(left_front_theta+left_theta) - left_x0 + left_rear_ptr->x;
    left_front_ptr->y = *left_r * std::sin(left_front_theta+left_theta) - left_y0 + left_rear_ptr->y;

    double right_x0 = *right_r * std::cos(right_theta);
    double right_y0 = *right_r * std::sin(right_theta);
    right_front_ptr->x = *right_r * std::cos(right_front_theta+right_theta) - right_x0  + right_rear_ptr->x;
    right_front_ptr->y = *right_r * std::sin(right_front_theta+right_theta) - right_y0  + right_rear_ptr->y;


    //std::cout << r << " " << left_r << " " << right_r << " " << left_theta*180.0/M_PI << " " << right_theta*180.0/M_PI  << " " << left_t*180.0/M_PI << " " << right_t*180.0/M_PI << " " << theta*180.0/M_PI << std::endl;
}

geometry_msgs::PolygonStamped StopArea::createStopAreaPolygon(Area stop_area, Point2d circle_center, double left_r, double right_r, Point2d left_rear, Point2d right_rear, Point2d left_front, Point2d right_front, ros::Time ros_time)
{
    auto createPoint = [](const double x, const double y, const double z)
	{
		geometry_msgs::Point32 tmp_p;
		tmp_p.x = x;
		tmp_p.y = y;
		tmp_p.z = z;
		return tmp_p;
	};

    double left_rear_theta   = std::atan2(left_rear.y   - circle_center.y, left_rear.x   - circle_center.x);
    double right_rear_theta  = std::atan2(right_rear.y  - circle_center.y, right_rear.x  - circle_center.x);
    double left_front_theta  = std::atan2(left_front.y  - circle_center.y, left_front.x  - circle_center.x);
    double right_front_theta = std::atan2(right_front.y - circle_center.y, right_front.x - circle_center.x);

    double left_x0 = left_r * std::cos(left_rear_theta);
    double left_y0 = left_r * std::sin(left_rear_theta);
    double right_x0 = right_r * std::cos(right_rear_theta);
    double right_y0 = right_r * std::sin(right_rear_theta);

    //std::cout << r << " " << left_r << " " << right_r << " " << left_theta*180.0/M_PI << " " << right_theta*180.0/M_PI  << " " << left_t*180.0/M_PI << " " << right_t*180.0/M_PI << " " << theta*180.0/M_PI << std::endl;

    geometry_msgs::PolygonStamped body_msg;
	body_msg.header.frame_id = baselink_frame_;
	body_msg.header.stamp = ros_time;
    size_t max_i = 30;

    body_msg.polygon.points.push_back(createPoint(stop_area.rear, stop_area.left, stop_area.bottom));

    for(size_t i = 0; i < max_i; ++i){
        double t = i*(left_front_theta-left_rear_theta)/(max_i-1);
        double x = left_r * std::cos(t+left_rear_theta) - left_x0 + left_rear.x;
        double y = left_r * std::sin(t+left_rear_theta) - left_y0 + left_rear.y;
        body_msg.polygon.points.push_back(createPoint(x, y, stop_area.bottom));
    }

    for(int i = (max_i-1); i >= 0; --i){
        double t = i*(right_front_theta-right_rear_theta)/(max_i-1);
        double x = right_r * std::cos(t+right_rear_theta) - right_x0  + right_rear.x;
        double y = right_r * std::sin(t+right_rear_theta) - right_y0  + right_rear.y;
        body_msg.polygon.points.push_back(createPoint(x, y, stop_area.bottom));
    }
    body_msg.polygon.points.push_back(createPoint(stop_area.rear, stop_area.right, stop_area.bottom));


    body_msg.polygon.points.push_back(createPoint(stop_area.rear, stop_area.left, stop_area.bottom));
    body_msg.polygon.points.push_back(createPoint(stop_area.rear, stop_area.left, stop_area.top));

    for(size_t i = 0; i < max_i; ++i){
        double t = i*(left_front_theta-left_rear_theta)/(max_i-1);
        double x = left_r * std::cos(t+left_rear_theta) - left_x0 + left_rear.x;
        double y = left_r * std::sin(t+left_rear_theta) - left_y0 + left_rear.y;
        body_msg.polygon.points.push_back(createPoint(x, y, stop_area.top));
    }
    body_msg.polygon.points.push_back(createPoint(left_front.x, left_front.y, stop_area.bottom));
    body_msg.polygon.points.push_back(createPoint(left_front.x, left_front.y, stop_area.top));

    body_msg.polygon.points.push_back(createPoint(right_front.x, right_front.y, stop_area.top));
    body_msg.polygon.points.push_back(createPoint(right_front.x, right_front.y, stop_area.bottom));
    for(int i = (max_i-1); i >= 0; --i){
        double t = i*(right_front_theta-right_rear_theta)/(max_i-1);
        double x = right_r * std::cos(t+right_rear_theta) - right_x0  + right_rear.x;
        double y = right_r * std::sin(t+right_rear_theta) - right_y0  + right_rear.y;
        body_msg.polygon.points.push_back(createPoint(x, y, stop_area.top));
    }

    body_msg.polygon.points.push_back(createPoint(stop_area.rear, stop_area.right, stop_area.top));
    body_msg.polygon.points.push_back(createPoint(stop_area.rear, stop_area.right, stop_area.bottom));
    body_msg.polygon.points.push_back(createPoint(stop_area.rear, stop_area.right, stop_area.top));

    body_msg.polygon.points.push_back(createPoint(stop_area.rear, stop_area.left, stop_area.top));

    return body_msg;
}

geometry_msgs::PolygonStamped StopArea::createAreaPolygon(const Area& area, const ros::Time& ros_time)
{
	auto createPoint = [](const double x, const double y, const double z)
	{
		geometry_msgs::Point32 tmp_p;
		tmp_p.x = x;
		tmp_p.y = y;
		tmp_p.z = z;
		return tmp_p;
	};

	geometry_msgs::PolygonStamped body_msg;
	body_msg.header.frame_id = baselink_frame_;
	body_msg.header.stamp = ros_time;
	body_msg.polygon.points.push_back(createPoint(area.front, area.left,  area.bottom));
	body_msg.polygon.points.push_back(createPoint(area.rear,  area.left,  area.bottom));
	body_msg.polygon.points.push_back(createPoint(area.rear,  area.right, area.bottom));
	body_msg.polygon.points.push_back(createPoint(area.front, area.right, area.bottom));
	body_msg.polygon.points.push_back(createPoint(area.front, area.left,  area.bottom));

	body_msg.polygon.points.push_back(createPoint(area.front, area.left,  area.top));

	body_msg.polygon.points.push_back(createPoint(area.rear,  area.left,  area.top));
	body_msg.polygon.points.push_back(createPoint(area.rear,  area.left,  area.bottom));
	body_msg.polygon.points.push_back(createPoint(area.rear,  area.left,  area.top));

	body_msg.polygon.points.push_back(createPoint(area.rear,  area.right, area.top));
	body_msg.polygon.points.push_back(createPoint(area.rear,  area.right, area.bottom));
	body_msg.polygon.points.push_back(createPoint(area.rear,  area.right, area.top));

	body_msg.polygon.points.push_back(createPoint(area.front, area.right, area.top));
	body_msg.polygon.points.push_back(createPoint(area.front, area.right, area.bottom));
	body_msg.polygon.points.push_back(createPoint(area.front, area.right, area.top));

	body_msg.polygon.points.push_back(createPoint(area.front, area.left,  area.top));

    return body_msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stop_area");
	StopArea node;
	ros::spin();

	return 0;
}
