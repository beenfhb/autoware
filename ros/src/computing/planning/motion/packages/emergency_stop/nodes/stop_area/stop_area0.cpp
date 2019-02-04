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
#include <utility>
#include <algorithm>

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
#include <visualization_msgs/MarkerArray.h>
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
// int lncl(Point2d pt1, Point2d pt2, Point2d xyr, double r, Point2d pnear, Point2d* xy)
// {
//     double  a, b, c;
//     if(lnprm(pt1, pt2, &a, &b, &c)) return (-1);
//
//     double l = a*a+b*b;
//     double k = a*xyr.x + b*xyr.y + c;
//     double d = l*r*r - k*k;
//
//     double ds = std::sqrt(d);
//     double apl = a/l;
//     double bpl = b/l;
//     double xc = xyr.x - apl*k;
//     double yc = xyr.y - bpl*k;
//     double xd = bpl*ds;
//     double yd = apl*ds;
//
//     double x1 = xc - xd;
//     double y1 = yc + yd;
//     double x2 = xc + xd;
//     double y2 = yc - yd;
//
//     double sqdst1 = pow((pnear.x - x1), 2.0) + pow((pnear.y - y1), 2.0);
//     double sqdst2 = pow((pnear.x - x2), 2.0) + pow((pnear.y - y2), 2.0);
//
//     if (sqdst1 < sqdst2){
//         xy->x = x1;
//         xy->y = y1;
//     }else{
//         xy->x = x2;
//         xy->y = y2;
//     }
//
//     return 1;
// }

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

Point2d computeIntersectionPoint(const Point2d& pointA, const Point2d& pointB,
							     const Point2d& pointC, const Point2d& pointD)
{
	double dBunbo	= ( pointB.x - pointA.x ) * ( pointD.y - pointC.y )
					- ( pointB.y - pointA.y ) * ( pointD.x - pointC.x );

	if( 0 == dBunbo ) {
		return Point2d();
	}

	Point2d vectorAC;
    vectorAC.x = pointC.x - pointA.x;
    vectorAC.y = pointC.y - pointA.y;

	double dR = ( ( pointD.y - pointC.y ) * vectorAC.x
		 - ( pointD.x - pointC.x ) * vectorAC.y ) / dBunbo;
	double dS = ( ( pointB.y - pointA.y ) * vectorAC.x
		 - ( pointB.x - pointA.x ) * vectorAC.y ) / dBunbo;

    if(dR < 0 || dR > 1.0 || dS < 0 || dS > 1.0) {
        return Point2d();
    }

    Point2d intersection_point;
    intersection_point.x = dR * (pointB.x - pointA.x) + pointA.x;
    intersection_point.y = dR * (pointB.y - pointA.y) + pointA.y;

	return intersection_point;
}

double Calc_tan(const Point2d& point, const Point2d& con1, const Point2d& con2){
    double Ax,Ay,Bx,By;
    double AxB,AvB;
    double angle;
    Ax = con1.x - point.x;
    Ay = con1.y - point.y;
    Bx = con2.x - point.x;
    By = con2.y - point.y;
    AvB = Ax * Bx + Ay * By;
    AxB = Ax * By - Ay * Bx;

    angle = atan2(AxB,AvB);
    return(angle);
}

bool isInPolygon(const Point2d& point, const std::vector<Point2d>& polygon)
{
    double tmp=0;
    double angle=0;

    for(size_t i = 0; i < polygon.size(); i++){
        Point2d con1 = polygon.at(i);
        Point2d con2 = i == polygon.size()-1 ? polygon.at(0) : polygon.at(i+1);
        tmp = Calc_tan(point, con1, con2);
        angle += tmp;
    }
    if( std::fabs(2.0*M_PI-std::fabs(angle)) < 0.001 ) {
        return true;
    }
    else {
        return false;
    }
}

bool isInPolygon(const Point2d& point, const std::array<Point2d, 4>& polygon)
{
    std::vector<Point2d> array;
    for(const auto p : polygon) {
        array.push_back(p);
    }
    return isInPolygon(point, array);
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
	StopArea();

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
    boost::shared_ptr< pcl::PointCloud<PointType> > insidePointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> >& pointcloud, const Point2d& sphere, const double sphere1_r, const double sphere2_r, const Point2d &left_rear, const Point2d &right_rear, const Point2d &left_front, const Point2d &right_front, const double min_z, const double max_z);
    template <class PointType>
    boost::shared_ptr< pcl::PointCloud<PointType> > insidePointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> > & pointcloud, std::vector<Point2d> polygon);

    geometry_msgs::PolygonStamped createRectPolygon(const Rectangular& rect, const ros::Time& ros_time);
    visualization_msgs::MarkerArray createTireMakerArrayMsg(const std::vector<Point2d> tiers, const double radius, const double width, const Point2d circle_center, const ros::Time& ros_time);
    std::vector<Point2d> createVehicleTrajectoryPolygon();

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
    ros::Publisher  tier_marker_array_pub_;
    ros::Publisher  vehicle_marker_array_pub_;
    ros::Publisher  vehicle_points_marker_array_pub_;
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

StopArea::StopArea()
	:nh_()
	,nh_private_("~")
//	,tf_listener_(tf_buffer_)
	,baselink_frame_("/base_link")
    ,sim_time_(1.0)
    ,sim_time_delta_(0.1)
    ,filtering_radius_(0.5)
    ,filtering_points_size_(5)
{
	nh_private_.param<std::string>("baselink_frame",  baselink_frame_, baselink_frame_);

    nh_private_.param("sim_time", sim_time_, sim_time_);
    nh_private_.param("sim_time_delta", sim_time_delta_, sim_time_delta_);
    nh_private_.param("filtering_radius", filtering_radius_, filtering_radius_);
    nh_private_.param("filtering_points_size", filtering_points_size_, filtering_points_size_);

    Rectangular offset_rect;
    nh_private_.param("offset_front",  offset_rect.front,   0.1);
    nh_private_.param("offset_rear",   offset_rect.rear,    0.1);
    nh_private_.param("offset_left",   offset_rect.left,    0.1);
    nh_private_.param("offset_right",  offset_rect.right,   0.1);
    nh_private_.param("offset_top",    offset_rect.top,     0.1);
    nh_private_.param("offset_bottom", offset_rect.bottom,  -0.3);

    bool use_vehicle_info_param = true;
	nh_private_.param("use_vehicle_info_param",  use_vehicle_info_param, use_vehicle_info_param);

    vehicle_info_.length = 5.0;
    vehicle_info_.width = 2.0;
    vehicle_info_.height = 2.5;
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
        nh_private_.param("vehicle_info_length",          vehicle_info_.length,        vehicle_info_.length);
		nh_private_.param("vehicle_info_width",           vehicle_info_.width,          vehicle_info_.width);
		nh_private_.param("vehicle_info_height",          vehicle_info_.height,         vehicle_info_.height);
        nh_private_.param("vehicle_info_wheel_base",      vehicle_info_.wheel_base,     vehicle_info_.wheel_base);
        nh_private_.param("vehicle_info_tread_front",     vehicle_info_.tread_front,    vehicle_info_.tread_front);
        nh_private_.param("vehicle_info_tread_rear",      vehicle_info_.tread_rear,     vehicle_info_.tread_rear);
		nh_private_.param("vehicle_info_center_to_base",  vehicle_info_.center_to_base, vehicle_info_.center_to_base);
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

    points_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("points_obstacle", 10);
    vehicle_area_polygon_pub_ = nh_private_.advertise<geometry_msgs::PolygonStamped>("vehicle_area_polygon", 10);
    stop_area_polygon_pub_ = nh_private_.advertise<geometry_msgs::PolygonStamped>("stop_area_polygon", 10);
    rotate_center_point_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>("rotate_center_point", 10);
    emergency_flag_pub_ = nh_private_.advertise<std_msgs::Int32>("emergency_flag", 10);
    diag_pub_ = nh_private_.advertise<diagnostic_msgs::DiagnosticArray>("diag", 10);
    tier_marker_array_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("tier_marker_array", 10);
    vehicle_marker_array_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("vehicle_marker_array", 10);
    vehicle_points_marker_array_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("vehicle_points_marker_array", 10);
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


    auto vehicle_polygon_msg = createRectPolygon(vehicle_rect_, sensor_time);
    vehicle_area_polygon_pub_.publish(vehicle_polygon_msg);

    auto stop_area_polygon_msg = createRectPolygon(safety_rect_, sensor_time);
    stop_area_polygon_pub_.publish(stop_area_polygon_msg);

}

template <class PointType>
boost::shared_ptr< pcl::PointCloud<PointType> > StopArea::insidePointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> > & pointcloud, std::vector<Point2d> polygon)
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
    //回転半径算出
    const double v = current_twist_msg_.twist.linear.x;
    const double w = current_twist_msg_.twist.angular.z;
    const double r = current_twist_msg_.twist.angular.z != 0 ? current_twist_msg_.twist.linear.x/current_twist_msg_.twist.angular.z : 10000;

    //回転中心点算出
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
    // std::cout << circle_center.x << " " << circle_center.y << std::endl;

    //頂点リスト作成
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

    std::vector< std::array<Point2d, 4> > vehicle_corners_array;
    for(double t = 0; t < sim_time_; t+=sim_time_delta_) {
        std::array<Point2d, 4> vehicle_corners;
        double delta_theta = v*t/r;
        vehicle_corners.at(0) = predictPoint2d(vehicle_front_left, circle_center, delta_theta);
        vehicle_corners.at(1) = predictPoint2d(vehicle_rear_left, circle_center, delta_theta);
        vehicle_corners.at(2) = predictPoint2d(vehicle_rear_right, circle_center, delta_theta);
        vehicle_corners.at(3) = predictPoint2d(vehicle_front_right, circle_center, delta_theta);
        vehicle_corners_array.push_back(vehicle_corners);
    }

    //交点リスト作成
    std::vector< std::array<Point2d, 2> > intersection_points_array;
    for(size_t i = 0; i < vehicle_corners_array.size()-1; ++i) {
        for(size_t j = i+1; j < vehicle_corners_array.size(); ++j) {
            std::array<Point2d, 2> intersection_points;
            intersection_points.at(0) = computeIntersectionPoint(vehicle_corners_array.at(i).at(0), vehicle_corners_array.at(i).at(1),
                                                                 vehicle_corners_array.at(j).at(0), vehicle_corners_array.at(j).at(1));
            intersection_points.at(1) = computeIntersectionPoint(vehicle_corners_array.at(i).at(2), vehicle_corners_array.at(i).at(3),
                                                                 vehicle_corners_array.at(j).at(2), vehicle_corners_array.at(j).at(3));
            intersection_points_array.push_back(intersection_points);
        }
    }

    //頂点内外判定
    std::vector< std::array<Point2d, 4> > valid_vehicle_corners_array;
    for(size_t i = 0; i < vehicle_corners_array.size(); ++i) {
        std::array<Point2d, 4> valid_vehicle_corners;

        for(size_t j = 0; j < vehicle_corners_array.at(i).size(); ++j) {
            bool is_in = false;

            for(size_t k = 0; k < vehicle_corners_array.size(); ++k) {
                if(i == k) {
                    continue;
                }
                is_in = isInPolygon(vehicle_corners_array.at(i).at(j), vehicle_corners_array.at(k));
                if(is_in) {
                    break;
                }
            }

            valid_vehicle_corners.at(j) = is_in ? Point2d() : vehicle_corners_array.at(i).at(j);
        }
        valid_vehicle_corners_array.push_back(valid_vehicle_corners);
    }

    //交点内外判定
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
        auto pair = searchPair(i, vehicle_corners_array.size()-1);
        //std::cout << pair.first << " " << pair.second << std::endl;

        for(size_t j = 0; j < intersection_points_array.at(i).size(); ++j) {
            bool is_in = false;

            for(size_t k = 0; k < vehicle_corners_array.size(); ++k) {
                if(pair.first == k || pair.second == k) {
                    continue;
                }

                is_in = isInPolygon(intersection_points_array.at(i).at(j), vehicle_corners_array.at(k));
                if(is_in) {
                    break;
                }
            }

            valid_intersections.at(j) = is_in ? Point2d() : intersection_points_array.at(i).at(j);
        }
        valid_intersections_array.push_back(valid_intersections);
    }


    //頂点リストと交点リストの統合
    std::array< std::vector< Point2d > ,2> concat_array;
    for(size_t i = 0; i < valid_vehicle_corners_array.size(); ++i) {
        concat_array.at(0).push_back(valid_vehicle_corners_array.at(i).at(0));
        concat_array.at(0).push_back(valid_vehicle_corners_array.at(i).at(1));
        concat_array.at(1).push_back(valid_vehicle_corners_array.at(i).at(2));
        concat_array.at(1).push_back(valid_vehicle_corners_array.at(i).at(3));
    }
    for(size_t i = 0; i < valid_intersections_array.size(); ++i) {
        concat_array.at(0).push_back(valid_intersections_array.at(i).at(0));
        concat_array.at(1).push_back(valid_intersections_array.at(i).at(1));
    }

    //リストのソート
    const Point2d point01 = valid_vehicle_corners_array.at(0).at(1);
    std::sort(std::begin(concat_array.at(0)), std::end(concat_array.at(0)),
        [&point01](const Point2d& lhs_point, const Point2d& rhs_point)
        {
            const double lhs_square_dis = std::pow(lhs_point.x-point01.x, 2.0) + std::pow(lhs_point.y-point01.y, 2.0);
            const double rhs_square_dis = std::pow(rhs_point.x-point01.x, 2.0) + std::pow(rhs_point.y-point01.y, 2.0);
            return lhs_square_dis < rhs_square_dis;
        }
    );

    const Point2d point02 = valid_vehicle_corners_array.at(0).at(2);
    std::sort(std::begin(concat_array.at(1)), std::end(concat_array.at(1)),
        [&point02](const Point2d& lhs_point, const Point2d& rhs_point)
        {
            const double lhs_square_dis = std::pow(lhs_point.x-point02.x, 2.0) + std::pow(lhs_point.y-point02.y, 2.0);
            const double rhs_square_dis = std::pow(rhs_point.x-point02.x, 2.0) + std::pow(rhs_point.y-point02.y, 2.0);
            return lhs_square_dis > rhs_square_dis;
        }
    );

    //1つにまとめる
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

    //debug
    visualization_msgs::MarkerArray vehicle_corners_marker_array;
    for(size_t j = 0; j < vehicle_corners_array.size(); ++j) {
        visualization_msgs::Marker vehicle_corners_marker;
        vehicle_corners_marker.header.frame_id = baselink_frame_;
        vehicle_corners_marker.header.stamp = ros::Time::now();
        vehicle_corners_marker.ns = std::string("stop_area_vehicle_corner");
        vehicle_corners_marker.id = j; //TODO
        vehicle_corners_marker.type = visualization_msgs::Marker::LINE_STRIP;
        vehicle_corners_marker.action = visualization_msgs::Marker::ADD;
        vehicle_corners_marker.pose.position.x = 0;
        vehicle_corners_marker.pose.position.y = 0;
        vehicle_corners_marker.pose.position.z = 0;

        geometry_msgs::Point point;
        for(size_t i = 0; i < vehicle_corners_array.at(j).size(); ++i) {
            point.x = vehicle_corners_array.at(j).at(i).x;
            point.y = vehicle_corners_array.at(j).at(i).y;
            point.z = 0;
            vehicle_corners_marker.points.push_back(point);
        }
        point.x = vehicle_corners_array.at(j).at(0).x;
        point.y = vehicle_corners_array.at(j).at(0).y;
        point.z = 0;
        vehicle_corners_marker.points.push_back(point);

        vehicle_corners_marker.scale.x = 0.01;
        vehicle_corners_marker.scale.y = 0.01;
        vehicle_corners_marker.scale.z = 0.01;
        vehicle_corners_marker.color.a = 0.5;
        vehicle_corners_marker.color.r = 0.0;
        vehicle_corners_marker.color.g = 1.0;
        vehicle_corners_marker.color.b = 0.0;

        vehicle_corners_marker_array.markers.push_back(vehicle_corners_marker);
    }

    visualization_msgs::MarkerArray vehicle_points_marker_array;
    {
        visualization_msgs::Marker vehicle_points_marker;
        vehicle_points_marker.header.frame_id = baselink_frame_;
        vehicle_points_marker.header.stamp = ros::Time::now();
        vehicle_points_marker.ns = std::string("stop_area_vehicle_points");
        vehicle_points_marker.id = 0;
        vehicle_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        vehicle_points_marker.action = visualization_msgs::Marker::ADD;
        vehicle_points_marker.pose.position.x = 0;
        vehicle_points_marker.pose.position.y = 0;
        vehicle_points_marker.pose.position.z = 0;
        geometry_msgs::Point point;
        for(size_t i = 0; i < valid_vehicle_corners_array.size(); ++i) {
            for(size_t j = 0; j < valid_vehicle_corners_array.at(i).size(); ++j) {
                point.x = valid_vehicle_corners_array.at(i).at(j).x;
                point.y = valid_vehicle_corners_array.at(i).at(j).y;
                point.z = 0;
                if(point.x != 0 && point.y != 0) {
                    vehicle_points_marker.points.push_back(point);
                }
            }
        }
        vehicle_points_marker.scale.x = 0.1;
        vehicle_points_marker.scale.y = 0.1;
        vehicle_points_marker.scale.z = 0.1;
        vehicle_points_marker.color.a = 0.5;
        vehicle_points_marker.color.r = 0.0;
        vehicle_points_marker.color.g = 0.0;
        vehicle_points_marker.color.b = 1.0;

        vehicle_points_marker_array.markers.push_back(vehicle_points_marker);
    }
    {
        visualization_msgs::Marker vehicle_points_marker;
        vehicle_points_marker.header.frame_id = baselink_frame_;
        vehicle_points_marker.header.stamp = ros::Time::now();
        vehicle_points_marker.ns = std::string("stop_area_intersection_points");
        vehicle_points_marker.id = 0;
        vehicle_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        vehicle_points_marker.action = visualization_msgs::Marker::ADD;
        vehicle_points_marker.pose.position.x = 0;
        vehicle_points_marker.pose.position.y = 0;
        vehicle_points_marker.pose.position.z = 0;
        geometry_msgs::Point point;
        for(size_t i = 0; i < valid_intersections_array.size(); ++i) {
            for(size_t j = 0; j < valid_intersections_array.at(i).size(); ++j) {
                point.x = valid_intersections_array.at(i).at(j).x;
                point.y = valid_intersections_array.at(i).at(j).y;
                point.z = 0;
                if(point.x != 0 && point.y != 0) {
                    vehicle_points_marker.points.push_back(point);
                }
            }
        }
        vehicle_points_marker.scale.x = 0.1;
        vehicle_points_marker.scale.y = 0.1;
        vehicle_points_marker.scale.z = 0.1;
        vehicle_points_marker.color.a = 1.0;
        vehicle_points_marker.color.r = 1.0;
        vehicle_points_marker.color.g = 0.0;
        vehicle_points_marker.color.b = 0.0;

        vehicle_points_marker_array.markers.push_back(vehicle_points_marker);
    }
    {
        visualization_msgs::Marker concat_points_marker;
        concat_points_marker.header.frame_id = baselink_frame_;
        concat_points_marker.header.stamp = ros::Time::now();
        concat_points_marker.ns = std::string("stop_area_polygon");
        concat_points_marker.id = 0;
        concat_points_marker.type = visualization_msgs::Marker::LINE_STRIP;
        concat_points_marker.action = visualization_msgs::Marker::ADD;
        concat_points_marker.pose.position.x = 0;
        concat_points_marker.pose.position.y = 0;
        concat_points_marker.pose.position.z = 0;

        geometry_msgs::Point point;
        for(size_t i = 0; i < polygon.size(); ++i) {
            point.x = polygon.at(i).x;
            point.y = polygon.at(i).y;
            point.z = 0;
            concat_points_marker.points.push_back(point);
        }
        point.x = polygon.at(0).x;
        point.y = polygon.at(0).y;
        point.z = 0;
        concat_points_marker.points.push_back(point);

        concat_points_marker.scale.x = 0.01;
        concat_points_marker.scale.y = 0.01;
        concat_points_marker.scale.z = 0.01;
        concat_points_marker.color.a = 0.5;
        concat_points_marker.color.r = 1.0;
        concat_points_marker.color.g = 1.0;
        concat_points_marker.color.b = 0.0;

        vehicle_corners_marker_array.markers.push_back(concat_points_marker);
    }
    vehicle_marker_array_pub_.publish(vehicle_corners_marker_array);

    vehicle_points_marker_array_pub_.publish(vehicle_points_marker_array);

    geometry_msgs::PointStamped rotate_center_point_msg;
    rotate_center_point_msg.header.frame_id = baselink_frame_;
    rotate_center_point_msg.header.stamp = ros::Time::now();
    rotate_center_point_msg.point.x = circle_center.x;
    rotate_center_point_msg.point.y = circle_center.y;
    rotate_center_point_msg.point.z = 0;
    rotate_center_point_pub_.publish(rotate_center_point_msg);

    std::vector<Point2d> tiers;
    tiers.push_back(tier_rear_left);
    tiers.push_back(tier_rear_right);
    tiers.push_back(tier_front_left);
    tiers.push_back(tier_front_right);
    auto tier_marker_array_msg = createTireMakerArrayMsg(tiers, 0.4318*2.0, 0.215, circle_center, ros::Time::now());
    tier_marker_array_pub_.publish(tier_marker_array_msg);

    return polygon;
}

geometry_msgs::PolygonStamped StopArea::createRectPolygon(const Rectangular& rect, const ros::Time& ros_time)
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
	body_msg.polygon.points.push_back(createPoint(rect.front, rect.left,  rect.bottom));
	body_msg.polygon.points.push_back(createPoint(rect.rear,  rect.left,  rect.bottom));
	body_msg.polygon.points.push_back(createPoint(rect.rear,  rect.right, rect.bottom));
	body_msg.polygon.points.push_back(createPoint(rect.front, rect.right, rect.bottom));
	body_msg.polygon.points.push_back(createPoint(rect.front, rect.left,  rect.bottom));

	body_msg.polygon.points.push_back(createPoint(rect.front, rect.left,  rect.top));

	body_msg.polygon.points.push_back(createPoint(rect.rear,  rect.left,  rect.top));
	body_msg.polygon.points.push_back(createPoint(rect.rear,  rect.left,  rect.bottom));
	body_msg.polygon.points.push_back(createPoint(rect.rear,  rect.left,  rect.top));

	body_msg.polygon.points.push_back(createPoint(rect.rear,  rect.right, rect.top));
	body_msg.polygon.points.push_back(createPoint(rect.rear,  rect.right, rect.bottom));
	body_msg.polygon.points.push_back(createPoint(rect.rear,  rect.right, rect.top));

	body_msg.polygon.points.push_back(createPoint(rect.front, rect.right, rect.top));
	body_msg.polygon.points.push_back(createPoint(rect.front, rect.right, rect.bottom));
	body_msg.polygon.points.push_back(createPoint(rect.front, rect.right, rect.top));

	body_msg.polygon.points.push_back(createPoint(rect.front, rect.left,  rect.top));

    return body_msg;
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
        tier_marker.ns = "stop_area_tier";
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stop_area");
	StopArea node;
	ros::spin();

	return 0;
}
