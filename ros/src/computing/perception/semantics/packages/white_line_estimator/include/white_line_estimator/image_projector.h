#ifndef IMAGE_PROJECTOR_H_INCLUDED
#define IMAGE_PROJECTOR_H_INCLUDED

//headers in ROS
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers in OpenCV
#include <opencv2/imgproc/imgproc.hpp>

//headers in Boost
#include <boost/optional.hpp>

//headers in Autoware
#include <white_line_estimator/cv_camera_info.h>

struct ProjectedPoint
{
    geometry_msgs::PointStamped point_3d;
    cv::Point point_2d;
};

class ImageProjector
{
public:
    ImageProjector(double min_area,double max_area);
    ~ImageProjector();
    void setCameraInfo(sensor_msgs::CameraInfo info);
    boost::optional<std::vector<ProjectedPoint> > project(const sensor_msgs::ImageConstPtr& image,const sensor_msgs::PointCloud2ConstPtr& pointcloud);
private:
    std::vector<std::vector<cv::Point> > getWhiteLineContours(cv::Mat image, cv::Mat &mask);
    boost::optional<std::vector<ProjectedPoint> > projectPointCloudToImage(sensor_msgs::PointCloud2 point_cloud,std::string camera_frame,cv::Size size);
    boost::optional<cv::Point> projectPoint3dPointTo2d(geometry_msgs::PointStamped point_3d,cv::Size image_size);
    std::vector<cv::Point> getGroundCovexHull(std::vector<ProjectedPoint> ground_points_in_image);
    cv::Mat getGroundMaskImage(std::vector<ProjectedPoint> ground_points_in_image,cv::Size size);
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    double min_area_;
    double max_area_;
    boost::optional<CvCameraInfo> camera_info_;
};
#endif  //IMAGE_PROJECTOR_H_INCLUDED