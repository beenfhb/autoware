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
#include <tf/tf.h>

//headers in OpenCV
#include <opencv2/imgproc/imgproc.hpp>

//headers in Boost
#include <boost/optional.hpp>

//headers in Autoware
#include <white_line_estimator/cv_camera_info.h>

//headers in STL
#include <mutex>

struct ProjectedPoint
{
    geometry_msgs::Point point_3d;
    cv::Point point_2d;
};

class ImageProjector
{
public:
    ImageProjector(double min_area,double max_area);
    ~ImageProjector();
    void setCameraInfo(sensor_msgs::CameraInfo info);
    void setProjectionMatrix(autoware_msgs::ProjectionMatrix proj_matrix);
    boost::optional<std::vector<ProjectedPoint> > project(cv::Mat image,const sensor_msgs::PointCloud2ConstPtr& pointcloud,cv::Mat& mask_image, cv::Mat& ground_image);
private:
    std::vector<ProjectedPoint> projectPointCloudToImage(sensor_msgs::PointCloud2 point_cloud,cv::Size size);
    cv::Mat getGroundConvexHull(std::vector<ProjectedPoint> ground_points,cv::Size size);
    double min_area_;
    double max_area_;
    boost::optional<CvCameraInfo> camera_info_;
    boost::optional<CvProjMatrix> proj_matrix_;
    std::mutex mtx_;
};
#endif  //IMAGE_PROJECTOR_H_INCLUDED