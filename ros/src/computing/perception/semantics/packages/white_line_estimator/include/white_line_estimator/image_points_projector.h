#ifndef IMAGE_POINTS_PROJECTOR_H_INCLUDED
#define IMAGE_POINTS_PROJECTOR_H_INCLUDED

//headers in opencv
#include <opencv2/imgproc/imgproc.hpp>

//headers in Autoware
#include <white_line_estimator/cv_camera_info.h>

//headers in boost
#include <boost/optional.hpp>

//headers in STL
#include <mutex>

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//headers in Eigen
#include <Eigen/LU>

class ImagePointsProjector
{
public:
    ImagePointsProjector();
    ~ImagePointsProjector();
    boost::optional<std::vector<std::vector<geometry_msgs::Point> > > project(std::vector<std::vector<cv::Point> > image_points);
    void setCameraInfo(sensor_msgs::CameraInfo info);
    void setProjectionMatrix(autoware_msgs::ProjectionMatrix proj_matrix);
    void setMagnification(double x_axis,double y_axis,double z_offset);
private:
    double x_magnification_;
    double y_magnification_;
    double z_offset_;
    boost::optional<CvCameraInfo> camera_info_;
    boost::optional<CvProjMatrix> proj_matrix_;
    std::mutex mtx_;
};

#endif  //IMAGE_PROJECTOR_H_INCLUDED