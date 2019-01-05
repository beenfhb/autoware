#ifndef WHITE_LINE_ESTIMATOR_HINCLUDED
#define WHITE_LINE_ESTIMATOR_HINCLUDED

//headers in ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <autoware_msgs/ProjectionMatrix.h>

class WhiteLineEstimator
{
public:
    WhiteLineEstimator(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~WhiteLineEstimator();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void pointsGroundCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void projectionMatrixCallback(const autoware_msgs::ProjectionMatrixConstPtr& msg);
    cv::Mat proj_matrix_;
    cv::Mat camera_matrix_;
    cv::Size image_size_;
};
#endif  //WHITE_LINE_ESTIMATOR_HINCLUDED