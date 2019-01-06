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

//headers in this Autoware
#include <white_line_estimator/image_projector.h>

//headers in boost
#include <boost/shared_ptr.hpp>

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
    cv::Mat proj_matrix_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;
    cv::Size image_size_;
    boost::shared_ptr<ImageProjector> image_projector_ptr_;
};
#endif  //WHITE_LINE_ESTIMATOR_HINCLUDED