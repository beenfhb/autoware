#ifndef WHITE_LINE_DETECTOR_HINCLUDED
#define WHITE_LINE_DETECTOR_HINCLUDED

//headers in ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

class WhiteLineDetector
{
public:
    WhiteLineDetector(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~WhiteLineDetector();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};
#endif  //WHITE_LINE_DETECTOR_HINCLUDED