//headers in this package
#include <white_line_detector/white_line_detector.h>

WhiteLineDetector::WhiteLineDetector(ros::NodeHandle nh,ros::NodeHandle pnh) : it_(nh)
{
    nh_ = nh;
    pnh_ = pnh;
}

WhiteLineDetector::~WhiteLineDetector()
{

}

void WhiteLineDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    return;
}