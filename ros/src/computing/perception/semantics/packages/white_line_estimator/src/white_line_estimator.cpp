//headers in this package
#include <white_line_estimator/white_line_estimator.h>

WhiteLineEstimator::WhiteLineEstimator(ros::NodeHandle nh,ros::NodeHandle pnh) : it_(nh)
{
    nh_ = nh;
    pnh_ = pnh;
    image_projector_ptr_ = boost::make_shared<ImageProjector>(500,15000);
    image_pub_ = it_.advertise("/points_image", 10);
    image_sub_ = it_.subscribe("/image_raw", 10, &WhiteLineEstimator::imageCallback, this);
}

WhiteLineEstimator::~WhiteLineEstimator()
{

}

void WhiteLineEstimator::pointsGroundCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    return;
}

void WhiteLineEstimator::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    return;
}

void WhiteLineEstimator::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    return;
}