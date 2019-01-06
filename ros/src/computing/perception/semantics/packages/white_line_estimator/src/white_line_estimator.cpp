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
    image_size_.height = msg->height;
    image_size_.width = msg->width;
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            camera_matrix_.at<double>(row, col) = msg->K[row * 3 + col];
        }
    }
    dist_coeff_ = cv::Mat(1, 5, CV_64F);
    for (int col = 0; col < 5; col++)
    {
        dist_coeff_.at<double>(col) = msg->D[col];
    }
    return;
}

void WhiteLineEstimator::projectionMatrixCallback(const autoware_msgs::ProjectionMatrixConstPtr& msg)
{
    proj_matrix_ = cv::Mat(4, 4, CV_64F);
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            proj_matrix_.at<double>(row, col) = msg->projection_matrix[row * 4 + col];
        }
    }
    return;
}

void WhiteLineEstimator::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    return;
}