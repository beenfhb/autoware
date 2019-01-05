//headers in this package
#include <white_line_estimator/white_line_estimator.h>

WhiteLineEstimator::WhiteLineEstimator(ros::NodeHandle nh,ros::NodeHandle pnh) : it_(nh)
{
    nh_ = nh;
    pnh_ = pnh;
    image_pub_ = it_.advertise("/filtered_image", 10);
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
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_pub;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr_pub = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image = cv_ptr->image;
    try
    {
        cvtColor(image, image, CV_RGB2HSV);
        std::vector<cv::Mat> hsv_planes;
        cv::split(image, hsv_planes);
        image = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
        image = hsv_planes[2];
        cv::GaussianBlur(image, image, cv::Size(11, 11), 1.0, 1.0);
        cv::threshold(image, image, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        std::vector<std::vector<cv::Point> > contours_subset;
        for(int i=0; i<contours.size();i++)
        { 
            double area=contourArea(contours.at(i));
            if(area > 100 && area < 15000)
            {
                contours_subset.push_back(contours.at(i));
            }
        }
        cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
        drawContours(mask,contours_subset,-1,cv::Scalar(255),-1);
        cvtColor(mask, mask, CV_GRAY2BGR);
        cv_ptr_pub->image = mask;
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("opencv exception: %s", e.what());
        return;
    }
    image_pub_.publish(cv_ptr_pub->toImageMsg());
    return;
}