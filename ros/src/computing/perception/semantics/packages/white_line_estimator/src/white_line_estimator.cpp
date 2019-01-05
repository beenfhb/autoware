//headers in this package
#include <white_line_estimator/white_line_estimator.h>

WhiteLineEstimator::WhiteLineEstimator(ros::NodeHandle nh,ros::NodeHandle pnh) : it_(nh)
{
    nh_ = nh;
    pnh_ = pnh;
    image_pub_ = it_.advertise("/detected_image", 10);
    image_sub_ = it_.subscribe("/image_raw", 10, &WhiteLineEstimator::imageCallback, this);
}

WhiteLineEstimator::~WhiteLineEstimator()
{

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
        /*
        for(int i=0; i<contours_subset.size(); i++)
        {
            cv::Point2f com;
            cv::Moments mu = moments(contours_subset[i]);
            com = cv::Point2f(mu.m10/mu.m00,mu.m01/mu.m00);
            cv::circle(mask,com,3,cv::Scalar(0,255,0),-1,CV_AA);
        }
        */
        cv_ptr_pub->image = mask;
        //image.copyTo(cv_ptr_pub->image,mask);
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("opencv exception: %s", e.what());
        return;
    }
    image_pub_.publish(cv_ptr_pub->toImageMsg());
    return;
}