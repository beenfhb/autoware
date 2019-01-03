//headers in this package
#include <white_line_detector/white_line_detector.h>

WhiteLineDetector::WhiteLineDetector(ros::NodeHandle nh,ros::NodeHandle pnh) : it_(nh)
{
    nh_ = nh;
    pnh_ = pnh;
    image_pub_ = it_.advertise("/detected_image", 10);
    image_sub_ = it_.subscribe("/image_raw", 10, &WhiteLineDetector::imageCallback, this);
}

WhiteLineDetector::~WhiteLineDetector()
{

}

void WhiteLineDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_pub;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr_pub = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image = cv_ptr->image;
    try
    {
        cvtColor(image, image, CV_RGB2GRAY);
        cv::GaussianBlur(image, image, cv::Size(5, 5), 1.0, 1.0);
        cv::threshold(image, image, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        std::vector<std::vector<cv::Point> > contours_subset;
        for(int i=0; i<contours.size();i++)
        { 
            double area=contourArea(contours.at(i));
            if(area < 5000)
            {
                contours_subset.push_back(contours.at(i));
            }
        }
        cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
        drawContours(mask,contours_subset,-1,cv::Scalar(255),-1);
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