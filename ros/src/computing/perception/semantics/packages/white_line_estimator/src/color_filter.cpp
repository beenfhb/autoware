#include <white_line_estimator/color_filter.h>

ColorFilter::ColorFilter()
{

}

ColorFilter::~ColorFilter()
{

}

std::vector<std::vector<cv::Point> > ColorFilter::filterWhiteLine(cv::Mat image,cv::Mat ground_mask,cv::Mat& filtered_image)
{
    cv::Mat ground_image = cv::Mat::zeros(image.size(),CV_8UC1);
    filtered_image = cv::Mat::zeros(image.size(),CV_8UC1);
    cv::Mat hsv_image;
    cvtColor(image,hsv_image,CV_BGR2HSV);
    std::vector<cv::Mat> planes;
    cv::split(image,planes);
    cv::Mat white_line_mask;
    threshold(planes[2], white_line_mask, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    bitwise_and(white_line_mask,ground_mask,ground_image);
    std::vector<std::vector<cv::Point> > contours = filterWhiteLineContours(ground_image,filtered_image);
    return contours;
}

void ColorFilter::updateParameters(double min_white_line_area,double max_white_line_area)
{
    min_white_line_area_ = min_white_line_area;
    max_white_line_area_ = max_white_line_area;
    return;
}

std::vector<std::vector<cv::Point> > ColorFilter::filterWhiteLineContours(cv::Mat image, cv::Mat& dst_image)
{
    std::vector<std::vector<cv::Point> > contours;
    findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    std::vector<std::vector<cv::Point> > contours_subset;
    for(int i=0; i<contours.size();i++)
    {
        double area=contourArea(contours.at(i));
        if(area>min_white_line_area_&&area<max_white_line_area_)
        {
            contours_subset.push_back(contours.at(i));
        }
    }
    dst_image = cv::Mat::zeros(image.size(), CV_8UC1);
    drawContours(dst_image,contours_subset,-1,cv::Scalar(255),-1);
    return contours_subset;
}