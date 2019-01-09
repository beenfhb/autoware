#include <white_line_estimator/color_filter.h>

ColorFilter::ColorFilter()
{

}

ColorFilter::~ColorFilter()
{

}

cv::Mat ColorFilter::filterWhiteLine(cv::Mat image,cv::Mat ground_mask)
{
    cv::Mat ret = cv::Mat::zeros(image.size(),CV_8UC1);
    cv::Mat hsv_image;
    cvtColor(image,hsv_image,CV_BGR2HSV);
    std::vector<cv::Mat> planes;
    cv::split(image,planes);
    cv::Mat white_line_mask;
    threshold(planes[2], white_line_mask, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    bitwise_and(white_line_mask,ground_mask,ret);
    return ret;
}

void ColorFilter::updateParameters(double min_white_line_area,double max_white_line_area)
{
    min_white_line_area_ = min_white_line_area;
    max_white_line_area_ = max_white_line_area;
    return;
}

void ColorFilter::filterWhiteLineContours(cv::Mat& image)
{
    return;
}