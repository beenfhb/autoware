#ifndef COLOR_FILTER_H_INCLUDED
#define COLOR_FILTER_H_INCLUDED

//headers in OpenCV
#include <opencv2/imgproc/imgproc.hpp>

class ColorFilter
{
public:
    ColorFilter();
    ~ColorFilter();
    cv::Mat filterWhiteLine(cv::Mat image,cv::Mat ground_mask);
    void updateParameters(double min_white_line_area,double max_white_line_area);
private:
    double max_white_line_area_;
    double min_white_line_area_;
    cv::Mat filterWhiteLineContours(cv::Mat image);
};

#endif  //COLOR_FILTER_H_INCLUDED