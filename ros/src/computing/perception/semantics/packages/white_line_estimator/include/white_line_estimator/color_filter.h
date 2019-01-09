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
};

#endif  //COLOR_FILTER_H_INCLUDED