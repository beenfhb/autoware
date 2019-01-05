#ifndef RANSAC_APPROXIMATOR_H_INCLUDED
#define RANSAC_APPROXIMATOR_H_INCLUDED

#include <opencv2/imgproc/imgproc.hpp>

struct WhiteLineModel
{

};

class RansacApproximator
{
public:
    RansacApproximator();
    ~RansacApproximator();
    void approximate(std::vector<cv::Point> contour);
private:
    cv::Point2f getCenterOfMoment(std::vector<cv::Point> contour);
};
#endif  //RANSAC_APPROXIMATOR_H_INCLUDED