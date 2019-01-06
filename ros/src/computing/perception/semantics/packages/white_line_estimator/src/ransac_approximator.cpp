//headers in this package
#include <white_line_estimator/ransac_approximator.h>

RansacApproximator::RansacApproximator()
{
    
}

RansacApproximator::~RansacApproximator()
{

}

void RansacApproximator::approximate(std::vector<cv::Point> contour)
{
    cv::Point2f com = getCenterOfMoment(contour);
    return;
}

cv::Point2f RansacApproximator::getCenterOfMoment(std::vector<cv::Point> contour)
{
    cv::Point2f com;
    cv::Moments mu = moments(contour);
    com = cv::Point2f(mu.m10/mu.m00,mu.m01/mu.m00);
    return com;
}