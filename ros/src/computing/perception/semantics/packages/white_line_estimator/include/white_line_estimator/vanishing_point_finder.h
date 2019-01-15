#ifndef VANISHING_POINT_FINDER_H_INCLUDED
#define VANISHING_POINT_FINDER_H_INCLUDED

//headers in OpenCV
#include <opencv2/opencv.hpp>

//headers in boost
#include <boost/optional.hpp>

struct HoughParams
{
    double hough_rho;
    double hough_theta;
    int hough_threshold;
    double hough_min_line_length;
    double hough_max_line_gap;
};

class VanishingPointFinder
{
public:
    VanishingPointFinder();
    ~VanishingPointFinder();
    boost::optional<cv::Point> find(cv::Mat image);
    void setParameters(HoughParams params)
    {
        params_ = params;
    }
private:
    std::vector<cv::Point> findCandidatePoints(std::vector<cv::Vec4i> lines);
    HoughParams params_;
};
#endif  //VANISHING_POINT_FINDER_H_INCLUDED