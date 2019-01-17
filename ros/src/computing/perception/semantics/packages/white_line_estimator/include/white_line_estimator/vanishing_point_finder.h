#ifndef VANISHING_POINT_FINDER_H_INCLUDED
#define VANISHING_POINT_FINDER_H_INCLUDED

//headers in OpenCV
#include <opencv2/opencv.hpp>

//headers in boost
#include <boost/optional.hpp>

//headers in STL
#include <array>

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
    std::vector<cv::Point2d> findCandidatePoints(std::vector<cv::Vec4i> lines);
    boost::optional<cv::Point2d> findCrossingPoint(std::array<cv::Point,2> line0, std::array<cv::Point,2> line1);
    HoughParams params_;
};
#endif  //VANISHING_POINT_FINDER_H_INCLUDED