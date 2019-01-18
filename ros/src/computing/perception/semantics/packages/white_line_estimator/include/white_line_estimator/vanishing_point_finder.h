#ifndef VANISHING_POINT_FINDER_H_INCLUDED
#define VANISHING_POINT_FINDER_H_INCLUDED

//headers in OpenCV
#include <opencv2/opencv.hpp>

//headers in boost
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

//headers in STL
#include <array>

//headers in Autoware
#include <white_line_estimator/particle_filter.h>

struct HoughParams
{
    double hough_rho;
    double hough_theta;
    int hough_threshold;
    double hough_min_line_length;
    double hough_max_line_gap;
};

struct ParticleFilterParams
{
    int num_particles;
    int init_vanishing_point_x;
    int init_vanishing_point_y;
};

class VanishingPointFinder
{
public:
    VanishingPointFinder();
    ~VanishingPointFinder();
    boost::optional<cv::Point> find(cv::Mat image);
    void setHoughParameters(HoughParams params)
    {
        params_ = params;
    }
    void setParticleFilterParameters(ParticleFilterParams params)
    {
        pf_config_recieved_ = true;
        pf_params_ = params;
        Eigen::VectorXd init_value = Eigen::VectorXd(2);
        init_value << pf_params_.init_vanishing_point_x,pf_params_.init_vanishing_point_y;
        particle_filter_ptr_ = boost::make_shared<ParticleFilter>(2,pf_params_.num_particles,init_value);
    }
private:
    cv::Point2d estimateVanishingPoint(std::vector<cv::Point2d> candidate_points);
    std::vector<cv::Point2d> findCandidatePoints(std::vector<cv::Vec4i> lines);
    boost::optional<cv::Point2d> findCrossingPoint(std::array<cv::Point,2> line0, std::array<cv::Point,2> line1);
    HoughParams params_;
    boost::shared_ptr<ParticleFilter> particle_filter_ptr_;
    ParticleFilterParams pf_params_;
    volatile bool pf_config_recieved_;
};
#endif  //VANISHING_POINT_FINDER_H_INCLUDED