#include <white_line_estimator/vanishing_point_finder.h>

VanishingPointFinder::VanishingPointFinder()
{
    pf_config_recieved_ = false;
}

VanishingPointFinder::~VanishingPointFinder()
{

}

cv::Point2d VanishingPointFinder::estimateVanishingPoint(std::vector<cv::Point2d> candidate_points)
{
    cv::Point2d p;
    cv::Mat_<float> points_mat(0,2);
    // if no candidate points were found.
    if(candidate_points.size() == 0)
    {
        return p;
    }
    for(auto && point : candidate_points)
    {
        cv::Mat row = (cv::Mat_<float>(1, 2) << point.x, point.y);
        points_mat.push_back(row);
    }
    boost::shared_ptr<cv::flann::Index> flann_index_ptr;
    try
    {
        flann_index_ptr = boost::make_shared<cv::flann::Index>(points_mat, cv::flann::KDTreeIndexParams(1));
    }
    catch(...)
    {
        ROS_ERROR_STREAM("failed to calculate kd-tree index");
        return p;
    }
    Eigen::MatrixXd states = particle_filter_ptr_->getStates();
    for (int i = 0; i < pf_params_.num_particles; i++)
    {
        cv::Mat query = (cv::Mat_<float>(1, 2) << states(0,i),states(1,i));
        std::vector<int> indices;
        std::vector<float> dists;
        flann_index_ptr->radiusSearch(query, indices, dists, rad_params_.radius, rad_params_.max_neighbours,cv::flann::SearchParams(32));
        getWeight(indices,dists);
    }
    return p;
}

double VanishingPointFinder::getWeight(std::vector<int> indices,std::vector<float> dists)
{
    double ret;
    for(int i=0; i<indices.size(); i++)
    {
        //ROS_ERROR_STREAM("i = " << i << ",index = " << indices[i]);
    }
    return ret;
}

boost::optional<cv::Point> VanishingPointFinder::find(cv::Mat image)
{
    cv::Point p;
    std::vector<cv::Vec4i> lines;
    if(!pf_config_recieved_)
    {
        return boost::none;
    }
    cv::HoughLinesP(image,lines,params_.hough_rho,params_.hough_theta,
        params_.hough_threshold,params_.hough_min_line_length,params_.hough_max_line_gap);
    std::vector<cv::Point2d> candidate_points = findCandidatePoints(lines,image.size());
    cv::Point2d vanishing_point = estimateVanishingPoint(candidate_points);
    return p;
}

boost::optional<cv::Point2d> VanishingPointFinder::findCrossingPoint(std::array<cv::Point,2> line0, std::array<cv::Point,2> line1)
{
    cv::Point2d ret;
    if ((line0[1].x-line0[0].x) != 0.0 && (line1[1].x-line1[0].x) != 0.0)
    {
        double a0 = (line0[1].y-line0[0].y)/(line0[1].x-line0[0].x);
        double a1 = (line1[1].y-line1[0].y)/(line1[1].x-line1[0].x);
        if(a0 == a1)
        {
            return boost::none;
        }
        double b0 = -a0*line0[1].x+line0[1].y;
        double b1 = -a1*line1[1].x+line1[1].y;
        ret.x = (b1-b0)/(a0-a1);
        ret.y = b0+ret.x*a0;
        return ret;
    }
    else if((line0[1].x-line0[0].x) == 0.0 && (line1[1].x-line1[0].x) == 0.0)
    {
        return boost::none;
    }
    else if((line0[1].x-line0[0].x) == 0.0)
    {
        double a1 = (line1[1].y-line1[0].y)/(line1[1].x-line1[0].x);
        double b1 = -a1*line1[1].x+line1[1].y;
        ret.x = line0[1].x;
        ret.y = b1+ret.x*a1;
        return ret;
    }
    else if((line1[1].x-line1[0].x) == 0.0)
    {
        double a0 = (line0[1].y-line0[0].y)/(line0[1].x-line0[0].x);
        double b0 = -a0*line0[1].x+line0[1].y;
        ret.x = line1[1].x;
        ret.y = b0+ret.x*a0;
        return ret;
    }
    else
    {
        return boost::none;
    }
}

std::vector<cv::Point2d> VanishingPointFinder::findCandidatePoints(std::vector<cv::Vec4i> lines,cv::Size size)
{
    std::vector<cv::Point2d> points;
    for(int i=0; i<lines.size(); i++)
    {
        for(int m=0; m<lines.size(); m++)
        {
            if(i == m)
            {
                continue;
            }
            cv::Vec4i l0,l1;
            l0 = lines[i];
            l1 = lines[m];
            cv::Point p0 = cv::Point(l0[0],l0[1]);
            cv::Point p1 = cv::Point(l0[2],l0[3]);
            cv::Point p2 = cv::Point(l1[0],l1[1]);
            cv::Point p3 = cv::Point(l1[2],l1[3]);
            std::array<cv::Point,2> line0 = {p0,p1};
            std::array<cv::Point,2> line1 = {p2,p3};
            boost::optional<cv::Point2d> crossing_point = findCrossingPoint(line0,line1);
            if(crossing_point)
            {
                if(crossing_point->x < 0 || crossing_point->x >= size.width)
                {
                    continue;
                }
                if(crossing_point->y < 0 || crossing_point->y >= size.height)
                {
                    continue;
                }
                points.push_back(crossing_point.get());
            }
        }
    }
    return points;
}