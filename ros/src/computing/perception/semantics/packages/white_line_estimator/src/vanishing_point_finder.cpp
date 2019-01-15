#include <white_line_estimator/vanishing_point_finder.h>

VanishingPointFinder::VanishingPointFinder()
{

}

VanishingPointFinder::~VanishingPointFinder()
{

}

boost::optional<cv::Point> VanishingPointFinder::find(cv::Mat image)
{
    cv::Point p;
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(image,lines,params_.hough_rho,params_.hough_theta,
        params_.hough_threshold,params_.hough_min_line_length,params_.hough_max_line_gap);
    return p;
}

std::vector<cv::Point> VanishingPointFinder::findCandidatePoints(std::vector<cv::Vec4i> lines)
{
    std::vector<cv::Point> points;
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
        }
    }
    return points;
}