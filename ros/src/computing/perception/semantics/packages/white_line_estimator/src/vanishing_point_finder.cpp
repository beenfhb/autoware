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

boost::optional<cv::Point> VanishingPointFinder::findCrossingPoint(std::array<cv::Point,2> line0, std::array<cv::Point,2> line1)
{
    cv::Point ret;
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
        ret.y = a0+ret.x*b0;
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
        ret.y = a1+ret.x*b1;
        return ret;
    }
    else if((line1[1].x-line1[0].x) == 0.0)
    {
        double a0 = (line0[1].y-line0[0].y)/(line0[1].x-line0[0].x);
        double b0 = -a0*line0[1].x+line0[1].y;
        ret.x = line1[1].x;
        ret.y = a0+ret.x*b0;
        return ret;
    }
    else
    {
        return boost::none;
    }
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