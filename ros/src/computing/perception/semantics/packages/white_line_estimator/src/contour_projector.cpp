#include <white_line_estimator/contour_projector.h>

ContourProjector::ContourProjector()
{

}

ContourProjector::~ContourProjector()
{

}

void ContourProjector::projectWhiteLineContours(std::vector<ProjectedPoint> projected_pointcloud,std::vector<std::vector<cv::Point> > contors)
{
    cv::Mat data = (cv::Mat_<double>(2,projected_pointcloud.size()));
    for(int i=0; i<projected_pointcloud.size(); i++)
    {
        data.data[2*i] = projected_pointcloud[i].point_2d.x;
        data.data[2*i+1] = projected_pointcloud[i].point_2d.y;
    }
    cv::flann::Index idx(data, cv::flann::KDTreeIndexParams(4), cvflann::FLANN_DIST_EUCLIDEAN);
    return;
}