#ifndef CONTOUR_PROJECTOR_H_INCLUDED
#define CONTOUR_PROJECTOR_H_INCLUDED

//headers in OpenCV
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//headers in Autoware
#include <white_line_estimator/image_projector.h>

class ContourProjector
{
public:
    ContourProjector();
    ~ContourProjector();
    void projectWhiteLineContours(std::vector<ProjectedPoint> projected_pointcloud,std::vector<std::vector<cv::Point> > contors);
};

#endif  //CONTOUR_PROJECTOR_H_INCLUDED