#include <white_line_estimator/image_points_projector.h>

ImagePointsProjector::ImagePointsProjector()
{

}

ImagePointsProjector::~ImagePointsProjector()
{
    
}

boost::optional<std::vector<geometry_msgs::Point> > ImagePointsProjector::project(std::vector<cv::Point> image_points)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if(!camera_info_)
    {
        ROS_ERROR_STREAM("cameara info does not recieved.");
        return boost::none;
    }
    if(!proj_matrix_)
    {
        ROS_ERROR_STREAM("projection matrix does not recieved.");
        return boost::none;
    }
    std::vector<geometry_msgs::Point> projected_points;
    return projected_points;
}

void ImagePointsProjector::setCameraInfo(sensor_msgs::CameraInfo info)
{
    std::lock_guard<std::mutex> lock(mtx_);
    camera_info_ = CvCameraInfo(info);
    return;
}

void ImagePointsProjector::setProjectionMatrix(autoware_msgs::ProjectionMatrix proj_matrix)
{
    std::lock_guard<std::mutex> lock(mtx_);
    proj_matrix_ = CvProjMatrix(proj_matrix);
    return;
}