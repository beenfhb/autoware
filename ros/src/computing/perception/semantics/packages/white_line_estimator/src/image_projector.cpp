#include <white_line_estimator/image_projector.h>

ImageProjector::ImageProjector()
{

}

ImageProjector::~ImageProjector()
{

}

void ImageProjector::project(const sensor_msgs::PointCloud2ConstPtr& pointcloud_ptr,
    const cv::Mat& proj_matrix, const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeff, const cv::Size& image_size)
{
    int w = image_size.width;
    int h = image_size.height;
    return;
}