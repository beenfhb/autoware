#ifndef IMAGE_PROJECTOR_H_INCLUDED
#define IMAGE_PROJECTOR_H_INCLUDED

//headers in ROS
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

//headers in OpenCV
#include <opencv2/imgproc/imgproc.hpp>

//headers in STL
#include <map>

class ImageProjector
{
public:
    ImageProjector();
    ~ImageProjector();
    void project(const sensor_msgs::PointCloud2ConstPtr& pointcloud_ptr,
        const cv::Mat& proj_matrix, const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeff, const cv::Size& image_size);
private:
};
#endif  //IMAGE_PROJECTOR_H_INCLUDED