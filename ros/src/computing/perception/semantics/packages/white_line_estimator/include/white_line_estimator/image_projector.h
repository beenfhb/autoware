#ifndef IMAGE_PROJECTOR_H_INCLUDED
#define IMAGE_PROJECTOR_H_INCLUDED

//headers in ROS
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

//headers in OpenCV
#include <opencv2/imgproc/imgproc.hpp>

//headers in PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

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