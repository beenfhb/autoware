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
private:
    cv::Mat filterColor(cv::Mat image,cv::Mat camera_matrix,cv::Mat dist_coeff,double min_area,double max_area);
};
#endif  //IMAGE_PROJECTOR_H_INCLUDED