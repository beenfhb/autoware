#ifndef WHITE_LINE_ESTIMATOR_HINCLUDED
#define WHITE_LINE_ESTIMATOR_HINCLUDED

//headers in ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

//headers in boost
#include <boost/shared_ptr.hpp>

//headers in Autoware
#include <autoware_msgs/ProjectionMatrix.h>
#include <white_line_estimator/image_points_projector.h>
#include <white_line_estimator/pointcloud_projector.h>
#include <white_line_estimator/color_filter.h>
#include <white_line_estimator/white_line_estimatorConfig.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy;

class WhiteLineEstimator
{
public:
    WhiteLineEstimator(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~WhiteLineEstimator();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher marker_pub_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber projection_matrix_sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    void projectionMatrixCallback(const autoware_msgs::ProjectionMatrixConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void sensorCallback(const sensor_msgs::ImageConstPtr& image,const sensor_msgs::PointCloud2ConstPtr& pointcloud);
    cv::Mat proj_matrix_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;
    cv::Size image_size_;
    boost::shared_ptr<PointCloudProjector> pointcloud_projector_ptr_;
    boost::shared_ptr<ImagePointsProjector> image_points_projector_ptr_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > image_sub_ptr_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > pointcloud_sub_ptr_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_ptr_;
    ColorFilter filter_;
    dynamic_reconfigure::Server<white_line_estimator::white_line_estimatorConfig> server_;
    void configureCallback(white_line_estimator::white_line_estimatorConfig &config, uint32_t level);
    dynamic_reconfigure::Server<white_line_estimator::white_line_estimatorConfig>::CallbackType callback_func_type_;
};
#endif  //WHITE_LINE_ESTIMATOR_HINCLUDED