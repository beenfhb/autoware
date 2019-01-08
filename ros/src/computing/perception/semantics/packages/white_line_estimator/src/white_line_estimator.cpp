//headers in this package
#include <white_line_estimator/white_line_estimator.h>

WhiteLineEstimator::WhiteLineEstimator(ros::NodeHandle nh,ros::NodeHandle pnh) : it_(nh)
{
    nh_ = nh;
    pnh_ = pnh;
    image_projector_ptr_ = boost::make_shared<ImageProjector>(500,15000);
    image_pub_ = it_.advertise("/ground_image", 10);
    image_sub_ptr_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::Image> >(nh_, "image_raw", 1);
    pointcloud_sub_ptr_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2> >(nh_, "points_ground", 1);
    sync_ptr_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(10), *image_sub_ptr_, *pointcloud_sub_ptr_);
    sync_ptr_->registerCallback(boost::bind(&WhiteLineEstimator::sensorCallback,this,_1,_2));
    camera_info_sub_ = nh_.subscribe("camera_info",1,&WhiteLineEstimator::cameraInfoCallback,this);
    projection_matrix_sub_ = nh_.subscribe("projection_matrix",1,&WhiteLineEstimator::projectionMatrixCallback,this);
}

WhiteLineEstimator::~WhiteLineEstimator()
{

}

void WhiteLineEstimator::sensorCallback(const sensor_msgs::ImageConstPtr& image,const sensor_msgs::PointCloud2ConstPtr& pointcloud)
{
    cv::Mat mask;
    cv::Mat ground_image;
    boost::optional<std::vector<ProjectedPoint> > projected_points = image_projector_ptr_->project(image,pointcloud,mask,ground_image);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
        cv_ptr->image = ground_image;
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    return;
}

void WhiteLineEstimator::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    image_projector_ptr_->setCameraInfo(*msg);
    return;
}

void WhiteLineEstimator::projectionMatrixCallback(const autoware_msgs::ProjectionMatrixConstPtr& msg)
{
    image_projector_ptr_->setProjectionMatrix(*msg);
    return;
}