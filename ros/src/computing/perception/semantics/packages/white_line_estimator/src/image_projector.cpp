#include <white_line_estimator/image_projector.h>

ImageProjector::ImageProjector(double min_area,double max_area) : tf_listener_(tf_buffer_)
{
    min_area_ = min_area;
    max_area_ = max_area;
}

ImageProjector::~ImageProjector()
{

}

boost::optional<std::vector<ProjectedPoint> > ImageProjector::project(const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,cv::Mat camera_matrix,cv::Mat dist_coeff)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return boost::none;
    }
    cv::Mat mask;
    std::vector<std::vector<cv::Point> > contours = getWhiteLineContours(cv_ptr->image,mask,camera_matrix,dist_coeff);
    boost::optional<std::vector<ProjectedPoint> > projected_points = projectPointCloudToImage(*pointcloud_msg,image_msg->header.frame_id,contours,cv::Size(mask.rows,mask.cols));
    if(!projected_points)
    {
        ROS_ERROR_STREAM("failed to project pointcloud to the image.");
        return boost::none;
    }
}

boost::optional<std::vector<ProjectedPoint> > ImageProjector::projectPointCloudToImage(sensor_msgs::PointCloud2 point_cloud,std::string camera_frame,
    std::vector<std::vector<cv::Point> > contours, cv::Size size)
{
    std::vector<ProjectedPoint> projected_points;
    std_msgs::Header header = point_cloud.header;
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(camera_frame, header.frame_id, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return boost::none;
    }
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(point_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(point_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(point_cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        ProjectedPoint p;
        p.point_3d.header = header;
        p.point_3d.point.x = *iter_x;
        p.point_3d.point.y = *iter_y;
        p.point_3d.point.z = *iter_z;
        tf2::doTransform(p.point_3d,p.point_3d,transform_stamped);
        boost::optional<cv::Point> point_2d = projectPoint3dPointTo2d(p.point_3d,size);
        if(!point_2d)
        {
            continue;
        }
        p.point_2d = point_2d.get();
        projected_points.push_back(p);
    }
    return projected_points;
}

boost::optional<cv::Point> ImageProjector::projectPoint3dPointTo2d(geometry_msgs::PointStamped point_3d,cv::Size image_size)
{
    return boost::none;
}

std::vector<std::vector<cv::Point> > ImageProjector::getWhiteLineContours(cv::Mat image,cv::Mat &mask,cv::Mat camera_matrix,cv::Mat dist_coeff)
{
    cvtColor(image, image, CV_RGB2HSV);
    std::vector<cv::Mat> hsv_planes;
    cv::split(image, hsv_planes);
    image = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    cv::undistort(image,image,camera_matrix,dist_coeff);
    image = hsv_planes[2];
    cv::GaussianBlur(image, image, cv::Size(11, 11), 1.0, 1.0);
    cv::threshold(image, image, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    std::vector<std::vector<cv::Point> > contours_subset;
    for(int i=0; i<contours.size();i++)
    { 
        double area=contourArea(contours.at(i));
        if(area > min_area_ && area < max_area_)
        {
            contours_subset.push_back(contours.at(i));
        }
    }
    mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    drawContours(mask,contours_subset,-1,cv::Scalar(255),-1);
    cvtColor(mask, mask, CV_GRAY2BGR);
    return contours_subset;
}