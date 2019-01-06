#include <white_line_estimator/image_projector.h>

ImageProjector::ImageProjector(double min_area,double max_area) : tf_listener_(tf_buffer_)
{
    min_area_ = min_area;
    max_area_ = max_area;
    camera_info_ = boost::none;
}

ImageProjector::~ImageProjector()
{

}

void ImageProjector::setCameraInfo(sensor_msgs::CameraInfo info)
{
    camera_info_ = CvCameraInfo(info);
    return;
}

boost::optional<std::vector<ProjectedPoint> > ImageProjector::project(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
    cv::Mat& mask_image)
{
    if(!camera_info_)
    {
        ROS_ERROR_STREAM("cameara info does not recieved.");
        return boost::none;
    }
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
    cv::undistort(cv_ptr->image,cv_ptr->image,camera_info_->camera_matrix,camera_info_->dist_coeff);
    boost::optional<std::vector<ProjectedPoint> > projected_points = projectPointCloudToImage(*pointcloud_msg,image_msg->header.frame_id,cv::Size(cv_ptr->image.rows,cv_ptr->image.cols));
    if(!projected_points)
    {
        ROS_ERROR_STREAM("failed to project pointcloud to the image.");
        return boost::none;
    }
    mask_image = getGroundMaskImage(projected_points.get(),cv::Size(cv_ptr->image.rows,cv_ptr->image.cols));
    std::vector<std::vector<cv::Point> > contours = getWhiteLineContours(cv_ptr->image,mask_image);
}

cv::Mat ImageProjector::getGroundMaskImage(std::vector<ProjectedPoint> ground_points_in_image,cv::Size size)
{
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);;
    std::vector<cv::Point> contour = getGroundCovexHull(ground_points_in_image);
    drawContours(mask,contour,-1,cv::Scalar(255),-1);
    return mask;
}

std::vector<cv::Point> ImageProjector::getGroundCovexHull(std::vector<ProjectedPoint> ground_points_in_image)
{
    std::vector<cv::Point> approx,contour;
    for(auto point_itr = ground_points_in_image.begin(); point_itr != ground_points_in_image.end(); point_itr++)
    {
        contour.push_back(point_itr->point_2d);
    }
    cv::convexHull(contour,approx);
    return approx;
}

boost::optional<std::vector<ProjectedPoint> > ImageProjector::projectPointCloudToImage(sensor_msgs::PointCloud2 point_cloud,std::string camera_frame,cv::Size size)
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
    cv::Point p;
    try
    {
        p.x = (camera_info_->proj_matrix.fx * point_3d.point.x + camera_info_->proj_matrix.Tx)/point_3d.point.z + camera_info_->proj_matrix.cx;
        p.y = (camera_info_->proj_matrix.fy * point_3d.point.y + camera_info_->proj_matrix.Ty)/point_3d.point.z + camera_info_->proj_matrix.cy;
        if(p.x < 0 || p.x >= image_size.width)
        {
            return boost::none;
        }
        if(p.y < 0 || p.y >= image_size.height)
        {
            return boost::none;
        }
    }
    catch(...)
    {
        return boost::none;
    }
    return p;
}

std::vector<std::vector<cv::Point> > ImageProjector::getWhiteLineContours(cv::Mat image,cv::Mat &mask)
{
    cvtColor(image, image, CV_RGB2HSV);
    std::vector<cv::Mat> hsv_planes;
    cv::split(image, hsv_planes);
    image = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    image = image * mask;
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