#include <white_line_estimator/image_projector.h>

ImageProjector::ImageProjector(double min_area,double max_area)
{
    min_area_ = min_area;
    max_area_ = max_area;
    camera_info_ = boost::none;
    proj_matrix_ = boost::none;
}

ImageProjector::~ImageProjector()
{

}

void ImageProjector::setCameraInfo(sensor_msgs::CameraInfo info)
{
    std::lock_guard<std::mutex> lock(mtx_);
    camera_info_ = CvCameraInfo(info);
    return;
}

void ImageProjector::setProjectionMatrix(autoware_msgs::ProjectionMatrix proj_matrix)
{
    std::lock_guard<std::mutex> lock(mtx_);
    proj_matrix_ = CvProjMatrix(proj_matrix);
    return;
}

std::vector<cv::Point> ImageProjector::getGroundConvexHull(std::vector<ProjectedPoint> ground_points,cv::Mat& ground_image)
{
    std::vector<cv::Point> approx;
    std::vector<cv::Point> contour;
    //std::vector<std::vector<cv::Point> > contours;
    for(auto ground_point_itr = ground_points.begin(); ground_point_itr != ground_points.end(); ground_point_itr++)
    {
        contour.push_back(ground_point_itr->point_2d);
    }
    cv::convexHull(contour, approx);
    std::vector<std::vector<cv::Point> > ground_contours;
    ground_contours.push_back(approx);
    drawContours(ground_image,ground_contours,-1,cv::Scalar(0,255,0),-1);
    return approx;
}

boost::optional<std::vector<ProjectedPoint> > ImageProjector::project(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
    cv::Mat& mask_image, cv::Mat& ground_image)
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
    std::vector<ProjectedPoint> projected_points = projectPointCloudToImage(*pointcloud_msg,cv_ptr->image.size(),ground_image);
    std::vector<cv::Point> ground_contour = getGroundConvexHull(projected_points,ground_image);
    return projected_points;
}

std::vector<ProjectedPoint> ImageProjector::projectPointCloudToImage(sensor_msgs::PointCloud2 point_cloud,cv::Size size,cv::Mat& ground_image)
{
    std::vector<ProjectedPoint> projected_points;
    std_msgs::Header header = point_cloud.header;
    uintptr_t cp = (uintptr_t)point_cloud.data.data();
    for (uint32_t y = 0; y < point_cloud.height; ++y)
    {
        for (uint32_t x = 0; x < point_cloud.width; ++x)
        {
            cv::Mat point(1, 3, CV_64F);
            cv::Point2d imagepoint;
            float* fp = (float*)(cp + (x + y * point_cloud.width) * point_cloud.point_step);
            double intensity = fp[4];
            for (int i = 0; i < 3; i++)
            {
                point.at<double>(i) = proj_matrix_->invTt.at<double>(i);
                for (int j = 0; j < 3; j++)
                {
                    point.at<double>(i) += double(fp[j]) * proj_matrix_->invRt.at<double>(j, i);
                }
            }
            if (point.at<double>(2) <= 1)
            {
                continue;
            }
            double tmpx = point.at<double>(0) / point.at<double>(2);
            double tmpy = point.at<double>(1) / point.at<double>(2);
            double r2 = tmpx * tmpx + tmpy * tmpy;
            double tmpdist =
                1 + camera_info_->dist_coeff.at<double>(0) * r2 + camera_info_->dist_coeff.at<double>(1) * r2 * r2 + camera_info_->dist_coeff.at<double>(4) * r2 * r2 * r2;
            imagepoint.x =
                tmpx * tmpdist + 2 * camera_info_->dist_coeff.at<double>(2) * tmpx * tmpy + camera_info_->dist_coeff.at<double>(3) * (r2 + 2 * tmpx * tmpx);
            imagepoint.y =
                tmpy * tmpdist + camera_info_->dist_coeff.at<double>(2) * (r2 + 2 * tmpy * tmpy) + 2 * camera_info_->dist_coeff.at<double>(3) * tmpx * tmpy;
            imagepoint.x = camera_info_->camera_matrix.at<double>(0, 0) * imagepoint.x + camera_info_->camera_matrix.at<double>(0, 2);
            imagepoint.y = camera_info_->camera_matrix.at<double>(1, 1) * imagepoint.y + camera_info_->camera_matrix.at<double>(1, 2);

            int px = int(imagepoint.x + 0.5);
            int py = int(imagepoint.y + 0.5);
            if (0 <= px && px < size.width && 0 <= py && py < size.height)
            {
                ProjectedPoint projected_point;
                geometry_msgs::Point p;
                p.x = point.at<double>(0);
                p.y = point.at<double>(1);
                p.z = point.at<double>(2);
                projected_point.point_3d = p;
                projected_point.point_2d.x = px;
                projected_point.point_2d.y = py;
                projected_points.push_back(projected_point);
            }
        }
    }
    ground_image = cv::Mat::zeros(size, CV_8UC3);
    /*
    for(auto point_itr = projected_points.begin(); point_itr != projected_points.end(); point_itr++)
    {
        circle(ground_image, point_itr->point_2d, 1, cv::Scalar(255,0,0), -1, 4);
    }
    */
    return projected_points;
}