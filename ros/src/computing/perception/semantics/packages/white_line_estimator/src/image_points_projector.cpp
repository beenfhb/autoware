#include <white_line_estimator/image_points_projector.h>

ImagePointsProjector::ImagePointsProjector()
{

}

ImagePointsProjector::~ImagePointsProjector()
{
    
}

boost::optional<std::vector<std::vector<geometry_msgs::Point> > > ImagePointsProjector::project(std::vector<std::vector<cv::Point> > image_points)
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
    std::vector<std::vector<geometry_msgs::Point> > projected_points_array;
    Eigen::MatrixXd p_mat = camera_info_->eigen_camera_matrix * proj_matrix_->eigen_proj_matrix;
    Eigen::MatrixXd p_mat_inv = p_mat.inverse();
    for(auto image_points_itr = image_points.begin(); image_points_itr != image_points.end(); image_points_itr++)
    {
        std::vector<geometry_msgs::Point> projected_points;
        for(auto image_point_itr = image_points_itr->begin(); image_point_itr != image_points_itr->end(); image_point_itr++)
        {
            Eigen::MatrixXd image_point_mat = Eigen::MatrixXd(3,1);
            image_point_mat(0,0) = image_point_itr->x;
            image_point_mat(1,0) = image_point_itr->y;
            image_point_mat(2,0) = 1;
            geometry_msgs::Point world_point;
            Eigen::MatrixXd world_point_mat = Eigen::MatrixXd(4,1);;
            world_point_mat = p_mat_inv * image_point_mat;
            world_point.x = world_point_mat(0,0);
            world_point.y = world_point_mat(1,0);
            world_point.z = world_point_mat(2,0);
            projected_points.push_back(world_point);
        }
        projected_points_array.push_back(projected_points);
    }
    return projected_points_array;
}

void ImagePointsProjector::getRPY(geometry_msgs::Quaternion q,double &roll,double &pitch,double &yaw)
{
    tf2::Quaternion quat(q.x,q.y,q.z,q.w);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return;
}

void ImagePointsProjector::getQuaternion(double roll,double pitch,double yaw,geometry_msgs::Quaternion& q)
{
    tf2::Quaternion quat;
    quat.setRPY(roll,pitch,yaw);
    q.x = quat.getAxis().x()*cos(quat.getAxis().w());
    q.y = quat.getAxis().y()*cos(quat.getAxis().w());
    q.z = quat.getAxis().z()*cos(quat.getAxis().w());
    q.w = quat.getAxis().w();
    return;
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