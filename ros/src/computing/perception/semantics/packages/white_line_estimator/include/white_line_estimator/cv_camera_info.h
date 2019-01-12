#ifndef CV_CAMERA_INFO_H_INCLUDED
#define CV_CAMERA_INFO_H_INCLUDED

//headers in ROS
#include <sensor_msgs/CameraInfo.h>

//headers in OpenCV
#include <opencv2/imgproc/imgproc.hpp>

//headers in Eigen
#include <Eigen/Core>

//headers in Autoware
#include <autoware_msgs/ProjectionMatrix.h>

struct CvProjMatrix
{
    cv::Mat invRt;
    cv::Mat invTt;
    Eigen::MatrixXd eigen_proj_mat;
    CvProjMatrix(autoware_msgs::ProjectionMatrix proj_matrix)
    {
        cv::Mat cv_proj_mat = cv::Mat(4, 4, CV_64F);
        eigen_proj_mat = Eigen::MatrixXd(4,4);
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                eigen_proj_mat(row,col) = proj_matrix.projection_matrix[row * 4 + col];
                cv_proj_mat.at<double>(row, col) = proj_matrix.projection_matrix[row * 4 + col];
            }
        }
        invRt = cv_proj_mat(cv::Rect(0, 0, 3, 3));
        cv::Mat invT = -invRt.t() * (cv_proj_mat(cv::Rect(3, 0, 1, 3)));
        invTt = invT.t();
    }
};

struct CvCameraInfo
{
    cv::Mat camera_matrix;
    cv::Mat dist_coeff;
    Eigen::MatrixXd eigen_camera_matrix;
    CvCameraInfo(sensor_msgs::CameraInfo info)
    {
        camera_matrix = cv::Mat(3, 3, CV_64F);
        eigen_camera_matrix = Eigen::MatrixXd(3, 4);
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                camera_matrix.at<double>(row, col) = info.K[row * 3 + col];
            }
        }
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                eigen_camera_matrix(row, col) = info.K[row * 4 + col];
            }
        }
        dist_coeff = cv::Mat(1, 5, CV_64F);
        for (int col = 0; col < 5; col++)
        {
            dist_coeff.at<double>(col) = info.D[col];
        }
    }
};

#endif  //CV_CAMERA_INFO_H_INCLUDED