#ifndef CV_CAMERA_INFO_H_INCLUDED
#define CV_CAMERA_INFO_H_INCLUDED

//headers in ROS
#include <sensor_msgs/CameraInfo.h>

//headers in OpenCV
#include <opencv2/imgproc/imgproc.hpp>

struct CvProjMatrix
{
    cv::Mat matrix;
    double Tx;
    double Ty;
    double fx;
    double fy;
    double cx;
    double cy;
    CvProjMatrix(sensor_msgs::CameraInfo info)
    {
        matrix = cv::Mat(4, 3, CV_64F);
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                matrix.at<double>(row, col) = info.P[row * 3 + col];
            }
        }
        Tx = info.P[3];
        Ty = info.P[7];
        cx = info.P[2];
        cy = info.P[6];
        fx = info.P[0];
        fy = info.P[5];
    }
};

struct CvCameraInfo
{
    cv::Mat camera_matrix;
    cv::Mat dist_coeff;
    CvProjMatrix proj_matrix;
    CvCameraInfo(sensor_msgs::CameraInfo info) : proj_matrix(info)
    {
        camera_matrix = cv::Mat(3, 3, CV_64F);
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                camera_matrix.at<double>(row, col) = info.K[row * 3 + col];
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