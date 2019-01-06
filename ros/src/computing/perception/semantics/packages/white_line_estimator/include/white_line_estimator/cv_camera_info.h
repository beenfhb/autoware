#ifndef CV_CAMERA_INFO_H_INCLUDED
#define CV_CAMERA_INFO_H_INCLUDED

struct CvCameraInfo
{
    cv::Mat camera_matrix;
    cv::Mat dist_coeff;
    cv::Mat proj_matrix;
    double Tx;
    double Ty;
    CvCameraInfo(sensor_msgs::CameraInfo info)
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
        proj_matrix = cv::Mat(4, 3, CV_64F);
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                camera_matrix.at<double>(row, col) = info.P[row * 3 + col];
            }
        }
        Tx = info.P[3];
        Ty = info.P[7];
    }
};

#endif  //CV_CAMERA_INFO_H_INCLUDED