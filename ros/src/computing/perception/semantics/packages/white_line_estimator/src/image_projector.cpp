#include <white_line_estimator/image_projector.h>

ImageProjector::ImageProjector()
{

}

ImageProjector::~ImageProjector()
{

}

cv::Mat ImageProjector::filterColor(cv::Mat image,cv::Mat camera_matrix,cv::Mat dist_coeff,double min_area,double max_area)
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
        if(area > min_area && area < max_area)
        {
            contours_subset.push_back(contours.at(i));
        }
    }
    cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    drawContours(mask,contours_subset,-1,cv::Scalar(255),-1);
    cvtColor(mask, mask, CV_GRAY2BGR);
    return mask;
}