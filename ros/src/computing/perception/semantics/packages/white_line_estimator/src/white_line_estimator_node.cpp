// headers for ros
#include <ros/ros.h>

//headers in this package
#include <white_line_estimator/white_line_estimator.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "white_line_estimator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    WhiteLineDetector detector(nh,pnh);
    ros::spin();
    return 0;
}