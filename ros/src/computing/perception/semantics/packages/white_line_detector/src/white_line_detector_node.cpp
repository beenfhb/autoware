// headers for ros
#include <ros/ros.h>

//headers in this package
#include <white_line_detector/white_line_detector.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "white_line_detctor_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    WhiteLineDetector detector(nh,pnh);
    ros::spin();
    return 0;
}