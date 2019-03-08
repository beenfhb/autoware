// headers in this Autoware

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cuda_diagnostic_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::spin();
  return 0;
}