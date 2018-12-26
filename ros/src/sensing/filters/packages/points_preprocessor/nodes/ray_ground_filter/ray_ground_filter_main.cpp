
#include "ray_ground_filter.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "ray_ground_filter");
  RayGroundFilter app;

  app.Run();

  return 0;
}
