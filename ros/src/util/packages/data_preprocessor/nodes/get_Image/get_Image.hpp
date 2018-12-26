#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

class SaveImage {
public:
  string save_path_;
  string topic_name_;
  void save_image(const sensor_msgs::Image &msg);
  void sub_image(int argc, char *argv[]);
};

static void check_arguments(int argc, char *argv[]);
// static void save_image(const sensor_msgs::Image& msg);
