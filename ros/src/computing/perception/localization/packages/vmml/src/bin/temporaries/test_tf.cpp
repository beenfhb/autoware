/*
 * test_tf.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include "datasets/LidarScanBag.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
namespace bfs = boost::filesystem;

int main (int argc, char *argv[])
{
	rosbag::Bag bagfd ("/media/sujiwo/ssd/log_2016-12-26-13-21-10.bag");

	bfs::path myPath(ros::package::getPath("vmml"));
	auto mask1Path = myPath / "params/64e-S2.yaml";

	RandomAccessBag rdbag (bagfd, "/camera1/image_raw", 315.49, 932.16);
	auto imageMsg = rdbag.at<sensor_msgs::Image>(60);
	auto s = imageMsg->header.stamp;


//	LidarScanBag lidarBag(bagfd, "/velodyne_packets", mask1Path.string(), 315.49, 932.16);
//
//	auto msg1000 = lidarBag.at(1000);
//	return LidarScanBag::save(msg1000, "/tmp/test1000.pcd");

	return 0;
}
