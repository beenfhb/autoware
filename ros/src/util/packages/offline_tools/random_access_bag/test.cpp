/*
 * test.cpp
 *
 *  Created on: Dec 3, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>

#include "RandomAccessBag.h"

using namespace std;


int main (int argc, char *argv[])
{
	rosbag::Bag _mybag("/media/sujiwo/ssd/log_2016-12-26-13-21-10.bag", rosbag::BagMode::Read);

//	RandomAccessBag mybag(_mybag, "/camera1/image_raw");
	RandomAccessBag mybag(_mybag, "/camera1/image_raw", 315.49, 932.16);
	auto imageMsg = mybag.at<sensor_msgs::Image>(120);
	cout << imageMsg->width << endl;

	return 0;
}
