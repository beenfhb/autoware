/*
 * test_tf.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <boost/filesystem.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "utilities.h"
#include "datasets/MeidaiBagDataset.h"

using namespace std;
namespace bfs = boost::filesystem;


CameraPinholeParams meidaiCamera1Params(
	1150.96938467,	// fx
	1150.96938467,	// fy
	988.511326762,	// cx
	692.803953253,	// cy
	1920,			// width
	1440			// height
);


int main (int argc, char *argv[])
{
	auto meidaiDs = MeidaiBagDataset::load("/media/sujiwo/ssd/motoyama.bag");
	meidaiDs->addCameraParameter(meidaiCamera1Params);

	auto frame0 = meidaiDs->getAsFrame(0),
		frame1 = meidaiDs->getAsFrame(25);

	auto F01 = BaseFrame::FundamentalMatrix(*frame0, *frame1);
	cout << F01 << endl;

	return 0;
}
