/*
 * test_priv.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <string>
#include <utility>
#include <Eigen/Eigen>

#include "datasets/MeidaiBagDataset.h"
#include "utilities.h"


using namespace std;
using namespace Eigen;


int main(int argc, char *argv[])
{
	MeidaiBagDataset::Ptr meidaiDs = MeidaiBagDataset::load("/media/sujiwo/ssd/motoyama.bag");
	auto cameraTrack = meidaiDs->getCompleteCameraTrajectory();
	PoseStamped p1000 = cameraTrack[1000];

	cout << dumpVector(p1000) << endl;

	cout << dumpVector(p1000.shift(Vector3d(0, -3.0, 0))) << endl;

	return 0;
}
