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
using namespace Eigen;
namespace bfs = boost::filesystem;


int main (int argc, char *argv[])
{
	TTransform I;
	I.translation() = Vector3d(1, 1, 1);
	I = I / 0.5;
	cout << dumpVector(I.position()) << endl;
	cout << dumpVector(I.orientation()) << endl;
}
