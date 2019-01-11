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
	Line2 AB = Line2::Through(Vector2d(-3,0), Vector2d(0,2));

	Line2 L1 = Line2::Through(Vector2d(0,0), Vector2d(4,0));
	Line2 L2 = Line2::Through(Vector2d(0,3), Vector2d(4,3));

	Vector2d ints1 = AB.intersection(L1);

	cout << ints1 << endl;

	return 0;
}
