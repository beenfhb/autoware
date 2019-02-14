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
	Matrix<double, 4, 2> X;
	X << 	0.3501104 , 0.02194434,
			0.15401266, 0.87935743,
			0.47155552, 0.67694619,
			0.34460845, 0.17656619;

	Matrix<double,2,4> X1 = pseudoInverse(X);
	cout << X1 << endl;

	auto I = X1 * X;
	cout << I << endl;
}
