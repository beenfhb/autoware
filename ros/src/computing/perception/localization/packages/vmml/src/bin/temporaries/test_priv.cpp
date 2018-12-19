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

#include "utilities.h"


using namespace std;
using namespace Eigen;


int main(int argc, char *argv[])
{
	Pose bike(2, 2, 0, 0, 0, 0);
	Pose me = bike.shift(Vector3d(0.5, 0, 0));

	return 0;
}
