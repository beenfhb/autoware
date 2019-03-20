/*
 * PclNdtLocalizer.cpp
 *
 *  Created on: Mar 13, 2019
 *      Author: sujiwo
 */

#include <iostream>
#include <pcl/io/pcd_io.h>

#include "PclNdtLocalizer.h"


#define PREDICT_POSE_THRESHOLD 0.5


using namespace std;


PclNdtLocalizer::PclNdtLocalizer()
{}

PclNdtLocalizer::~PclNdtLocalizer() {
}


void
PclNdtLocalizer::loadMap (const std::string &filename)
{
	pcMap = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader fReader;
	fReader.read(filename, *pcMap);
	mNdt.setInputTarget(pcMap);

	mNdt.setMaximumIterations(mParams.maximum_iterations);
	mNdt.setTransformationEpsilon(mParams.transformation_epsilon);
	mNdt.setStepSize(mParams.step_size);
	mNdt.setResolution(mParams.ndt_resolution);

	cout << "Map loaded\n";
}


void PclNdtLocalizer::putEstimation(const Pose &est)
{
	prev_pose = est;
	prev_pose2 = prev_pose;
}


Pose
PclNdtLocalizer::localize(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &currentScan, ptime curTime)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	mNdt.setInputSource(currentScan);

	// Calculate offset
	Pose npose_offset = prev_pose2.inverse() * prev_pose;

	// Calculate estimation
	Pose moveEstimate = prev_pose * npose_offset;

	// Registration
	mNdt.align(*output_cloud, moveEstimate.matrix().cast<float>());

	// Localizer output
	Pose cpose = mNdt.getFinalTransformation().cast<double>();
	prev_pose2 = prev_pose;
	prev_pose = cpose;
	lastLocalizationTime = curTime;
	return cpose;
}


void
createTrajectoryFromPclNdt
(LidarScanBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack, const std::string &pcdMapFile)
{
	bagsrc.filtered = true;
	PclNdtLocalizer lidarLocalizer;
	lidarLocalizer.loadMap(pcdMapFile);
	resultTrack.clear();

	int N = bagsrc.size();
	for (int ip=0; ip<N; ++ip) {

		Pose cNdtPose;
		PoseStamped gnssPose;

		ptime scanTime;
		auto cscan = bagsrc.at(ip, &scanTime);

		try {
			gnssPose = gnssTrack.at(scanTime);
		} catch (std::out_of_range &e) {
			gnssPose = gnssTrack.extrapolate(scanTime);
		}

		if (ip==0) {
			lidarLocalizer.putEstimation(gnssPose);
		}

		cNdtPose = lidarLocalizer.localize(cscan, scanTime);

/*
		double drot, dtrans;
		gnssPose.displacement(cNdtPose, dtrans, drot);
		if (dtrans>=15.0) {
			cerr << "\nResetting to GNSS in " << ip << endl;
			exit(1);
			lidarLocalizer.putEstimation(gnssPose);
		}
*/

		PoseStamped tpose(cNdtPose, scanTime);
		resultTrack.push_back(tpose);

		cerr << ip << " / " << N << "\r";
	}
}
