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
	currentEstimation = est;
	if (started==false) {
		prev_pose = currentEstimation;
		current_pose = currentEstimation;
	}
	estimationReset = true;
}


Pose
PclNdtLocalizer::localize2(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &currentScan, ptime curTime)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	mNdt.setInputSource(currentScan);

	double dt = toSeconds(curTime - lastLocalizationTime);
	prev_pose = current_pose;

	Pose priorEstimation;

	if (estimationReset==true) {
		priorEstimation = currentEstimation;
		estimationReset = false;
	}
	else {
		priorEstimation = prev_pose * (current_velocity * dt);
	}

	mNdt.align(*output_cloud, priorEstimation.matrix().cast<float>());
	current_pose = mNdt.getFinalTransformation().cast<double>();
	current_velocity = prev_pose.inverse() * current_pose;
	current_velocity = current_velocity / dt;

	lastLocalizationTime = curTime;

	if (mNdt.hasConverged()==false)
		cerr << "Not converged\n";

	return current_pose;
}


Pose
PclNdtLocalizer::localize(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &currentScan, ptime curTime)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	mNdt.setInputSource(currentScan);

	double dt = toSeconds(curTime - lastLocalizationTime);
	prev_pose = current_pose;

	if (!started) {
		mNdt.align(*output_cloud, currentEstimation.matrix().cast<float>());
		Pose localizerPose = mNdt.getFinalTransformation().cast<double>();
		current_pose = localizerPose;
		prev_pose = current_pose;

		started = true;
		lastLocalizationTime = curTime;
		estimationReset = false;
	}

	else {

		if (velocityIsSet==false) {
			// Estimate velocity for next scan
			if (estimationReset==true) {
				mNdt.align(*output_cloud, currentEstimation.matrix().cast<float>());
				estimationReset = false;
			}
			else {
				mNdt.align(*output_cloud, current_pose.matrix().cast<float>());
			}
		}

		else {
			if (estimationReset==false)
				currentEstimation = prev_pose * (current_velocity * dt);
			estimationReset = false;
			mNdt.align(*output_cloud, currentEstimation.matrix().cast<float>());
		}

		current_pose = mNdt.getFinalTransformation().cast<double>();
		current_velocity = prev_pose.inverse() * current_pose;
		current_velocity = current_velocity / dt;

		velocityIsSet = true;
		lastLocalizationTime = curTime;
	}

	if (mNdt.hasConverged()==false)
		cerr << "Not converged\n";

	return current_pose;
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

		if (!lidarLocalizer.isStarted()) {
			lidarLocalizer.putEstimation(gnssPose);
		}

		cNdtPose = lidarLocalizer.localize2(cscan, scanTime);

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
