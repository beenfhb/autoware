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


PclNdtLocalizer::PclNdtLocalizer() {
}

PclNdtLocalizer::~PclNdtLocalizer() {
}


void
PclNdtLocalizer::loadMap (const std::string &filename)
{
	pcMap = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader fReader;
	fReader.read(filename, *pcMap);
	mNdt.setInputTarget(pcMap);
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
PclNdtLocalizer::localize(pcl::PointCloud<pcl::PointXYZ>::ConstPtr currentScan, ptime curTime)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	mNdt.setMaximumIterations(mParams.maximum_iterations);
	mNdt.setTransformationEpsilon(mParams.transformation_epsilon);
	mNdt.setStepSize(mParams.step_size);
	mNdt.setResolution(mParams.ndt_resolution);
	mNdt.setInputSource(currentScan);

	if (!started) {
		mNdt.align(*output_cloud, currentEstimation.matrix().cast<float>());
		Pose localizerPose = mNdt.getFinalTransformation().cast<double>();
		current_pose = localizerPose;
		prev_pose = current_pose;

		started = true;
		lastLocalizationTime = curTime;
		estimationReset = false;
		return localizerPose;
	}

	else {
		double dt = toSeconds(curTime - lastLocalizationTime);
		prev_pose = current_pose;

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
		return current_pose;
	}
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

		auto cscan = bagsrc.at(ip);
		auto scanTime = bagsrc.timeAt(ip).toBoost();

		if (!lidarLocalizer.isStarted()) {
			try {
				gnssPose = gnssTrack.at(scanTime);
			} catch (std::out_of_range &e) {
				gnssPose = gnssTrack.extrapolate(scanTime);
			}
			lidarLocalizer.putEstimation(gnssPose);
			cNdtPose = lidarLocalizer.localize(cscan);
		}

		else {
			cNdtPose = lidarLocalizer.localize(cscan);
		}

		PoseStamped tpose(cNdtPose, scanTime);
		resultTrack.push_back(tpose);
	}
}
