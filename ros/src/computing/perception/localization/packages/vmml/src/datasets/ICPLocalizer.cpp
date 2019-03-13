/*
 * ICPLocalizer.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: sujiwo
 */

#include <iostream>

#include <pcl/io/pcd_io.h>
#include "ICPLocalizer.h"

using namespace std;
using namespace Eigen;


#define PREDICT_POSE_THRESHOLD 0.5


ICPLocalizer::ICPLocalizer()
{}


ICPLocalizer::~ICPLocalizer()
{}


void
ICPLocalizer::loadMap (const std::string &pcdFilename)
{
	pcMap = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader fReader;
	fReader.read(pcdFilename, *pcMap);
	mIcp.setInputTarget(pcMap);
}


void
ICPLocalizer::putEstimation (const Pose &pEst)
{
	current_pose = pEst;
}


/*
 * We wish the input to be filtered before entering this function
 */
Pose
ICPLocalizer::localize (pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan, ptime curTime)
{
	mIcp.setInputSource(scan);

	Pose predictPose = current_pose;
	predictPose.translation() += offset;

	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	mIcp.setMaximumIterations(mParams.maximum_iterations);
	mIcp.setTransformationEpsilon(mParams.transformation_epsilon);
	mIcp.setMaxCorrespondenceDistance(mParams.max_correspondence_distance);
	mIcp.setEuclideanFitnessEpsilon(mParams.euclidean_fitness_epsilon);
	mIcp.setRANSACOutlierRejectionThreshold(mParams.ransac_outlier_rejection_threshold);

	// XXX: Benchmark here ?
	mIcp.align(*output_cloud, predictPose.matrix().cast<float>());
	auto _localizerPose = mIcp.getFinalTransformation();
	Pose localizerPose = _localizerPose.cast<double>();

	double fitnessScore = mIcp.getFitnessScore();

	// Calculate errors
	double predictPoseError = (localizerPose.position() - predictPose.position()).norm();
	if (predictPoseError <= PREDICT_POSE_THRESHOLD) {

	}

	// Calculate velocity and acceleration
	Vector3d diffPos;

	// Update offset
	if (mParams.offsetMode==Parameters::Linear) {
		offset = diffPos;
	}

	else if (mParams.offsetMode==Parameters::Quadratic) {

	}

	else if (mParams.offsetMode==Parameters::Zero) {
		offset << 0, 0, 0;
	}

	return localizerPose;
}


void
createTrajectoryFromICP (
	LidarScanBag &bagsrc,
	Trajectory &resultTrack,
	const Trajectory &gnssTrack,
	const std::string &pcdMapFile)
{
	bagsrc.filtered = true;
	ICPLocalizer lidarLocalizer;
	lidarLocalizer.loadMap(pcdMapFile);
	resultTrack.clear();

	bool initialized=false;

	uint32_t N = bagsrc.size();
	for (uint32_t ip=0; ip<N; ++ip) {

		Pose cIcpPose;
		auto cscan = bagsrc.at(ip);
		auto scanTime = bagsrc.timeAt(ip).toBoost();
		cout << ip+1 << " / " << N << "   \r" << flush;

		if (!initialized) {
			auto cGnssPos = gnssTrack.at(scanTime);
//			if (lidarLocalizer.isPointInsideMap(cGnssPos.position())==false)
//				throw 1;

			lidarLocalizer.putEstimation(cGnssPos);
			cIcpPose = lidarLocalizer.localize(cscan);
			initialized = true;

		}

		else {
			cIcpPose = lidarLocalizer.localize(cscan);
//			if (lidarLocalizer.isPointInsideMap(cNdtPose.position())==false)
//				throw 2;
		}

		PoseStamped tpose (cIcpPose, scanTime);
		resultTrack.push_back(tpose);

	}

	cout << "Finished all scan points" << endl;
}



