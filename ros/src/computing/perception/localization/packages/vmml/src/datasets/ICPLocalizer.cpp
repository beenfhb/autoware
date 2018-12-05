/*
 * ICPLocalizer.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: sujiwo
 */

#include <pcl/io/pcd_io.h>
#include "src/datasets/ICPLocalizer.h"


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

}


/*
 * We wish the input to be filtered before entering this function
 */
Pose
ICPLocalizer::localize (pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan)
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
	Pose localizerPose (_localizerPose);

	// Calculate errors
	double predictPoseError = (localizerPose.position() - predictPose.position()).norm();
	if (predictPoseError <= PREDICT_POSE_THRESHOLD) {

	}

	// Calculate velocity and acceleration

	return Pose::Identity();
}
