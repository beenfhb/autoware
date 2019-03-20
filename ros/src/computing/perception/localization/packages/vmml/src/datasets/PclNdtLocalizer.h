/*
 * PclNdtLocalizer.h
 *
 *  Created on: Mar 13, 2019
 *      Author: sujiwo
 */

#ifndef _PCLNDTLOCALIZER_H_
#define _PCLNDTLOCALIZER_H_

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_omp_registration/ndt.h>

#include "utilities.h"
#include "datasets/MeidaiBagDataset.h"


class PclNdtLocalizer {
public:
	PclNdtLocalizer();
	virtual ~PclNdtLocalizer();

	void loadMap (const std::string &filename);

	void putEstimation (const Pose &pEst);

	Pose localize (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scan, ptime curTime);

	struct Parameters
	{
		int maximum_iterations = 30;
		float ndt_resolution = 1.0;
		double step_size = 0.1;
		double transformation_epsilon = 0.01;
		double max_correspondence_distance = 1.0;
		double euclidean_fitness_epsilon = 0.1;
		double ransac_outlier_rejection_threshold = 1.0;
		Pose initialGuess = Pose::Identity();

		enum {
			Linear, Quadratic, Zero
		} offsetMode;

	} mParams;

	bool isStarted() const
	{ return started; }

protected:

	pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> mNdt;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcMap = nullptr;

	Pose prev_pose, prev_pose2, currentEstimation;

	bool started = false;
	bool estimationReset = false;
	bool velocityIsSet = false;

/*
	Eigen::Vector3d
		current_velocity=Eigen::Vector3d::Zero(),
		current_accel=Eigen::Vector3d::Zero();
*/
	TTransform current_velocity;

	ptime lastLocalizationTime;

	Eigen::Vector3d offset = Eigen::Vector3d::Zero();

};


void
createTrajectoryFromPclNdt
(LidarScanBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack, const std::string &pcdMapFile);


#endif /* _PCLNDTLOCALIZER_H_ */
