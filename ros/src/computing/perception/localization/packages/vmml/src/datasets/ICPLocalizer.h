/*
 * ICPLocalizer.h
 *
 *  Created on: Dec 4, 2018
 *      Author: sujiwo
 */

#ifndef _ICPLOCALIZER_H_
#define _ICPLOCALIZER_H_


#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "utilities.h"
#include "datasets/MeidaiBagDataset.h"


class ICPLocalizer
{
public:

	ICPLocalizer();
	virtual ~ICPLocalizer();

	void loadMap (const std::string &filename);

	void putEstimation (const Pose &pEst);

	Pose localize (pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan, ptime curTime=getCurrentTime());

	struct Parameters
	{
		int maximum_iterations = 50;
		double transformation_epsilon = 0.01;
		double max_correspondence_distance = 1.0;
		double euclidean_fitness_epsilon = 0.1;
		double ransac_outlier_rejection_threshold = 1.0;
		Pose initialGuess = Pose::Identity();

		enum {
			Linear, Quadratic, Zero
		} offsetMode;

	} mParams;


protected:

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> mIcp;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcMap = nullptr;

	Pose current_pose;

	Vector3d current_velocity, current_accel;

	ptime lastLocalizationTime;

	Eigen::Vector3d offset = Eigen::Vector3d::Zero();
};


void
createTrajectoryFromICP (
	LidarScanBag &bagsrc,
	Trajectory &resultTrack,
	const Trajectory &gnssTrack,
	const std::string &pcdMapFile);

#endif /* _ICPLOCALIZER_H_ */
