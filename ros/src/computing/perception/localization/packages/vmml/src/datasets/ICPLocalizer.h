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


class ICPLocalizer
{
public:

	struct Parameters
	{
		int maximum_iterations = 100;
		double transformation_epsilon = 0.01;
		double max_correspondence_distance = 1.0;
		double euclidean_fitness_epsilon = 0.1;
		double ransac_outlier_rejection_threshold = 1.0;
		Pose initialGuess = Pose::Identity();
	};

	ICPLocalizer();
	virtual ~ICPLocalizer();

	void loadMap (const std::string &filename);

	void putEstimation (const Pose &pEst);

	Pose localize (pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan);

	Parameters mParams;

protected:

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> mIcp;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcMap = nullptr;

	Pose current_pose;

	Eigen::Vector3d offset = Eigen::Vector3d::Zero();
};

#endif /* _ICPLOCALIZER_H_ */
