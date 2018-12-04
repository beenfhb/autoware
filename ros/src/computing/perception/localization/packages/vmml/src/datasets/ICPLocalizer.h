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
	ICPLocalizer();
	virtual ~ICPLocalizer();

	void loadMap (const std::string &filename);

	void putEstimation (const Pose &pEst);

	Pose localize (pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan);

protected:

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> mIcp;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcMap = nullptr;
};

#endif /* _ICPLOCALIZER_H_ */
