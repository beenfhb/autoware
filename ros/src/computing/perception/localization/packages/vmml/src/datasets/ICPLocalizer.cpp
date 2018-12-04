/*
 * ICPLocalizer.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: sujiwo
 */

#include <pcl/io/pcd_io.h>
#include "src/datasets/ICPLocalizer.h"


using namespace Eigen;


ICPLocalizer::ICPLocalizer()
{
	// TODO Auto-generated constructor stub
}


ICPLocalizer::~ICPLocalizer()
{
	// TODO Auto-generated destructor stub
}


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

	Translation3f init_translation();
	AngleAxisf init_rotation_x;
	AngleAxisf init_rotation_y;
	AngleAxisf init_rotation_z;

	return Pose::Identity();
}
