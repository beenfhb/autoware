/*
 * LidarScanBag.cpp
 *
 *  Created on: Nov 19, 2018
 *      Author: sujiwo
 */


#include "datasets/LidarScanBag.h"


using namespace std;
using velodyne_rawdata::VPoint;
using velodyne_rawdata::VPointCloud;
using pcl::PointCloud;
//using pcl::PointXYZ;
//using LidarScanBag::scan_t;

/*
 * XXX: These values may need to be adjusted
 */
const float
	velodyneMinRange = 2.0,
	velodyneMaxRange = 130,
	velodyneViewDirection = 0,
	velodyneViewWidth = 2*M_PI;


template<class PointT>
static
LidarScanBag::scan_t::Ptr
convertToExternal (const PointCloud<PointT> &cloudSrc)
{
	const int w=cloudSrc.width, h=cloudSrc.height;
	LidarScanBag::scan_t::Ptr cloudExt (new LidarScanBag::scan_t(w*h, 1));

	if (h==1) for (int i=0; i<w; ++i) {
		cloudExt->at(i).x = cloudSrc.at(i).x;
		cloudExt->at(i).y = cloudSrc.at(i).y;
		cloudExt->at(i).z = cloudSrc.at(i).z;
	}

	else for (int i=0; i<w; ++i) {
		for (int j=0; j<h; ++j) {
			cloudExt->at(i*w + j).x = cloudSrc.at(j, i).x;
			cloudExt->at(i*w + j).y = cloudSrc.at(j, i).y;
			cloudExt->at(i*w + j).z = cloudSrc.at(j, i).z;
		}
	}

	return cloudExt;
}


LidarScanBag::LidarScanBag(
	rosbag::Bag const &bag,
	const std::string &topic,
	const std::string &lidarCalibFile,
	const ros::Time &startTime,
	const ros::Time &endTime) :

		RandomAccessBag(bag, topic, startTime, endTime),
		data_(new velodyne_rawdata::RawData())
{
	prepare(lidarCalibFile);
}


LidarScanBag::LidarScanBag(
	rosbag::Bag const &bag, const std::string &topic,
	const std::string &lidarCalibFile,
	const double seconds1FromOffset,
	const double seconds2FromOffset) :

		RandomAccessBag(bag, topic, seconds1FromOffset, seconds2FromOffset),
		data_(new velodyne_rawdata::RawData())
{
	prepare(lidarCalibFile);
}


void
LidarScanBag::prepare(const string &lidarCalibFile)
{
	if (data_->setupOffline(lidarCalibFile, velodyneMaxRange, velodyneMinRange)
		== -1)
		throw runtime_error("Unable to set velodyne converter");

	data_->setParameters(velodyneMinRange, velodyneMaxRange, velodyneViewDirection, velodyneViewWidth);
}



LidarScanBag
LidarScanBag::subset(const ros::Time &start, ros::Duration &d) const
{

}


LidarScanBag::scan_t::ConstPtr
LidarScanBag::convertMessage(velodyne_msgs::VelodyneScan::ConstPtr bagmsg)
{
	VPointCloud::Ptr outPoints(new VPointCloud);
	outPoints->header.stamp = pcl_conversions::toPCL(bagmsg->header).stamp;
	outPoints->header.frame_id = bagmsg->header.frame_id;
	outPoints->height = 1;

	for (int i=0; i<bagmsg->packets.size(); ++i) {
		data_->unpack(bagmsg->packets[i], *outPoints, bagmsg->packets.size());
	}

	scan_t::Ptr cloudTmp = convertToExternal(*outPoints);
	if (filtered)
	// These are the best value I know of
		return VoxelGridFilter(cloudTmp, 0.2, 3.0);
	else
		return cloudTmp;
}


LidarScanBag::scan_t::ConstPtr
LidarScanBag::at (int position, ptime *msgTime)
{
	auto msgP = RandomAccessBag::at<velodyne_msgs::VelodyneScan>(position);
	if (msgTime!=nullptr)
		*msgTime = msgP->header.stamp.toBoost();
	return convertMessage(msgP);
}


LidarScanBag::scan_t::ConstPtr
LidarScanBag::VoxelGridFilter (
	LidarScanBag::scan_t::ConstPtr vcloud,
	double voxel_leaf_size,
	double measurement_range)
{
	scan_t::Ptr filteredGridCLoud(new scan_t);

	assert(voxel_leaf_size>=0.1);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
	voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
	voxel_grid_filter.setInputCloud(vcloud);
	voxel_grid_filter.filter(*filteredGridCLoud);

	return filteredGridCLoud;
}


#include <pcl/io/pcd_io.h>

bool
LidarScanBag::save(LidarScanBag::scan_t::ConstPtr pclPtr, const std::string &filename)
{
	return pcl::io::savePCDFileBinary(filename, *pclPtr);
}
