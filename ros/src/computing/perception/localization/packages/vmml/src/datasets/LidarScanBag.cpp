/*
 * LidarScanBag.cpp
 *
 *  Created on: Nov 19, 2018
 *      Author: sujiwo
 */


#include "datasets/LidarScanBag.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>


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


class mPointcloudXYZIR : public velodyne_rawdata::DataContainerBase
{
public:
	velodyne_rawdata::VPointCloud::Ptr pc;

	mPointcloudXYZIR() : pc(new velodyne_rawdata::VPointCloud) {}

	virtual void
	addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth, const float& distance, const float& intensity)
	{
		// convert polar coordinates to Euclidean XYZ
		velodyne_rawdata::VPoint point;
		point.ring = ring;
		point.x = x;
		point.y = y;
		point.z = z;
		point.intensity = intensity;

		// append this point to the cloud
		pc->points.push_back(point);
		++pc->width;
	}
};




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


/*
string getVelodynePointCloudcalibrationFile()
{
	boost::filesystem::path vlp (ros::package::getPath("velodyne_pointcloud"));
	vlp /= "params/64e_s2.1-sztaki.yaml";
	return "/tmp/64e-S2.y"
	return vlp.string();
}
*/


LidarScanBag::LidarScanBag(
	rosbag::Bag const &bag,
	const std::string &topic,
	const ros::Time &startTime,
	const ros::Time &endTime,
	const std::string &velodyneCalibrationFile) :

		RandomAccessBag(bag, topic, startTime, endTime),
		data_(new velodyne_rawdata::RawData())
{
	prepare(velodyneCalibrationFile);
}


LidarScanBag::LidarScanBag(
	rosbag::Bag const &bag,
	const std::string &topic,
	const double seconds1FromOffset,
	const double seconds2FromOffset,
	const std::string &velodyneCalibrationFile) :

		RandomAccessBag(bag, topic, seconds1FromOffset, seconds2FromOffset),
		data_(new velodyne_rawdata::RawData())
{
	prepare(velodyneCalibrationFile);
}


void
LidarScanBag::prepare(const string &lidarCalibFile)
{
	string fname;
	if (lidarCalibFile.empty()) {
		boost::filesystem::path vlp (ros::package::getPath("velodyne_pointcloud"));
		vlp /= "params/64e_s2.1-sztaki.yaml";
		fname = vlp.string();
		cerr << "Bug: using default Velodyne Calibration Parameter" << endl;
	}
	else fname = lidarCalibFile;

	if (data_->setupOffline(fname, velodyneMaxRange, velodyneMinRange)
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
	mPointcloudXYZIR outPoints;
	outPoints.pc->header.stamp = pcl_conversions::toPCL(bagmsg->header).stamp;
	outPoints.pc->header.frame_id = bagmsg->header.frame_id;
	outPoints.pc->height = 1;

	outPoints.pc->points.reserve(bagmsg->packets.size() * data_->scansPerPacket());

	for (int i=0; i<bagmsg->packets.size(); ++i) {
		data_->unpack(bagmsg->packets[i], outPoints);
	}

	scan_t::Ptr cloudTmp = convertToExternal(*outPoints.pc);

	if (filtered)
	// These are the best value I know of
		return VoxelGridFilter(cloudTmp, 0.2, 3.0);
	else
		return cloudTmp;
}


LidarScanBag::scan_t::ConstPtr
LidarScanBag::at (int position, boost::posix_time::ptime *msgTime)
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
