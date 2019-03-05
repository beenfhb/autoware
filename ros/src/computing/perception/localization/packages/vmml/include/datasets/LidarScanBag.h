/*
 * LidarScanBag.h
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */

#ifndef _LIDARSCANBAG_H
#define _LIDARSCANBAG_H

#include <string>
#include <exception>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/rawdata.h>

//#include "utilities.h"
#include "RandomAccessBag.h"


/*
 * Representation of Rosbag Velodyne Scan as point cloud
 */
class LidarScanBag : public RandomAccessBag
{
public:

	typedef std::shared_ptr<LidarScanBag> Ptr;
	typedef pcl::PointXYZ point3_t;
	typedef pcl::PointCloud<point3_t> scan_t;

	LidarScanBag(
		rosbag::Bag const &bag, const std::string &topic,
		const ros::Time &startTime = ros::TIME_MIN,
		const ros::Time &endTime = ros::TIME_MAX,
		const std::string &velodyneCalibrationFile=std::string());

	LidarScanBag(
		rosbag::Bag const &bag, const std::string &topic,
		const double seconds1FromOffset,
		const double seconds2FromOffset,
		const std::string &velodyneCalibrationFile=std::string());

	LidarScanBag subset(const ros::Time &start, ros::Duration &d) const;

	scan_t::ConstPtr
	at (int position, boost::posix_time::ptime *msgTime=nullptr);

	inline
	scan_t::ConstPtr
	atDurationSecond (const double S)
	{ return at (getPositionAtDurationSecond(S)); }

	static
	bool save(scan_t::ConstPtr, const std::string &filename);

	bool filtered = false;

	static
	scan_t::ConstPtr
	VoxelGridFilter (
		scan_t::ConstPtr vcloud,
		double voxel_leaf_size=0.2,
		double measurement_range=3.0);

protected:
	boost::shared_ptr<velodyne_rawdata::RawData> data_;

	scan_t::ConstPtr
	convertMessage(velodyne_msgs::VelodyneScan::ConstPtr bagmsg);

	void prepare(const std::string &lidarCalibFile);
};



#endif /* _LIDARSCANBAG_H */
