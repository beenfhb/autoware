/*
 * MeidaiBag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/filesystem.hpp>

#include <exception>
#include <algorithm>
#include <fstream>
#include <string>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "BaseFrame.h"
#include "datasets/MeidaiBagDataset.h"
#include "PclNdtLocalizer.h"


using namespace std;
using namespace Eigen;

namespace bfs = boost::filesystem;

string MeidaiBagDataset::dSetName = "Nagoya University";

#define _DashboardMask "conf/meidai_mask.png"
#define _ExposureAdjustmentMask "conf/meidai_exposure_adjust.png"
#define _GroundPlanePatchMask "conf/meidai_ground_plane_patch.png"


const string
	meidaiBagImageTopic = "/camera1/image_raw",
	meidaiBagGnssTopic  = "/nmea_sentence",
	meidaiBagVelodyne   = "/velodyne_packets";


const TTransform defaultLidarToCameraTransform =
	TTransform::from_XYZ_RPY(
		Vector3d(0.9, 0.3, -0.6),
		-1.520777, -0.015, -1.5488);

const TTransform defaultGpsToCameraTransform
	(2.1003, 0.3004, 1.3996,		// Translation
	-0.496, 0.478, -0.510, 0.514);	// Quaternion

const TTransform defaultLidarToBaselinkTransform =
	TTransform::from_Pos_Quat(
		Vector3d(1.2, 0, 2.0),
		Quaterniond::Identity());


class cache_error : public runtime_error
{};


MeidaiBagDataset::MeidaiBagDataset(

	const string &path,
	bool loadPositions) :

		bagPath(path),
		bagfd(new rosbag::Bag(path))
{
	// Directory of VMML package
	bfs::path myPath = getMyPath();

	// try to load mask image
	auto mask1Path = myPath / _DashboardMask;
	auto mask2Path = myPath / _ExposureAdjustmentMask;
	dashBoardMask = cv::imread(mask1Path.string(), cv::IMREAD_GRAYSCALE);
/*
	if (dashBoardMask.empty())
		throw runtime_error("Unable to load Meidai car image");
*/
	exposureMask = cv::imread(mask2Path.string(), cv::IMREAD_GRAYSCALE);
/*
	if (exposureMask.empty())
		throw runtime_error("Unable to load Meidai exposure mask");
*/
	auto mask3Path = myPath / _GroundPlanePatchMask;
	groundPlanePatch = cv::imread(mask3Path.string(), cv::IMREAD_GRAYSCALE);

	mPreprocessor.setMode(ImagePreprocessor::ProcessMode::AGC);
	mPreprocessor.setMask(exposureMask);

	velodyneCalibrationFilePath = (myPath / "params/meidai-64e-S2.yaml").string();

	prepareBag();

	if (loadPositions==true)
		loadCache();
}


MeidaiBagDataset::Ptr
MeidaiBagDataset::load (
		const string &path,
		bool loadPositions
)
{
	MeidaiBagDataset::Ptr nuDatasetMem(new MeidaiBagDataset(
		path,
		loadPositions));
	return nuDatasetMem;
}


MeidaiBagDataset::MeidaiBagDataset
(const MeidaiBagDataset &cp):
	bagPath(cp.bagPath),
	bagfd(cp.bagfd),
	cameraParams(cp.cameraParams),
	zoomRatio(cp.zoomRatio),
	dashBoardMask(cp.dashBoardMask),
	exposureMask(cp.exposureMask),
	mPreprocessor(cp.mPreprocessor)
{}


void
MeidaiBagDataset::prepareBag (const ros::Time &beginTime, const ros::Time &stopTime)
{
	cameraRawBag = RandomAccessBag::Ptr(new RandomAccessBag(*bagfd, meidaiBagImageTopic));
	numOfFrames = cameraRawBag->size();

	gnssBag = RandomAccessBag::Ptr(new RandomAccessBag(*bagfd, meidaiBagGnssTopic));
//	velodyneBag = RandomAccessBag::Ptr(new RandomAccessBag(*bagfd, meidaiBagVelodyne));

	ros::Time
		tstart = ros::TIME_MIN,
		tstop = ros::TIME_MAX;

	velodyneBag = LidarScanBag::Ptr (new LidarScanBag(*bagfd, meidaiBagVelodyne, tstart, tstop, velodyneCalibrationFilePath));
}


void
MeidaiBagDataset::setTimeConstraint(const ptime &start, const ptime &stop)
{
//	cameraRawBag->
}


void
MeidaiBagDataset::loadPosition()
{
	loadCache();
}


MeidaiBagDataset::~MeidaiBagDataset()
{
}


size_t
MeidaiBagDataset::size() const
{
	return cameraRawBag->size();
}


size_t
MeidaiBagDataset::sizeAll() const
{
	RandomAccessBag bg(*bagfd, meidaiBagImageTopic);
	return bg.size();
}


CameraPinholeParams
MeidaiBagDataset::getCameraParameter()
const
{
	return cameraParams * zoomRatio;
}


void
MeidaiBagDataset::addCameraParameter(const CameraPinholeParams &c)
{ cameraParams = c; }


cv::Mat
MeidaiBagDataset::getMask()
{
	cv::Mat imgrs;
	if (zoomRatio==1.0)
		return dashBoardMask.clone();
	else {
		cv::resize(dashBoardMask, imgrs, cv::Size(), zoomRatio, zoomRatio, cv::INTER_CUBIC);
		return imgrs;
	}
}


cv::Mat
MeidaiBagDataset::getGroundPlaneMask() const
{
	cv::Mat groundPatchM;
	cv::resize(groundPlanePatch, groundPatchM, cv::Size(), zoomRatio, zoomRatio, cv::INTER_CUBIC);
	return groundPatchM;
}


MeidaiDataItem::Ptr
MeidaiBagDataset::getNative(dataItemId i) const
{
	MeidaiDataItem::Ptr Id(new MeidaiDataItem(*this, i));
	return Id;
}


GenericDataItem::ConstPtr
MeidaiBagDataset::get(dataItemId i)
const
{
/*
	MeidaiDataItem::ConstPtr dp(new MeidaiDataItem(*this, i));
	return dp;
*/
	return getNative(i);
}


float
MeidaiBagDataset::getZoomRatio () const
{ return zoomRatio; }


void
MeidaiBagDataset::loadCache()
{
	bfs::path bagCachePath = bagPath;
	bagCachePath += ".cache";
	bool isCacheValid = false;

	// Check if cache is valid
	if (bfs::exists(bagCachePath) and bfs::is_regular_file(bagCachePath)) {
		auto lastWriteTime = bfs::last_write_time(bagCachePath),
			lastBagModifyTime = bfs::last_write_time(bagPath);
		if (lastWriteTime >= lastBagModifyTime)
			isCacheValid = true;
	}

	if (isCacheValid) {
		doLoadCache(bagCachePath.string());
	}
}


void
MeidaiBagDataset::setLidarParameters (
	const std::string &pvelodyneCalibrationFile,
	const std::string &pmeidaiPCDMapFile,
	const TTransform &plidarToCameraTransform)
{
	lidarToCameraTransform = plidarToCameraTransform;
	pcdMapFilePath = pmeidaiPCDMapFile;
//	velodyneCalibrationFilePath = pvelodyneCalibrationFile;
}


void
MeidaiBagDataset::forceCreateCache (bool useLidar, const double startOffset, const double stopOffset)
{
	ptime
		t1 = (startOffset==-1 ? MIN_TIME : cameraRawBag->timeFromStart(startOffset).toBoost()),
		t2 = (stopOffset==-1 ? MAX_TIME : cameraRawBag->timeFromStart(stopOffset).toBoost());
	return forceCreateCache(useLidar, t1, t2);
}


void
MeidaiBagDataset::forceCreateCache (bool useLidar, const ptime &t1, const ptime &t2)
{
	bfs::path bagCachePath = bagPath;
	bagCachePath += ".cache";

	gnssTrack.clear();
	ndtTrack.clear();
	cameraTrack.clear();

	createTrajectories(t1, t2, useLidar);
	writeCache(bagCachePath.string());
}


void
MeidaiBagDataset::doLoadCache(const string &path)
{
	fstream cacheFd;
	cacheFd.open(path.c_str(), fstream::in);
	if (!cacheFd.is_open())
		throw runtime_error(string("Unable to open cache file: ") + path);

	boost::archive::binary_iarchive cacheIArc (cacheFd);

	cacheIArc >> gnssTrack;
	cacheIArc >> ndtTrack;
	cacheIArc >> cameraTrack;

	cacheIArc >> cameraTrackSource;

	cacheFd.close();
}


void
MeidaiBagDataset::createTrajectories(ptime startTimep, ptime stopTimep, bool useLidar)
{
	Trajectory *trajectorySrc;
	TTransform srcMultiplier = TTransform::Identity();
	bool doCompensateTime = false;

	ros::Time
		startTime = (startTimep==MIN_TIME ? ros::TIME_MIN : ros::Time::fromBoost(startTimep)),
		stopTime = (stopTimep==MAX_TIME ? ros::TIME_MAX : ros::Time::fromBoost(stopTimep));

	if (startTime!=ros::TIME_MIN and stopTime!=ros::TIME_MAX) {
		ros::Duration td(0.25);
		doCompensateTime = true;
		startTime -= td;
		stopTime += td;
	}

	/*
	 * Create full-bag trajectory for GNSS. It will be used as fallback
	 * when LIDAR localization fails
	 */
	cout << "Creating GNSS Trajectory\n";
	createTrajectoryFromGnssBag(*gnssBag, gnssTrack);

	/*
	 * As of December 2018, NDT localization still could fail undetected (unless it drifts out-of-the-map)
	 */
	if (useLidar==true) {
		cout << "Creating NDT Trajectory\n";
//		auto lidarBag = getLidarScanBag();

		if (doCompensateTime)
			velodyneBag->setTimeConstraint(startTime, stopTime);

		createTrajectoryFromPclNdt(*velodyneBag, ndtTrack, gnssTrack, pcdMapFilePath);
		trajectorySrc = &ndtTrack;
		cameraTrackSource = CameraTrackSource::NDT;
		// XXX: Check this value
		srcMultiplier = lidarToCameraTransform;
	}

	else {
		trajectorySrc = &gnssTrack;
		cameraTrackSource = CameraTrackSource::GNSS;
		// XXX: Find transformation from GNSS to camera
		srcMultiplier = defaultGpsToCameraTransform;
	}

	cout << "Creating Camera Trajectory\n";
	// XXX: It is possible that camera recording may have started earlier than lidar's

	uint32_t imageBagPos;
	if (doCompensateTime) {
		imageBagPos = cameraRawBag->getPositionAtTime(startTime);
		cameraRawBag->setTimeConstraint(startTime, stopTime);
	}
	else {
		imageBagPos = 0;
	}

	for (int i=0; i<cameraRawBag->size(); i++) {
		auto tm = cameraRawBag->timeAt(i).toBoost();

		// in both cases; extrapolate
		PoseStamped poseX;
		if (tm < trajectorySrc->at(0).timestamp or tm>=trajectorySrc->back().timestamp) {
			poseX = trajectorySrc->extrapolate(tm);
		}
		else
			poseX = trajectorySrc->interpolate(tm);

		PoseStamped poseX1 = poseX * srcMultiplier;
		cameraTrack[imageBagPos] = poseX1;

		cout << i+1 << " / " << cameraRawBag->size() << "  \r";
		imageBagPos += 1;
	}

	cameraRawBag->resetTimeConstraint();
	velodyneBag->resetTimeConstraint();
}


void MeidaiBagDataset::writeCache(const string &path)
{
	fstream cacheFd;
	cacheFd.open(path.c_str(), fstream::out);
	if (!cacheFd.is_open())
		throw runtime_error(string("Unable to open cache file: ") + path);

	boost::archive::binary_oarchive cacheOArc (cacheFd);

	cacheOArc << gnssTrack;
	cacheOArc << ndtTrack;
	cacheOArc << cameraTrack;

	cacheOArc << cameraTrackSource;

	cacheFd.close();
}


GenericDataItem::ConstPtr
MeidaiBagDataset::atDurationSecond (const double second)
const
{
	uint32_t pos = cameraRawBag->getPositionAtDurationSecond(second);
	return get(pos);
}


dataItemId
MeidaiBagDataset::getLowerBound (const ptime &t) const
{
	auto rt = ros::Time::fromBoost(t);
	return (dataItemId)cameraRawBag->getPositionAtTime(rt);
}



LidarScanBag::Ptr
MeidaiBagDataset::getLidarScanBag ()
{
	return velodyneBag;
}


cv::Mat
MeidaiBagDataset::computeImagePreprocessorMask()
const
{
	// XXX: Stub
}


Trajectory
MeidaiBagDataset::getCameraTrajectory(const ptime timeStart, const ptime timeStop) const
{
	// XXX: Stub
	return cameraTrack.toTrajectory();
}


double normalizeAngle(const double &r)
{
	if (r > 2*M_PI) {

	}

	else if (r<0) {

	}

	else return r;
}


void
MeidaiDataItem::init()
{
	bImageMsg = parent.cameraRawBag->at<sensor_msgs::Image>(pId);
}


Pose
MeidaiDataItem::getPose()
const
{
	auto &p = parent.cameraTrack.at(pId);
	return p;
}


Pose
MeidaiDataItem::getBaselinkPose() const
{
//	return getVelodynePose() * defaultLidarToBaselinkTransform;
	return getPose() * defaultLidarToCameraTransform.inverse() * defaultLidarToBaselinkTransform;
}

Pose
MeidaiDataItem::getVelodynePose() const
{
//	return parent.ndtTrack.interpolate(getTimestamp());
	return getPose() * defaultLidarToCameraTransform.inverse();
}


Vector3d
MeidaiDataItem::getPosition() const
{
	auto &p = parent.cameraTrack.at(pId);
	return p.position();
}


Quaterniond
MeidaiDataItem::getOrientation() const
{
	auto &p = parent.cameraTrack.at(pId);
	return p.orientation();
}


cv::Mat
MeidaiDataItem::getImage() const
{
	// Convert image to RGB colorspace (originally in RGGB)
	auto imgPtr = cv_bridge::toCvCopy(bImageMsg, sensor_msgs::image_encodings::BGR8);

	if (parent.isPreprocessed) {
		parent.mPreprocessor.preprocess(imgPtr->image);
	}

	cv::Mat imgrs;
	if (parent.zoomRatio==1.0)
		return imgPtr->image;
	else {
		cv::resize(imgPtr->image, imgrs, cv::Size(), parent.zoomRatio, parent.zoomRatio, cv::INTER_CUBIC);
	}

	return imgrs;
}


ptime
MeidaiDataItem::getTimestamp() const
{
//	return bImageMsg->header.stamp.toBoost();
	auto ts = parent.cameraRawBag->timeAt(pId);
	return ts.toBoost();
}


LidarScanBag::scan_t::ConstPtr
MeidaiDataItem::getLidarScan(ptime *scanTime)
{
	auto p = parent.velodyneBag->getPositionAtTime(ros::Time::fromBoost(getTimestamp()));
	return parent.velodyneBag->at(p, scanTime);
}


bool
MeidaiBagDataset::isCameraTrajectoryComplete() const
{
	if (cameraTrack.empty())
		return false;

	dataItemId firstp = cameraTrack.begin()->first;
	dataItemId lastp = cameraTrack.rbegin()->first;
	if (cameraTrack.at(firstp).timestamp > cameraRawBag->startTime().toBoost() or
		cameraTrack.at(lastp).timestamp < cameraRawBag->stopTime().toBoost())
	return false;

	return true;
}


Trajectory
MeidaiTrajectoryMap::toTrajectory() const
{
	Trajectory vTrack;

	for (auto itp: *this) {
		vTrack.push_back(itp.second);
	}

	return vTrack;
}


const Trajectory
MeidaiBagDataset::getCompleteCameraTrajectory() const
{ return cameraTrack.toTrajectory(); }


