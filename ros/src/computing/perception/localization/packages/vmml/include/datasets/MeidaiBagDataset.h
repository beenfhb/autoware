/*
 * MeidaiBag.h
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */

#ifndef _MEIDAIBAG_H_
#define _MEIDAIBAG_H_


#include <string>
#include <memory>
#include <vector>
#include <array>
#include <stdexcept>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/serialization/map.hpp>

#include "utilities.h"
#include "ImagePreprocessor.h"
#include "Trajectory.h"
#include "datasets/GenericDataset.h"
#include "RandomAccessBag.h"
#include "datasets/LidarScanBag.h"


extern const TTransform defaultLidarToCameraTransform;



class MeidaiBagDataset;

class BaseFrame;

class MeidaiDataItem : public GenericDataItem
{
public:

	MeidaiDataItem (const MeidaiBagDataset &p, dataItemId idx):
		parent(p), pId(idx)
	{ init(); }

	cv::Mat getImage() const;

	Pose getPose() const;

	Pose getBaselinkPose() const;

	Pose getVelodynePose() const;

	Eigen::Vector3d getPosition() const;

	Eigen::Quaterniond getOrientation() const;

	dataItemId getId() const
	{ return pId; }

	ptime getTimestamp() const;

	typedef std::shared_ptr<MeidaiDataItem> Ptr;
	typedef std::shared_ptr<MeidaiDataItem const> ConstPtr;

	LidarScanBag::scan_t::ConstPtr getLidarScan(ptime *scanTime=nullptr);

protected:
	const MeidaiBagDataset &parent;
	dataItemId pId;

	sensor_msgs::Image::ConstPtr bImageMsg;
	void init();
};



class MeidaiTrajectoryMap : public std::map<dataItemId, PoseStamped>
{
friend class boost::serialization::access;
	template<class Archive>
	inline void serialize(Archive &ar, const unsigned int version)
	{ ar & boost::serialization::base_object<std::map<dataItemId, PoseStamped>>(*this);}

public:
	Trajectory toTrajectory() const;
};



class MeidaiBagDataset : public GenericDataset
{
public:

	typedef std::shared_ptr<MeidaiBagDataset> Ptr;
	typedef std::shared_ptr<MeidaiBagDataset const> ConstPtr;

	MeidaiBagDataset(
		const std::string &filePath,
		bool loadPositions=true
	);


	static MeidaiBagDataset::Ptr load (
		const std::string &filePath,
		bool loadPositions=true
	);

	void setTimeConstraint(const ptime &start, const ptime &stop);
	void resetTimeConstraint();

	void loadPosition();

	virtual ~MeidaiBagDataset();

	size_t size() const;

	size_t sizeAll() const;

	void addCameraParameter(const CameraPinholeParams &c);
	CameraPinholeParams getCameraParameter() const;

	cv::Mat getMask();

	std::string getPath() const
	{ return bagfd->getFileName(); }

	MeidaiDataItem::Ptr getNative(dataItemId i) const;

	const Trajectory& getGnssTrajectory() const
	{ return gnssTrack; }

	const Trajectory& getNdtTrajectory() const
	{ return ndtTrack; }

	const Trajectory getCompleteCameraTrajectory() const;

	bool isCameraTrajectoryComplete() const;

	GenericDataItem::ConstPtr get(dataItemId i) const;

	GenericDataItem::ConstPtr atDurationSecond (const double second) const;

	bool hasPositioning() const
	{ return !gnssTrack.empty(); }

	void forceCreateCache (bool useLidar=true, const double startOffset=-1, const double stopOffset=-1);
	void forceCreateCache (bool useLidar=true, const ptime &t1=MIN_TIME, const ptime &t2=MAX_TIME);

	inline void setZoomRatio (float r)
	{ zoomRatio = r; }

	float getZoomRatio () const;

	inline RandomAccessBag::Ptr getVelodyneBag()
	{ return velodyneBag; }

	LidarScanBag::Ptr getLidarScanBag ();

	virtual Trajectory getCameraTrajectory(const ptime timeStart=MIN_TIME, const ptime timeStop=MAX_TIME) const;

	void setLidarParameters (
		const std::string &pvelodyneCalibrationFile,
		const std::string &pmeidaiPCDMapFile,
		const TTransform &plidarToCameraTransform=TTransform::Identity());

	cv::Mat getGroundPlaneMask () const;

	/*
	* Convert time as represented by seconds from start of bag
	*/
	inline ptime timeFromStart(const double seconds) const
	{ return cameraRawBag->timeFromStart(seconds).toBoost(); }

	virtual
	dataItemId getLowerBound (const ptime &t) const;

	cv::Mat computeImagePreprocessorMask() const;

	// Flag to request preprocessed image
	bool isPreprocessed = true;

	enum CameraTrackSource {
		GNSS,
		NDT,
		ICP
	} cameraTrackSource;


	inline float getFullResolutionCoC() const
	{ return 1.74545; }


protected:
	static std::string dSetName;

	// Bag Handler
	std::shared_ptr<rosbag::Bag> bagfd;
	RandomAccessBag::Ptr cameraRawBag;
	RandomAccessBag::Ptr gnssBag;
//	RandomAccessBag::Ptr velodyneBag;
	LidarScanBag::Ptr velodyneBag;

	const boost::filesystem::path bagPath;

	CameraPinholeParams cameraParams;

	// Masks for mapping and exposure adjustment
	cv::Mat dashBoardMask;
	cv::Mat exposureMask;
	cv::Mat groundPlanePatch;
	ImagePreprocessor mPreprocessor;

	// Number of all images in the bag
	uint64_t numOfFrames;

private:
	void loadCache ();
	void doLoadCache (const std::string &);
	void createTrajectories (ptime startTime=MIN_TIME, ptime stopTime=MAX_TIME, bool useNdt=true);
	void writeCache (const std::string&);

	Trajectory gnssTrack;
	Trajectory ndtTrack;
	MeidaiTrajectoryMap cameraTrack;

	float zoomRatio = 1.0;

	friend class MeidaiDataItem;

	// To be used when generating Lidar trajectory
	std::string
		velodyneCalibrationFilePath,
		pcdMapFilePath;
	TTransform lidarToCameraTransform;

	// Copy constructor. Reserved for subset, thus made private
	MeidaiBagDataset(const MeidaiBagDataset &cp);

	void prepareBag (const ros::Time &beginTime=ros::TIME_MIN, const ros::Time &stopTime=ros::TIME_MAX);
};


void createTrajectoryFromGnssBag (
	RandomAccessBag &bagsrc,
	Trajectory &trajectory,
	int plane_number=7);

void createTrajectoryFromNDT (
	LidarScanBag &bagsrc,
	Trajectory &resultTrack, const Trajectory &gnssTrack,
	const std::string &pcdMapFile);

void createTrajectoryFromNDT2 (
	LidarScanBag &bagsrc,
	Trajectory &resultTrack, const Trajectory &gnssTrack,
	const std::string &pcdMapFile);

void
createTrajectoryFromICP (
	LidarScanBag &bagsrc,
	Trajectory &resultTrack,
	const Trajectory &gnssTrack,
	const std::string &pcdMapFile);


#endif /* _MEIDAIBAG_H_ */
