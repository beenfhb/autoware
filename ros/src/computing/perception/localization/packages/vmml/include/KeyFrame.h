/*
 * KeyFrame.h
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <map>
#include <Eigen/Eigen>
#include <string>
#include <memory>
#include <tuple>
#include <boost/serialization/serialization.hpp>
#include <limits>

#include "VMap.h"
#include "MapPoint.h"
#include "triangulation.h"
#include "utilities.h"
#include "BaseFrame.h"
#include "Matcher.h"
#include "datasets/GenericDataset.h"


namespace boost {
namespace serialization {
	template <class Archive>
		void serialize (Archive & ar, KeyFrame &keyframe, const unsigned int version);
}
}


//typedef std::tuple<int64, cv::Point2f, int64, cv::Point2f> FeaturePair;
struct FeaturePair {
	kpid kpid1;
	cv::Point2f keypoint1;
	kpid kpid2;
	cv::Point2f keypoint2;

	Eigen::Vector2d toEigen1 () const
	{ return Eigen::Vector2d(keypoint1.x, keypoint1.y); }

	Eigen::Vector2d toEigen2 () const
	{ return Eigen::Vector2d(keypoint2.x, keypoint2.y); }
};

struct CameraPinholeParams;

//const kfid defaultKfId = std::numeric_limits<kfid>;

class Frame;


class KeyFrame : public BaseFrame
{
public:

	KeyFrame();

	/*
	 * Notes:
	 * we require that the pose specified here must be in world coordinate,
	 * with X->left, Y->bottom, Z->front
	 */
	KeyFrame(const cv::Mat &imgSrc,
			const Eigen::Vector3d &p, const Eigen::Quaterniond &o,
			cv::Mat &mask,
			cv::Ptr<cv::FeatureDetector> fdetector,
			const CameraPinholeParams *cameraIntr,
			const int _cameraId,
			dataItemId _srcItemId=std::numeric_limits<dataItemId>::max());

	virtual ~KeyFrame();

	inline std::vector<cv::KeyPoint> getKeypoints() const
	{ return fKeypoints; }

	const VMap& getParent() const
	{ return *parentMap; }

	/*static void match (const KeyFrame &k1, const KeyFrame &k2,
		cv::Ptr<cv::DescriptorMatcher> matcher,
		std::vector<FeaturePair> &featurePairs
	);*/


	static void triangulate (
		const KeyFrame &KF1, const KeyFrame &KF2,
		const std::vector<Matcher::KpPair> &kpPair,
		std::vector<mpid> &newMapPointList,
		std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame1,
		std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame2,
		VMap &parent);

	static void triangulateCV (
		const KeyFrame &KF1, const KeyFrame &KF2,
		const std::vector<Matcher::KpPair> &kpPair,
		std::vector<mpid> &newMapPointList,
		std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame1,
		std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame2,
		VMap &parent);


	kfid getId () const
	{ return id; }

	int getCameraId() const
	{ return cameraId; }

	static std::set<kpid> allKeyPointId (const KeyFrame &kf);

	cv::Mat getImage() const
	{ return image; }

	void setTimestamp
	(const ptime &t)
	{ frCreationTime = t; }

	ptime getTimestamp() const
	{ return frCreationTime; }

	std::vector<Eigen::Vector2d>
	projectAllMapPoints () const;

	dataItemId getSourceItemId() const
	{ return srcItemId; }

	void debugMapPoints() const;

	void debugKeyPoints() const;

	void updateNormal();

	kfid previousKeyframe = std::numeric_limits<kfid>::max();

protected:

	template <class Archive>
    friend void boost::serialization::serialize (Archive & ar, KeyFrame &keyframe, const unsigned int version);

	friend class VMap;

	friend class Matcher;

	kfid id;

	// To be used for referring to original dataset
	dataItemId srcItemId;

	int cameraId;

	// Time at which the image was taken
	ptime frCreationTime;

	static kfid nextId;

	VMap* parentMap;

};

#endif /* KEYFRAME_H_ */
