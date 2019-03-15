/*
 * KeyFrame.cpp
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#include <Eigen/Core>
#include <bitset>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "KeyFrame.h"
#include <exception>
#include "Frame.h"
//#include "MapBuilder.h"
#include "utilities.h"
#include "MapPoint.h"



using namespace std;
using namespace Eigen;


/*
 * Ensure that KeyFrame ID is always positive and non-zero
 */
kfid KeyFrame::nextId = 1;


typedef Matrix<double,3,4> poseMatrix;
typedef Matrix4d poseMatrix4;


const float pixelReprojectionError = 6.0;


std::set<kpid>
KeyFrame::allKeyPointId (const KeyFrame &kf)
{
	std::set<kpid> allkp;
	for (kpid i=0; i<kf.fKeypoints.size(); i++)
		allkp.insert(i);
	return allkp;
}


KeyFrame::KeyFrame()
{}


KeyFrame::KeyFrame(
	const cv::Mat &imgSrc,
	const Vector3d &p, const Eigen::Quaterniond &o,
	cv::Mat &mask,
	cv::Ptr<cv::FeatureDetector> fdetector,
	const CameraPinholeParams *cameraIntr,
	const int _cameraId,
	dataItemId _srcItemId) :

	cameraId(_cameraId),
	frCreationTime(boost::posix_time::second_clock::local_time()),
	srcItemId(_srcItemId),
	parentMap(NULL)

{
	setPose(p, o);
	cameraParam = *cameraIntr;

	if(cameraIntr->width < 0 or cameraIntr->height < 0)
		throw runtime_error("Camera parameter has not been initialized properly (<0)");

	id = nextId++;

	image = imgSrc;
	computeFeatures(fdetector, mask);
}


KeyFrame::~KeyFrame()
{
	// TODO Auto-generated destructor stub
}


Eigen::Vector2f
convertToEigen (const cv::Point2f &P)
{
	return Eigen::Vector2f(P.x, P.y);
}



void debugMatch (const cv::Mat &imgToDraw, const vector<cv::DMatch> &matches, const vector<cv::KeyPoint> &kpList1, const vector<cv::KeyPoint> &kpList2, const string &filename)
{
	cv::Mat newColorImage;
	cv::cvtColor(imgToDraw, newColorImage, CV_GRAY2BGR);

	for (int i=0; i<matches.size(); ++i) {
		const cv::DMatch &mt = matches[i];
		if (mt.trainIdx >= kpList1.size() or mt.queryIdx >= kpList2.size())
			continue;
		const auto &kp1 = kpList1[mt.trainIdx];
		const auto &kp2 = kpList2[mt.queryIdx];
		cv::line(newColorImage, kp1.pt, kp2.pt, cv::Scalar(0,255,0));
	}

	cv::imwrite (filename, newColorImage);
}


void KeyFrame::triangulate (
	const KeyFrame &KF1, const KeyFrame &KF2,
	const std::vector<Matcher::KpPair> &featurePairs,
	std::vector<mpid> &newMapPointList,
	std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame1,
	std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame2,
	VMap &parent)
{
	const poseMatrix pm1 = KF1.projectionMatrix(),
		pm2 = KF2.projectionMatrix();

	newMapPointList.clear();

	for (uint i=0; i<featurePairs.size(); i++) {
		auto &fp = featurePairs[i];

		Vector2d proj1 = KF1.keypointv(fp.first),
			proj2 = KF2.keypointv(fp.second);

		Vector4d triangulatedpt;
		TriangulateDLT (pm1, pm2, proj1, proj2, triangulatedpt);
		Vector3d pointm = triangulatedpt.head(3);

		// Check for Reprojection Errors
		float pj1 = (KF1.project(pointm) - proj1).norm(),
			pj2 = (KF2.project(pointm) - proj2).norm();
		if (pj1 > pixelReprojectionError or pj2 > pixelReprojectionError)
			continue;

		// checking for regularity of triangulation result
		// 1: Point must be in front of camera
		Vector3d v1 = pointm - KF1.position();
		double cos1 = v1.dot(KF1.normal()) / v1.norm();
		if (cos1 < 0)
			continue;
		double dist1 = v1.norm();
		Vector3d v2 = pointm - KF2.position();
		double cos2 = v2.dot(KF2.normal()) / v2.norm();
		if (cos2 < 0)
			continue;
		double dist2 = v2.norm();

		// 2: Must have enough parallax (ie. remove faraway points)
		double cosParallax = (-v1).dot(-v2) / (dist1 * dist2);
		if (cosParallax >= 0.999990481)
			continue;

		mpid newMp = parent.createMapPoint(pointm);
		newMapPointList.push_back(newMp);
		mapPointToKeyPointInKeyFrame1[newMp] = fp.first;
		mapPointToKeyPointInKeyFrame2[newMp] = fp.second;
	}
}


void
KeyFrame::triangulateCV (
	const KeyFrame &KF1, const KeyFrame &KF2,
	const std::vector<Matcher::KpPair> &kpPair,
	std::vector<mpid> &newMapPointList,
	std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame1,
	std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame2,
	VMap &parent)
{
	const int N = kpPair.size();
	const poseMatrix
		pm1 = KF1.projectionMatrix(),
		pm2 = KF2.projectionMatrix();
	cv::Mat projMat1, projMat2;
	vector<cv::Point2f> pointsInFrame1(N), pointsInFrame2(N);
	cv::Mat triangulationResults;


	cv::eigen2cv(pm1, projMat1);
	cv::eigen2cv(pm2, projMat2);

	int i=0;
	for (auto &pointPair: kpPair) {
		pointsInFrame1[i] = KF1.keypoint(pointPair.first).pt;
		pointsInFrame2[i] = KF2.keypoint(pointPair.second).pt;
		++i;
	}

	cv::triangulatePoints(projMat1, projMat2, pointsInFrame1, pointsInFrame2, triangulationResults);
	if (N!=triangulationResults.cols)
		throw runtime_error("Unexpected number of points; should be "+to_string(kpPair.size()));

	/*
	 * Check triangulation results
	 */
	for (i=0; i<N; ++i) {
		cv::Vec4f p = triangulationResults.col(i);
		p /= p[3];
		Vector3d pointm(p[0], p[1], p[2]);

		auto
			proj1 = KF1.keypointv(kpPair[i].first),
			proj2 = KF2.keypointv(kpPair[i].second);
		auto
			e1 = KF1.keypoint(kpPair[i].first).octave * Matcher::circleOfConfusionDiameter,
			e2 = KF2.keypoint(kpPair[i].second).octave * Matcher::circleOfConfusionDiameter;

		// Check for Reprojection Errors
		float pj1 = (KF1.project(pointm) - proj1).norm(),
			pj2 = (KF2.project(pointm) - proj2).norm();
		if (pj1 > e1 or pj2 > e2)
			continue;

		// checking for regularity of triangulation result
		// 1: Point must be in front of camera
		Vector3d v1 = pointm - KF1.position();
		Vector3d trans1 = KF1.transform(pointm);
		if (trans1.z() < 0)
			continue;

		Vector3d v2 = pointm - KF2.position();
		Vector3d trans2 = KF2.transform(pointm);
		if (trans2.z() < 0)
			continue;

		// 2: Must have enough parallax (ie. remove faraway points)
		double cosParallax = (-v1).dot(-v2) / (v1.norm() * v2.norm());
		if (cosParallax >= 0.999990481)
			continue;

		mpid newMp = parent.createMapPoint(pointm);
		newMapPointList.push_back(newMp);
		mapPointToKeyPointInKeyFrame1[newMp] = kpPair[i].first;
		mapPointToKeyPointInKeyFrame2[newMp] = kpPair[i].second;
	}
}


vector<Vector2d>
KeyFrame::projectAllMapPoints() const
{
	auto visibleMapPts = parentMap->allMapPointsAtKeyFrame(this->id);
	vector<Vector2d> projectionResult(visibleMapPts.size());

	int i = 0;
	for (auto &ptx: visibleMapPts) {
		mpid pt3d = ptx.first;
		projectionResult[i] = project(*parentMap->mappoint(pt3d));
		i++;
	}

	return projectionResult;
}


void
KeyFrame::debugMapPoints() const
{
	static const string _dbgKfMapPoints = "/tmp/kf_mappoints";
}


void
KeyFrame::debugKeyPoints() const
{

}




