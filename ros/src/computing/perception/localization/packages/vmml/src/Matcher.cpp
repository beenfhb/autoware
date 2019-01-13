/*
 * Matcherx.cpp
 *
 *  Created on: Jan 12, 2019
 *      Author: sujiwo
 */

#include <Matcher.h>

#include "utilities.h"


using namespace std;
using namespace Eigen;


typedef
	std::map<kpid, std::set<kpid>>
		WhichKpId;


cv::Mat
Matcher::createMatcherMask(
	const KeyFrame &kf1, const KeyFrame &kf2,
	const std::vector<kpid> &kp1List, const std::vector<kpid> &kp2List)
{
	cv::Mat mask (kf2.numOfKeyPoints(), kf1.numOfKeyPoints(), CV_8UC1, 0);

	for (int i=0; i<kp2List.size(); ++i) {
		const kpid &i2 = kp2List[i];
		for (int j=0; j<kp1List.size(); ++j) {
			const kpid &i1 = kp1List[j];
			mask.at<char>(i2, i1) = 1;
		}
	}

	return mask;
}


cv::Mat
createMatcherMask(
	const KeyFrame &kf1, const KeyFrame &kf2,
	const WhichKpId &map1to2)
{
	cv::Mat mask (kf2.numOfKeyPoints(), kf1.numOfKeyPoints(), CV_8UC1, 0);

	for (auto &pr: map1to2) {
		const kpid &k1 = pr.first;
		const auto &kp2list = pr.second;
		for (auto &k2: kp2list) {
			mask.at<char>(k2, k1) = 0xff;
		}
	}

	return mask;
}


void
Matcher::matchForInitialization(
		const KeyFrame &kf1,
		const KeyFrame &kf2,
		std::vector<FeaturePair> &featurePairs,
		cv::Ptr<cv::DescriptorMatcher> matcher)
{
	featurePairs.clear();
	Matrix3d F12 = BaseFrame::FundamentalMatrix(kf1, kf2);

	Line2 L1 = Line2::Through(Vector2d(0,0), Vector2d(kf2.cameraParam.width,0));
	Line2 L2 = Line2::Through(Vector2d(0,0), Vector2d(0,kf2.cameraParam.height));

	set<kpid> kf2targetList;
	WhichKpId kpList1to2;

	for (kpid i1=0; i1<kf1.fKeypoints.size(); i1++) {

		// Epipolar line in KF2 for this keypoint
		Vector3d kp1 (kf1.fKeypoints[i1].pt.x, kf1.fKeypoints[i1].pt.y, 1.0);

		Line2 epl2;
		epl2.coeffs() = F12 * kp1;

		// Skip if this line is not intersecting with image rectangle
		Vector2d
			intersect1 = epl2.intersection(L1),
			intersect2 = epl2.intersection(L2);

		if (intersect1.x() < 0 and intersect2.y() > kf2.cameraParam.height)
			continue;
		if (intersect1.x() > kf2.cameraParam.width and intersect2.y() < 0)
			continue;
		if (intersect1.x() > kf2.cameraParam.width and intersect2.y() > kf2.cameraParam.height)
			continue;
		if (intersect1.x() < 0 and intersect2.y() < 0)
			continue;

		kf2targetList.clear();

		for(kpid i2=0; i2<kf2.fKeypoints.size(); i2++) {
			Vector2d kp2(kf2.fKeypoints[i2].pt.x, kf2.fKeypoints[i2].pt.y);
			auto d = epl2.absDistance(kp2);

			if (d > 3.84*VMap::mScaleFactors[kf2.fKeypoints[i2].octave])
				continue;

			kf2targetList.insert(i2);
		}

		kpList1to2.insert(make_pair(i1, kf2targetList));
	}

	cv::Mat matcherMask = createMatcherMask(kf1, kf2, kpList1to2);
	vector<cv::DMatch> matchResult;
	matcher->clear();
	matcher->match(kf2.fDescriptors, kf1.fDescriptors, matchResult, matcherMask);

	for (auto &match: matchResult) {

	}
}
