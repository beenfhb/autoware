/*
 * Matcherx.h
 *
 *  Created on: Jan 12, 2019
 *      Author: sujiwo
 */

#ifndef _MATCHER_H_
#define _MATCHER_H_

#include <vector>
#include <map>
#include <set>

#include "VMap.h"
#include "KeyFrame.h"


class Matcher {
public:

	typedef std::pair<kpid,kpid> KpPair;

	static void
	matchForInitialization(
		const KeyFrame &kf1,
		const KeyFrame &kf2,
		std::vector<FeaturePair> &featurePairs,
		cv::Ptr<cv::DescriptorMatcher> matcher);

	// XXX: Not implemented yet
	static void
	matchMapPoints(
		const KeyFrame &kfOld,
		const KeyFrame &kfNew,
		std::vector<FeaturePair> &matchedKeyPts,
		cv::Ptr<cv::DescriptorMatcher> matcher);

	// XXX: Not implemented yet
	static void
	matchNonMapPoints(
		const KeyFrame &kfOld,
		const KeyFrame &kfNew,
		std::vector<FeaturePair> &featurePairs,
		cv::Ptr<cv::DescriptorMatcher> matcher);

	static void
	matchAny(
		const BaseFrame &F1,
		const BaseFrame &F2,
		std::vector<KpPair> &featurePairs,
		cv::Ptr<cv::DescriptorMatcher> matcher,
		TTransform &T12);

	enum DrawMode {
		DrawOpticalFlow,
		DrawSideBySide,
		DrawOnlyPoints
	};

	static cv::Mat
	drawMatches(
		const BaseFrame &F1,
		const BaseFrame &F2,
		const std::vector<KpPair> &featurePairs,
		DrawMode m,
		int maxNumOfPairs=-1);

	static void
	decomposeE (const Eigen::Matrix3d &E, Eigen::Matrix3d &R1, Eigen::Matrix3d &R2, Eigen::Vector3d &t);

	static float
	circleOfConfusionDiameter;

protected:

	static cv::Mat
	createMatcherMask(
		const KeyFrame &kf1, const KeyFrame &kf2,
		const std::vector<kpid> &kp1List, const std::vector<kpid> &kp2List);

	static cv::Mat
	createMatcherMask(
		const BaseFrame &kf1, const BaseFrame &kf2,
		const std::map<kpid, std::set<kpid>> &map1to2);

	static bool
	isKeypointInEpipolarLine (const Line2 &epl2, const cv::KeyPoint &cvkp2);

	static int
	CheckRT (
		const Eigen::Matrix3d R, const Eigen::Vector3d &t,
		const BaseFrame &F1, const BaseFrame &F2,
		const std::vector<KpPair> &featurePairs,
		float &parallax);
};

#endif /* _MATCHER_H_ */
