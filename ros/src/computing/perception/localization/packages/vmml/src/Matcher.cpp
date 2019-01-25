/*
 * Matcherx.cpp
 *
 *  Created on: Jan 12, 2019
 *      Author: sujiwo
 */

#include <algorithm>
#include <Matcher.h>

#include "utilities.h"
#include "triangulation.h"

#include <opencv2/core/eigen.hpp>


using namespace std;
using namespace Eigen;


typedef
	std::map<kpid, std::set<kpid>>
		WhichKpId;

float
	Matcher::circleOfConfusionDiameter = 1.0;


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


void
Matcher::decomposeE (
	const Eigen::Matrix3d &E,
	Eigen::Matrix3d &R1, Eigen::Matrix3d &R2,
	Eigen::Vector3d &t)
{
	JacobiSVD <Matrix3d> svd(E, ComputeFullU|ComputeFullV);
	Matrix3d U, W, V;
	U = svd.matrixU();
	V = svd.matrixV();

	t = U.col(2);
	t /= t.norm();

	W = Matrix3d::Zero();
	W(0,1) = -1;
	W(1,0) = 1;
	W(2,2) = 1;

	R1 = U * W * V.transpose();
	if (R1.determinant() < 0)
		R1 = -R1;
	R2 = U * W.transpose() * V.transpose();
	if (R2.determinant() < 0)
		R2 = -R2;
}


cv::Mat
Matcher::createMatcherMask(
	const BaseFrame &kf1, const BaseFrame &kf2,
	const WhichKpId &map1to2)
{
//	cv::Mat mask (kf1.numOfKeyPoints(), kf2.numOfKeyPoints(), CV_8UC1, 0);
	cv::Mat mask;
	mask = cv::Mat::zeros(kf1.numOfKeyPoints(), kf2.numOfKeyPoints(), CV_8UC1);

	for (auto &pr: map1to2) {
		const kpid &k1 = pr.first;
		const auto &kp2list = pr.second;
		for (auto &k2: kp2list) {
			mask.at<char>(k1, k2) = 0xff;
		}
	}

	return mask;
}


/*
 * Create an epipolar line in Frame 2 based on Fundamental Matrix F12, using a keypoint from Frame 1
 */
Line2 createEpipolarLine (const Matrix3d &F12, const cv::KeyPoint &kp1)
{
	Line2 epl2;
	epl2.coeffs() = F12 * Vector3d(kp1.pt.x, kp1.pt.y, 1.0);
	epl2.normalize();
	return epl2;
}

double __d;

/*
 * XXX: Modify this function to employ circle of confusion instead of scale factors
 */
bool Matcher::isKeypointInEpipolarLine (const Line2 &epl2, const cv::KeyPoint &cvkp2)
{
	Vector2d kp2(cvkp2.pt.x, cvkp2.pt.y);
	auto cof = epl2.coeffs();
	auto d = abs( cof.dot(kp2.homogeneous()) / sqrt(cof[0]*cof[0] + cof[1]*cof[1]) );
//	auto lim = 3.84*VMap::mScaleFactors[cvkp2.octave];
	auto lim = circleOfConfusionDiameter;

	// XXX: Using scale factor makes us more dependent to ORB
	if (d > lim)
		return false;
	else {
		__d = d;
		return true;
	}
}


/*
 * XXX: This function is subject to elimination
 */
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

		Vector2d keypoint1 (kf1.fKeypoints[i1].pt.x, kf1.fKeypoints[i1].pt.y);

		// Epipolar line in KF2 for this keypoint
		Line2 epl2 = createEpipolarLine(F12, kf1.fKeypoints[i1]);

		// Skip if this line is not intersecting with image rectangle in Frame 2
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

		for(kpid i2=0; i2<kf2.fKeypoints.size(); ++i2) {

			Vector2d keypoint2 (kf2.fKeypoints[i2].pt.x, kf2.fKeypoints[i2].pt.y);

			if (isKeypointInEpipolarLine(epl2, kf2.fKeypoints[i2]) == false)
				continue;
			kf2targetList.insert(i2);
		}

		if (kf2targetList.size() != 0)
			kpList1to2.insert(make_pair(i1, kf2targetList));
	}

	cv::Mat matcherMask = createMatcherMask(kf1, kf2, kpList1to2);
	vector<cv::DMatch> matchResult;
	matcher->clear();
	matcher->match(kf2.fDescriptors, kf1.fDescriptors, matchResult, matcherMask);

	for (auto &match: matchResult) {
		kpid
			kp1 = match.queryIdx,
			kp2 = match.trainIdx;

		// XXX: Re-Check if kp2 is truly in kp1's epipolar line
		Line2 epl = createEpipolarLine(F12, kf1.fKeypoints[kp1]);

		if (isKeypointInEpipolarLine(epl, kf2.fKeypoints[kp2])==true) {
			FeaturePair pair12 = {kp1, kf1.fKeypoints[kp1].pt, kp2, kf2.fKeypoints[kp2].pt};
			featurePairs.push_back(pair12);
		}
	}
}





void
Matcher::matchAny(
	const BaseFrame &Fr1,
	const BaseFrame &Fr2,
	std::vector<KpPair> &featurePairs,
	cv::Ptr<cv::DescriptorMatcher> matcher,
	TTransform &T12)
{
	// debug variable. set this inside debugger
	int __debugMatch__ = 2;

	// Estimate F
	featurePairs.clear();
	Matrix3d F12 = BaseFrame::FundamentalMatrix(Fr1, Fr2);

	// Establish initial correspondences
	vector<cv::DMatch> initialMatches;
	matcher->match(Fr1.fDescriptors, Fr2.fDescriptors, initialMatches);
	sort(initialMatches.begin(), initialMatches.end());
	vector<int> inliersMatch;

	// Debug brute force matching
	if (__debugMatch__==1) {
		featurePairs.reserve(initialMatches.size());
		for (int i=0; i<initialMatches.size(); ++i) {
			auto pr = initialMatches[i];
			featurePairs.push_back( make_pair(static_cast<kpid>(pr.queryIdx), static_cast<kpid>(pr.trainIdx)) );
		}
		return;
	}

	/*
	 * Find outlier/inlier
	 */
	vector<double> distv;
	for (int ip=0; ip<initialMatches.size(); ++ip) {

		auto dm = initialMatches[ip];
		Line2 line2 = createEpipolarLine(F12, Fr1.fKeypoints[dm.queryIdx]);
		if (isKeypointInEpipolarLine(line2, Fr2.fKeypoints[dm.trainIdx])==true) {
			inliersMatch.push_back(ip);
			distv.push_back(__d);
		}
	}

	// Debug Inlier/Outlier selection from initial F12
	if (__debugMatch__==2) {
		featurePairs.reserve(inliersMatch.size());
		for (auto &i: inliersMatch) {
			auto pr = initialMatches[i];
			featurePairs.push_back( make_pair((kpid)pr.queryIdx, (kpid)pr.trainIdx) );
		}
		return;
	}

	/*
	 * Compute F
	 */
	cv::Mat points1(inliersMatch.size(), 2, CV_32F),
			points2(inliersMatch.size(), 2, CV_32F);
	for (int ip=0; ip<inliersMatch.size(); ++ip) {
		cv::DMatch m = initialMatches[inliersMatch[ip]];
		points1.at<float>(ip,0) = Fr1.fKeypoints[m.queryIdx].pt.x;
		points1.at<float>(ip,1) = Fr1.fKeypoints[m.queryIdx].pt.y;
		points2.at<float>(ip,0) = Fr2.fKeypoints[m.trainIdx].pt.x;
		points2.at<float>(ip,1) = Fr2.fKeypoints[m.trainIdx].pt.y;
	}
	cv::Mat Fcv = cv::findFundamentalMat(points1, points2);
	Matrix3d F12x;
	cv2eigen(Fcv, F12x);

	/*
	 * Guided matching using epipolar lines
	 */
	WhichKpId kpList1to2;
	for (kpid i1=0; i1<Fr1.fKeypoints.size(); ++i1) {
		Vector2d keypoint1 (Fr1.fKeypoints[i1].pt.x, Fr1.fKeypoints[i1].pt.y);

		// Epipolar line in KF2 for this keypoint
		Line2 epl2 = createEpipolarLine(F12x, Fr1.fKeypoints[i1]);

		set<kpid> kf2targetList;
		kf2targetList.clear();

		for(kpid i2=0; i2<Fr2.fKeypoints.size(); ++i2) {
			if (isKeypointInEpipolarLine(epl2, Fr2.fKeypoints[i2]) == false)
				continue;
			kf2targetList.insert(i2);
		}

		if (kf2targetList.size() != 0)
			kpList1to2.insert(make_pair(i1, kf2targetList));
	}

	cv::Mat matcherMask = createMatcherMask(Fr1, Fr2, kpList1to2);
	vector<cv::DMatch> matchResult;
	matcher->clear();
	matcher->match(Fr1.fDescriptors, Fr2.fDescriptors, matchResult, matcherMask);
	sort(matchResult.begin(), matchResult.end());

	/*
	 * Weed out outliers
	 * Put valid feature pairs in results
	 */
	inliersMatch.clear();
	for (int ip=0; ip<matchResult.size(); ++ip) {
		auto dm = matchResult[ip];
		Line2 line2 = createEpipolarLine(F12x, Fr1.fKeypoints[dm.queryIdx]);
		if (isKeypointInEpipolarLine(line2, Fr2.fKeypoints[dm.trainIdx])==true) {
			inliersMatch.push_back(ip);
			auto p = make_pair(static_cast<kpid>(dm.queryIdx), static_cast<kpid>(dm.trainIdx));
			featurePairs.push_back(p);
		}
	}
	if (__debugMatch__==3) {
		return;
	}

	/*
	 * Convert F to Essential Matrix E, and compute R & T from Fr1 to Fr2
	 */
	Matrix3d Ex = Fr2.cameraParam.toMatrix3().transpose() * F12x * Fr1.cameraParam.toMatrix3();
	Matrix3d R1, R2;
	Vector3d t1, t2;
	decomposeE(Ex, R1, R2, t1);
	t2 = -t1;

	// Currently unused
	float parallax1, parallax2, parallax3, parallax4;
	vector<int> nGood(4);

	nGood[0] = CheckRT(R1, t1, Fr1, Fr2, featurePairs, parallax1);
	nGood[1] = CheckRT(R2, t1, Fr1, Fr2, featurePairs, parallax1);
	nGood[2] = CheckRT(R1, t2, Fr1, Fr2, featurePairs, parallax1);
	nGood[3] = CheckRT(R2, t2, Fr1, Fr2, featurePairs, parallax1);

	// XXX: Watch out the result of this line
	int bestGood = std::max_element(nGood.begin(), nGood.end()) - nGood.begin();
	Affine3d T1122;
	if (bestGood==0) {
		T1122 = Eigen::Translation3d(t1) * Quaterniond(R1);
	}
	else if (bestGood==1) {
		T1122 = Eigen::Translation3d(t1) * Quaterniond(R2);
	}
	else if (bestGood==2) {
		T1122 = Eigen::Translation3d(t2) * Quaterniond(R1);
	}
	else if (bestGood==3) {
		T1122 = Eigen::Translation3d(t2) * Quaterniond(R2);
	}
	T12 = T1122;
}


cv::Mat
Matcher::drawMatches(
	const BaseFrame &F1,
	const BaseFrame &F2,
	const std::vector<KpPair> &featurePairs,
	DrawMode mode,
	int maxNumOfPairs)
{
	cv::Mat result(std::max(F1.height(), F2.height()), F1.width()+F2.width(), F1.image.type());
	F1.image.copyTo( result(cv::Rect(0,0,F1.width(),F1.height())) );
	F2.image.copyTo( result(cv::Rect(F1.width(),0,F2.width(),F2.height())) );
//	cv::Mat result = F2.image.clone();
	if (maxNumOfPairs<0)
		maxNumOfPairs = featurePairs.size();
	else
		maxNumOfPairs = min(maxNumOfPairs, static_cast<int>(featurePairs.size()));

	vector<pair<cv::Point2f, cv::Point2f>> pointPairList(featurePairs.size());
	for (int i=0; i<featurePairs.size(); ++i) {
		cv::Point2f p2 = F2.fKeypoints[featurePairs[i].second].pt;
		p2.x += F1.width();
		pointPairList[i] = make_pair(
			F1.fKeypoints[featurePairs[i].first].pt,
			p2);
	}

	const cv::Scalar
		colorBlue(255, 0, 0),
		colorGreen(0, 255, 0),
		colorRed(0, 0, 255),
		colorYellow(0, 255, 255);
	const int
		pointRadius = 3;

	// Draw P2 in P1
	Pose P1z = F2.pose();
	Vector2d C2in1 = F1.project(P1z.position());
	cv::circle(result, cv::Point2f(C2in1.x(), C2in1.y()), pointRadius*2, colorYellow);

	if (mode==DrawOpticalFlow) {
		for (int n=0; n<maxNumOfPairs; ++n) {
			auto &pr = pointPairList[n];
			cv::circle(result, pr.first, pointRadius, colorBlue);
			cv::circle(result, pr.second, pointRadius, colorBlue);
			cv::Point2f pt1s = pr.first;
			pt1s.x += F1.width();
			cv::line(result, pt1s, pr.second, colorGreen);
		}
	}

	else if (mode==DrawSideBySide) {
		for (int n=0; n<maxNumOfPairs; ++n) {
			auto &pr = pointPairList[n];
			cv::circle(result, pr.first, pointRadius, colorBlue);
			cv::circle(result, pr.second, pointRadius, colorBlue);
			cv::line(result, pr.first, pr.second, colorGreen);
		}
	}

	else if (mode==DrawOnlyPoints) {
		for (int n=0; n<maxNumOfPairs; ++n) {
			auto &pr = pointPairList[n];
			cv::circle(result, pr.first, pointRadius, colorBlue);
			cv::circle(result, pr.second, pointRadius, colorBlue);
		}
	}

	else throw runtime_error("Invalid mode");

	return result;
}


int
Matcher::CheckRT (
	const Eigen::Matrix3d R, const Eigen::Vector3d &t,
	const BaseFrame &F1, const BaseFrame &F2,
	const std::vector<KpPair> &featurePairs,
	float &parallax)
{
	int nGood = 0;
	Vector3d
		origin1 = Vector3d::Zero(),
		origin2 = -R.transpose() * t;

	// Build new projection matrices
	// For camera 1: K[I | O]
	BaseFrame::ProjectionMat P1 = BaseFrame::ProjectionMat::Zero();
	P1.block<3,3>(0,0) = F1.cameraParam.toMatrix3();

	// For camera 2: K[R | t]
	BaseFrame::ProjectionMat P2 = BaseFrame::ProjectionMat::Zero();
	P2.block<3,3>(0,0) = R;
	P2.block<3,1>(0,3) = t;
	P2 = F2.cameraParam.toMatrix3() * P2;

	vector<double> vCosParallax(featurePairs.size());

	/*
	 * Triangulate each point pair as if first camera is in the origin
	 */
	for (int ip=0; ip<featurePairs.size(); ++ip) {

		Vector2d
			pt1 = cv2eigen(F1.fKeypoints[featurePairs[ip].first].pt),
			pt2 = cv2eigen(F1.fKeypoints[featurePairs[ip].second].pt);

		Vector4d point3D_;
		TriangulateDLT(P1, P2, pt1, pt2, point3D_);
		Vector3d point3D = (point3D_ / point3D_[3]).hnormalized();

		if (!isfinite(point3D[2]) or !isfinite(point3D[1]) or !isfinite(point3D[2]) )
			continue;

		Vector3d
			normal1 = point3D - origin1,
			normal2 = point3D - origin2;
		double
			dist1 = normal1.norm(),
			dist2 = normal2.norm();

		double cosParallax = normal1.dot(normal2) / (dist1 * dist2);

		// Check that depth regarding camera1&2 must be positive
		if (cosParallax < 0.99998) {
			if (point3D[2]<=0)
				continue;
			Vector3d point3D2 = R*point3D + t;
			if (point3D2[2]<=0)
				continue;
		}

		// Check reprojection errors
		Vector3d
			proj1 = P1 * point3D.homogeneous(),
			proj2 = P2 * point3D.homogeneous();
		proj1 /= proj1[2];
		proj2 /= proj2[2];

		float
			squareError1 = pow((proj1.hnormalized()-pt1).norm(), 2),
			squareError2 = pow((proj2.hnormalized()-pt2).norm(), 2),
			threshold = pow(Matcher::circleOfConfusionDiameter, 2);
		if (squareError1 > threshold or squareError2 > threshold)
			continue;

		nGood++;
	}

	if (nGood > 0) {
		// XXX: Unfinished, we do not collect parallaxes
	}
	else
		parallax = 0;

	return nGood;
}
