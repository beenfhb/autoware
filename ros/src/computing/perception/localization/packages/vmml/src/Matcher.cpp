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

const cv::Scalar
	colorBlue(255, 0, 0),
	colorGreen(0, 255, 0),
	colorRed(0, 0, 255),
	colorYellow(0, 255, 255);
const int
	pointRadius = 3;

int
	Matcher::__maxDraw = 1;

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
Line2 createEpipolarLine (const Matrix3d &F12, const Vector2d &kp1)
{
	Line2 epl2;
	epl2.coeffs() = F12 * kp1.homogeneous();
	epl2.normalize();
	return epl2;
}


Line2 createEpipolarLine (const Matrix3d &F12, const cv::KeyPoint &kp1)
{
	return createEpipolarLine( F12, Vector2d(kp1.pt.x, kp1.pt.y) );
}


double __d;


bool Matcher::isKeypointInEpipolarLine (const Line2 &epl2, const cv::KeyPoint &cvkp2)
{
	return isKeypointInEpipolarLine(epl2, Vector2d(cvkp2.pt.x, cvkp2.pt.y));
}


bool
Matcher::isKeypointInEpipolarLine (const Line2 &epl2, const Eigen::Vector2d &kp2)
{
	auto cof = epl2.coeffs();
	auto d = abs( cof.dot(kp2.homogeneous()) / sqrt(cof[0]*cof[0] + cof[1]*cof[1]) );
//	auto lim = 3.84*VMap::mScaleFactors[cvkp2.octave];
	auto lim = 3.84 * circleOfConfusionDiameter;

	// XXX: Using scale factor makes us more dependent to ORB
	if (d > lim)
		return false;
	else {
		__d = d;
		return true;
	}
}


void drawEpipolarLine (cv::Mat &image, const Line2 &l)
{
	cv::Point2f pt1, pt2;
	Vector3d lc = l.coeffs();
	lc /= lc[2];
	float Xc = -1/lc[0], Yc = -1/lc[1];

	if (Xc >= 0 and Yc >= 0) {
		pt1 = cv::Point2f(Xc, 0);
		pt2 = cv::Point2f(0, Yc);

	}

	else if (Xc < 0 and Yc >= 0) {
		pt1 = cv::Point2f( image.cols-1, -(1+lc[0]*image.cols)/lc[1] );
		pt2 = cv::Point2f(0, Yc);
	}

	else if (Xc >= 0 and Yc < 0) {
		pt1 = cv::Point2f(Xc, 0);
		pt2 = cv::Point2f( -(1+lc[1]*image.rows)/lc[0], image.rows-1 );
	}

	cv::line(image, pt1, pt2, colorRed);
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
	featurePairs.clear();
	// debug variable. set this inside debugger
	int __debugMatch__ = -1;

	// Establish initial correspondences
	vector<cv::DMatch> initialMatches;
	matcher->match(Fr1.fDescriptors, Fr2.fDescriptors, initialMatches);

	// Sort by `distance'
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

	const int MaxBestMatch = 500;

	// Select N best matches
	vector<cv::Point2f> pointsIn1(MaxBestMatch), pointsIn2(MaxBestMatch);
	for (int i=0; i<MaxBestMatch; ++i) {
		auto &m = initialMatches[i];
		pointsIn1[i] = Fr1.fKeypoints[m.queryIdx].pt;
		pointsIn2[i] = Fr2.fKeypoints[m.trainIdx].pt;
	}
	cv::Mat Fcv = cv::findFundamentalMat(pointsIn1, pointsIn2, cv::FM_RANSAC, 3.84*Matcher::circleOfConfusionDiameter);
	// Need Eigen Matrix of F

	Matrix3d F12;
	cv2eigen(Fcv, F12);

	/*
	 * Guided Match using new F
	 */
	// Prepare constraints
/*
	cv::Mat gdMask = cv::Mat::zeros(Fr1.numOfKeyPoints(), Fr2.numOfKeyPoints(), CV_8UC1);
	vector<kpid> kpid1NeedMatch;
	set<kpid> kpid2NeedMatch;
	float maxDistance = -1;
	for (int i=0; i<initialMatches.size(); ++i) {
		auto &m = initialMatches[i];
		const Vector2d keypoint1v = Fr1.keypointv(m.queryIdx);
		const Line2 epl2 = createEpipolarLine(F12, Fr1.fKeypoints[m.queryIdx]);
		const Line2 epl1 = createEpipolarLine(F12.transpose(), Fr2.fKeypoints[m.trainIdx]);
		if (isKeypointInEpipolarLine(epl2, Fr2.keypointv(m.trainIdx))==true and isKeypointInEpipolarLine(epl1, Fr1.keypointv(m.queryIdx))==true) {
			featurePairs.push_back(make_pair(m.queryIdx, m.trainIdx));
			if (m.distance > maxDistance)
				maxDistance = m.distance;

		}
		else {
			kpid1NeedMatch.push_back(static_cast<kpid>(m.queryIdx));
			kpid2NeedMatch.insert(static_cast<kpid>(m.trainIdx));
		}
	}
*/

	/*
	 * Unfortunately, this piece of code for guided matching is still buggy at this time.
	 */
	/*
	for (int i1=0; i1<kpid1NeedMatch.size(); ++i1) {
		kpid kp1 = kpid1NeedMatch[i1];
		gdMask.setTo(cv::Scalar(0));
		const Line2 epl2 = createEpipolarLine(F12, Fr1.fKeypoints[kp1]);
		vector<kpid> listkp2;
		for (auto kp2: kpid2NeedMatch) {
			if (isKeypointInEpipolarLine(epl2, Fr2.keypointv(kp2))==true) {
				gdMask.at<uint8_t>(kp1, kp2) = 0xff;
				listkp2.push_back(kp2);
			}
		}
		vector<cv::DMatch> currentMatches;
		matcher->match(Fr1.fDescriptors, Fr2.fDescriptors, currentMatches, gdMask);
		if (currentMatches.size()<=0)
			continue;
		cv::DMatch bestMatch = *std::min_element(currentMatches.begin(), currentMatches.end());
		if (bestMatch.distance <= maxDistance) {
			cerr << "Distance: " << bestMatch.distance << "; #Candidates: " << currentMatches.size();
			featurePairs.push_back(make_pair(kp1, bestMatch.trainIdx));
			kpid2NeedMatch.erase(bestMatch.trainIdx);
		}
	}
	*/

	Matrix3d E12 = Fr2.cameraParam.toMatrix3().transpose() *
		F12 *
		Fr1.cameraParam.toMatrix3();
	cv::Mat Ecv, R1cv, R2cv, tcv;
	eigen2cv(E12, Ecv);

	/*
	 * We are too lazy here and better use OpenCV 3 routine for decomposing E
	 */
	cv::decomposeEssentialMat(Ecv, R1cv, R2cv, tcv);
	Matrix3d R1, R2;
	Vector3d t;
	cv2eigen(R1cv, R1);
	cv2eigen(R2cv, R2);
	cv2eigen(tcv, t);

	// Scale
	float S = (Fr1.mPose.inverse() * Fr2.mPose).translation().norm() / t.norm();

	float parallax1, parallax2, parallax3, parallax4;
	vector<int> good(4);
	good[0] = Matcher::CheckRT(R1, t, Fr1, Fr2, featurePairs, parallax1);
	good[1] = Matcher::CheckRT(R1, -t, Fr1, Fr2, featurePairs, parallax2);
	good[2] = Matcher::CheckRT(R2, t, Fr1, Fr2, featurePairs, parallax3);
	good[3] = Matcher::CheckRT(R2, -t, Fr1, Fr2, featurePairs, parallax4);

	// XXX: Untested
	auto g = *std::max_element(good.begin(), good.end());
	t *= S;
	if (g==good[0]) {
		T12 = Eigen::Translation3d(t) * R1;
	}
	else if (g==good[1]) {
		T12 = Eigen::Translation3d(-t) * R1;
	}
	else if (g==good[2]) {
		T12 = Eigen::Translation3d(t) * R2;
	}
	else if (g==good[3]) {
		T12 = Eigen::Translation3d(-t) * R2;
	}

	return;
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
	const Eigen::Matrix3d &R, const Eigen::Vector3d &t,
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
			pt1 = F1.keypointv(featurePairs[ip].first),
			pt2 = F2.keypointv(featurePairs[ip].second);

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


