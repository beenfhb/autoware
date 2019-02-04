/*
 * Matcherx.cpp
 *
 *  Created on: Jan 12, 2019
 *      Author: sujiwo
 */

#include <algorithm>
#include <Matcher.h>
#include "KeyFrame.h"

#include "utilities.h"
#include "triangulation.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
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


void
Matcher::matchAny(
	const BaseFrame &Fr1,
	const BaseFrame &Fr2,
	std::vector<KpPair> &featurePairsRet,
	cv::Ptr<cv::DescriptorMatcher> matcher,
	TTransform &T12)
{
	vector<KpPair> featurePairs;
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

	Matrix3d R1, R2;
	Vector3d t;
	decomposeE(E12, R1, R2, t);

	float parallax1, parallax2, parallax3, parallax4;
	vector<int> good(4);
	vector<bool>
		goodFeaturePairs1(featurePairs.size()),
		goodFeaturePairs2(featurePairs.size()),
		goodFeaturePairs3(featurePairs.size()),
		goodFeaturePairs4(featurePairs.size()),
		*goodFeaturePairs;
	good[0] = Matcher::CheckRT(R1, t, Fr1, Fr2, featurePairs, goodFeaturePairs1, parallax1);
	good[1] = Matcher::CheckRT(R1, -t, Fr1, Fr2, featurePairs, goodFeaturePairs2, parallax2);
	good[2] = Matcher::CheckRT(R2, t, Fr1, Fr2, featurePairs, goodFeaturePairs3, parallax3);
	good[3] = Matcher::CheckRT(R2, -t, Fr1, Fr2, featurePairs, goodFeaturePairs4, parallax4);

	auto g = *std::max_element(good.begin(), good.end());
	if (g==good[0]) {
		T12 = TTransform::from_R_t(t, R1);
		goodFeaturePairs = &goodFeaturePairs1;
	}
	else if (g==good[1]) {
		T12 = TTransform::from_R_t(-t, R1);
		goodFeaturePairs = &goodFeaturePairs2;
	}
	else if (g==good[2]) {
		T12 = TTransform::from_R_t(t, R2);
		goodFeaturePairs = &goodFeaturePairs3;
	}
	else if (g==good[3]) {
		T12 = TTransform::from_R_t(-t, R2);
		goodFeaturePairs = &goodFeaturePairs4;
	}
	featurePairsRet.clear();
	for (int i=0; i<featurePairs.size(); ++i) {
		if (goodFeaturePairs->at(i)==true)
			featurePairsRet.push_back(featurePairs.at(i));
	}

	return;
}


void
Matcher::matchMapPoints(
	const KeyFrame &KFsrc,
	const BaseFrame &Ft,
	std::vector<KpPair> &featurePairs,
	cv::Ptr<cv::DescriptorMatcher> matcher)
{
	featurePairs.clear();

	// Establish initial correspondences
	vector<cv::DMatch> initialMatches;
	matcher->match(KFsrc.fDescriptors, Ft.fDescriptors, initialMatches);

	// Sort by `distance'
	sort(initialMatches.begin(), initialMatches.end());
	vector<int> inliersMatch;

	const int MaxBestMatch = 500;
	int howmany = std::min(MaxBestMatch, static_cast<int>(initialMatches.size()));

	// Select N best matches
	vector<cv::Point2f> pointsIn1(MaxBestMatch), pointsIn2(MaxBestMatch);
	for (int i=0; i<howmany; ++i) {
		auto &m = initialMatches[i];
		pointsIn1[i] = KFsrc.fKeypoints[m.queryIdx].pt;
		pointsIn2[i] = Ft.fKeypoints[m.trainIdx].pt;
	}
	cv::Mat Fcv = cv::findFundamentalMat(pointsIn1, pointsIn2, cv::FM_RANSAC, 3.84*Matcher::circleOfConfusionDiameter);
	// Need Eigen Matrix of F

	Matrix3d F12;
	cv2eigen(Fcv, F12);

	/*
	 * Guided Match using new F
	 */
	// Prepare constraints
	cv::Mat gdMask = cv::Mat::zeros(KFsrc.numOfKeyPoints(), Ft.numOfKeyPoints(), CV_8UC1);
	float maxDistance = -1;
	for (int i=0; i<initialMatches.size(); ++i) {
		auto &m = initialMatches[i];
		const Vector2d keypoint1v = KFsrc.keypointv(m.queryIdx);
		const Line2 epl2 = createEpipolarLine(F12, KFsrc.fKeypoints[m.queryIdx]);
		const Line2 epl1 = createEpipolarLine(F12.transpose(), Ft.fKeypoints[m.trainIdx]);
		if (isKeypointInEpipolarLine(epl2, Ft.keypointv(m.trainIdx))==true and isKeypointInEpipolarLine(epl1, KFsrc.keypointv(m.queryIdx))==true) {
			featurePairs.push_back(make_pair(m.queryIdx, m.trainIdx));
		}
	}
}


void
Matcher::rotationFinder
(const BaseFrame &Fr1, const BaseFrame &Fr2,
const std::vector<KpPair> &featurePairs,
std::vector<double> &cs12)
{
	MatrixX4d M;
	M.resize(featurePairs.size(), Eigen::NoChange);
	for (int ip=0; ip<featurePairs.size(); ++ip) {
		auto point_pair = featurePairs[ip];
		Vector3d p1 = Fr1.cameraParam.toMatrix3().inverse() * Fr1.keypointh(point_pair.first);
		Vector3d p2 = Fr2.cameraParam.toMatrix3().inverse() * Fr2.keypointh(point_pair.second);
		M(ip, 0) = p1.x() * p2.y();
		M(ip, 1) = p1.y() * p2.x();
		M(ip, 2) = p1.z() * p2.y();
		M(ip, 3) = p1.y() * p2.z();
	}

	// Solve for h
	JacobiSVD <decltype(M)> svd(M, ComputeFullV);
	Vector4d h = svd.matrixV().col(3);
	double
		c1 = -h[0],
		s1 = h[3],
		c2 = (h[0]*h[1] - h[2]*h[3]) / (h[2]*h[2] - h[1]*h[1]),
		s2 = (h[1]*h[2] - h[0]*h[3]) / (h[2]*h[2] - h[0]*h[0]);

	cs12.resize(4);
	cs12[0] = c1;
	cs12[1] = s1;
	cs12[2] = c2;
	cs12[3] = s2;
}


double
Matcher::getCameraBaselinkOffset
(const Pose &baselinkPose1, const Pose &baselinkPose2, const std::vector<double> &cs12)
{
	double
		sin_c_2 = sqrt((1-cs12[2])/2),
		cos_c_2 = sqrt((1+cs12[2])/2),
		sin_c_2_1 = sin_c_2*cs12[0] - cos_c_2*cs12[1];
	double
		rho = (baselinkPose2.position()-baselinkPose1.position()).norm();
	double
		L = rho * (-sin_c_2_1) / ( cs12[1] + cs12[3]*cs12[0] - cs12[2]*cs12[1] );
	return L;
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
	std::vector<bool> &goodFeaturePairs,
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
		goodFeaturePairs[ip] = true;
	}

	if (nGood > 0) {
		// XXX: Unfinished, we do not collect parallaxes
	}
	else
		parallax = 0;

	return nGood;
}


