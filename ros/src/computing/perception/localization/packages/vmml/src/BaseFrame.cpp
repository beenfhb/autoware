/*
 * BaseFrame.cpp
 *
 *  Created on: Oct 31, 2018
 *      Author: sujiwo
 */


#include <exception>
#include "BaseFrame.h"
#include "MapPoint.h"


using namespace Eigen;
using namespace std;

typedef Matrix<double,3,4> poseMatrix;
typedef Matrix4d poseMatrix4;


BaseFrame::BaseFrame()
{
	// TODO Auto-generated constructor stub
}

BaseFrame::~BaseFrame()
{
	// TODO Auto-generated destructor stub
}


Eigen::Vector2d
BaseFrame::project (const Eigen::Vector3d &pt3) const
{
	Vector3d ptx = projectionMatrix() * pt3.homogeneous();
	return ptx.head(2) / ptx[2];
}


Eigen::Vector3d
BaseFrame::project3 (const Eigen::Vector3d &pt3) const
{
	return projectionMatrix() * pt3.homogeneous();
}


Eigen::Vector2d
BaseFrame::project (const MapPoint &pt3) const
{
	return project(pt3.getPosition());
}



Vector3d
BaseFrame::transform (const Eigen::Vector3d &pt3) const
{
	Vector4d P = externalParamMatrix4() * pt3.homogeneous();
	return P.hnormalized();
}


poseMatrix4
BaseFrame::externalParamMatrix4 () const
{
	return createExternalParamMatrix4(mPose);
}


Eigen::Matrix4d
BaseFrame::createExternalParamMatrix4(const Pose &ps)
{
	poseMatrix4 ex = poseMatrix4::Identity();
	Matrix3d R = ps.orientation().toRotationMatrix().transpose();
	ex.block<3,3>(0,0) = R;
	ex.col(3).head(3) = -(R*ps.position());
	return ex;
}


Eigen::Matrix<double,3,4>
BaseFrame::projectionMatrix () const
{
	assert (cameraParam != nullptr);
	return cameraParam->toMatrix() * externalParamMatrix4();
}


Vector3d
BaseFrame::normal() const
{
	return externalParamMatrix4().block(0,0,3,3).transpose().col(2);
}


void
BaseFrame::computeFeatures (cv::Ptr<cv::FeatureDetector> fd, const cv::Mat &mask)
{
	assert (image.empty() == false);

	fd->detectAndCompute(
		image,
		mask,
		fKeypoints,
		fDescriptors,
		false);
}


void
BaseFrame::perturb (PerturbationMode mode,
	bool useRandomMotion,
	double displacement, double rotationAngle)
{
	Vector3d movement;
	switch (mode) {
	case PerturbationMode::Lateral:
		movement = Vector3d(1,0,0); break;
	case PerturbationMode::Vertical:
		movement = Vector3d(0,-1,0); break;
	case PerturbationMode::Longitudinal:
		movement = Vector3d(0,0,1); break;
	}
	movement = displacement * movement;

	mPose = mPose.shift(movement);
}


std::vector<BaseFrame::PointXYI>
BaseFrame::projectLidarScan
(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidarScan, const TTransform &lidarToCameraTransform, const CameraPinholeParams &cameraParams)
{
	std::vector<PointXYI> projections;

	// Create fake frame
	BaseFrame frame;
	frame.setPose(lidarToCameraTransform);
	frame.setCameraParam(&cameraParams);

	projections.resize(lidarScan->size());
	int i=0, j=0;
	for (auto it=lidarScan->begin(); it!=lidarScan->end(); ++it) {
		auto &pts = *it;
		Vector3d pt3d (pts.x, pts.y, pts.z);

		auto p3cam = frame.externalParamMatrix4() * pt3d.homogeneous();
		if (p3cam.z() >= 0) {
			auto p2d = frame.project(pt3d);
			projections[i] = PointXYI(p2d.x(), p2d.y(), j);
			++i;
		}
		j++;
	}

	return projections;
}


g2o::SBACam
BaseFrame::forG2O () const
{
	// XXX: Verify this
	g2o::SBACam mycam(orientation(), position());
	mycam.setKcam(cameraParam->fx, cameraParam->fy, cameraParam->cx, cameraParam->cy, 0);

	return mycam;
}


Plane3
BaseFrame::projectionPlane() const
{
	if (cameraParam==nullptr)
		throw runtime_error("Camera parameter is not defined");

	Vector3d centerPt = mPose * Vector3d(0, 0, cameraParam->f());

	Plane3 P(normal(), centerPt);
	P.normalize();
	return P;
}


Eigen::Matrix3d
BaseFrame::FundamentalMatrix(const BaseFrame &F1, const BaseFrame &F2)
{
	if (F1.cameraParam==nullptr or F2.cameraParam==nullptr)
		throw runtime_error("Camera parameters are not defined");

	TTransform T12 = F1.mPose.inverse() * F2.mPose;
	Matrix3d R = T12.rotation();
	Vector3d t = T12.translation();

	auto A = F1.cameraParam->toMatrix3() * R.transpose() * t;
	Matrix3d C = Matrix3d::Zero();
	C(0,1) = -A[2];
	C(0,2) = A[1];
	C(1,0) = A[2];
	C(1,2) = -A[0];
	C(2,0) = -A[1];
	C(2,1) = A[0];

	return F2.cameraParam->toMatrix3().inverse().transpose() * R * F1.cameraParam->toMatrix3().transpose() * C;
}

