/*
 * Trajectory.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: sujiwo
 */

#include <stdexcept>

#include "Trajectory.h"


using namespace std;


PoseStamped
PoseStamped::operator* (const Pose &transform)
{
//	Pose me = Pose::from_Pos_Quat(this->position(), this->orientation());
	Pose P = static_cast<Pose&>(*this) * transform;
	return PoseStamped(P, this->timestamp);
}


PoseStamped
PoseStamped::interpolate(
	const PoseStamped &p1,
	const PoseStamped &p2,
	const ptime &t)
{
	assert (p1.timestamp<=t and t<=p2.timestamp);
	double r = toSeconds(t - p1.timestamp) / toSeconds(p2.timestamp - p1.timestamp);
	return PoseStamped (Pose::interpolate(p1, p2, r), t);
}


Quaterniond
PoseStamped::extrapolate(const PoseStamped &p1, const PoseStamped &p2, const decltype(PoseStamped::timestamp) &txinp)
{
	assert (p1.timestamp < p2.timestamp);
	if (p1.timestamp <= txinp and txinp <= p2.timestamp)
		throw runtime_error("Requested timestamp are within both data points; use interpolation instead");

	Vector3d euler1, euler2, eulerx;
	euler1 = quaternionToRPY(p1.orientation());
	euler2 = quaternionToRPY(p2.orientation());
	double t1, t2, tx, r;
	t1 = toSeconds(p1.timestamp);
	t2 = toSeconds(p2.timestamp);
	tx = toSeconds(txinp);

	if (txinp<=p1.timestamp) {
		r = (t1 - tx) / (t2 - t1);
		eulerx.x() = euler1.x() - r*(euler2.x() - euler1.x());
		eulerx.y() = euler1.y() - r*(euler2.y() - euler1.y());
		eulerx.z() = euler1.z() - r*(euler2.z() - euler1.z());
	}

	else {
		r = (tx - t1) / (t2 - t1);
		eulerx.x() = euler1.x() + r*(euler2.x() - euler1.x());
		eulerx.y() = euler1.y() + r*(euler2.y() - euler1.y());
		eulerx.z() = euler1.z() + r*(euler2.z() - euler1.z());
	}

//	eulerx.x() = normalizeAngle(eulerx.x());
//	eulerx.y() = normalizeAngle(eulerx.y());
//	eulerx.z() = normalizeAngle(eulerx.z());

	return fromRPY(eulerx.x(), eulerx.y(), eulerx.z());
}



Trajectory::Trajectory()
{
	// TODO Auto-generated constructor stub

}

Trajectory::~Trajectory()
{
	// TODO Auto-generated destructor stub
}

